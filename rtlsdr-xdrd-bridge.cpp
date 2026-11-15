#include <functional>
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <cstring>
#include <csignal>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/socket.h>

#include <rtl-sdr.h>
#include <liquid/liquid.h>
#include <alsa/asoundlib.h>

// ============================================================================
//                          CONFIGURATION BLOCK
// ============================================================================

// RTL-SDR capture sample rate in Hz (must be a multiple of IF_RATE)
constexpr uint32_t CAPTURE_RATE            = 1024000;

// Intermediate Frequency (IF) sample rate in Hz after initial decimation
constexpr uint32_t IF_RATE                 = 256000;

// Output ALSA audio sample rate in Hz
constexpr uint32_t AUDIO_RATE              = 48000;

// Sample rate required by the external 'redsea' RDS decoder in Hz
constexpr uint32_t REDSEA_RATE             = 171000;

// Decimation factor from CAPTURE_RATE to IF_RATE (1024000 / 256000 = 4)
constexpr int      DECIM_FACTOR            = CAPTURE_RATE / IF_RATE;

// Minimum and maximum tunable frequencies in Hz (100 kHz LW to 1.75 GHz UHF)
constexpr uint32_t MIN_TUNABLE_FREQ_HZ     = 100000;
constexpr uint32_t MAX_TUNABLE_FREQ_HZ     = 1750000000;

// Threshold frequency in Hz below which RTL-SDR Direct Sampling (Q-branch) is enabled
constexpr uint32_t DIRECT_SAMPLING_MAX_HZ  = 24000000;
constexpr int      DIRECT_SAMPLING_MODE    = 2;

// FM De-emphasis time constant in seconds (50e-6 for Europe/CIS, 75e-6 for Americas)
constexpr float    DEEMPHASIS_TAU          = 50e-6f;

// Audio output gain multipliers for FM and AM modes
constexpr float    FM_AUDIO_GAIN           = 7000.0f;
constexpr float    AM_AUDIO_GAIN           = 9500.0f;

// Signal level calibration offset and maximum allowed dBf value
// FM-DX Webserver converts dBf to dBm as (dBm = dBf - 120).
// MAX_SIGNAL_DBF = 115.0f allows up to -5.0 dBm.
constexpr float    SIGNAL_OFFSET_DBF       = 18.0f;
constexpr float    MAX_SIGNAL_DBF          = 115.0f;

// Maximum number of queued IQ buffers from USB
constexpr size_t   MAX_IQ_QUEUE_SIZE       = 24;

// Target audio ringbuffer fill level in stereo frames (4800 frames = 100ms at 48kHz)
constexpr size_t   AUDIO_RING_TARGET_FRAMES = 4800;
constexpr size_t   AUDIO_RING_MAX_FRAMES    = 14400;
constexpr size_t   ALSA_PERIOD_FRAMES       = 960; // 20ms continuous chunks

// Watchdog timeout in milliseconds: restarts USB stream if RTL-SDR stops sending data
constexpr int64_t  USB_WATCHDOG_MS         = 2500;

// Self-diagnostics health check interval in milliseconds (60 seconds)
constexpr int64_t  DIAGNOSTICS_INTERVAL_MS = 60000;

// Duration in milliseconds to mute RDS pipe after tuning to flush stale station data
constexpr int64_t  RDS_FLUSH_WINDOW_MS     = 240;

// ============================================================================

std::atomic<bool> running(true);
rtlsdr_dev_t* global_sdr = nullptr;
std::mutex sdr_mtx;

void sig_handler(int) {
    running = false;
    if (global_sdr) {
        rtlsdr_cancel_async(global_sdr);
    }
}

static inline int64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static inline float sanitize_float(float v, float fallback = 0.0f) {
    return std::isfinite(v) ? v : fallback;
}

static inline std::complex<float> sanitize_complex(std::complex<float> v) {
    if (!std::isfinite(v.real()) || !std::isfinite(v.imag())) {
        return {0.0f, 0.0f};
    }
    return v;
}

// ============================================================================
// CONTINUOUS ZERO-UNDERRUN ALSA AUDIO ENGINE (PREVENTS BROWSER STREAM DROPS)
// ============================================================================
class ContinuousAlsaEngine {
    std::string dev_name;
    snd_pcm_t* pcm = nullptr;
    std::mutex ring_mtx;
    std::deque<int16_t> ring;
    std::thread worker;
    int16_t last_l = 0;
    int16_t last_r = 0;

public:
    std::atomic<uint64_t> underrun_recoveries{0};
    std::atomic<uint64_t> synthetic_frames_fed{0};
    std::atomic<uint64_t> reopen_count{0};

    bool start(const std::string& device) {
        dev_name = device;
        if (!open_device()) return false;
        worker = std::thread(&ContinuousAlsaEngine::audio_loop, this);
        return true;
    }

    void stop() {
        if (worker.joinable()) worker.join();
        if (pcm) {
            snd_pcm_close(pcm);
            pcm = nullptr;
        }
    }

    void push_stereo(const int16_t* interleaved_data, size_t frames) {
        if (frames == 0) return;
        std::lock_guard<std::mutex> lock(ring_mtx);

        size_t current_frames = ring.size() / 2;
        if (current_frames + frames > AUDIO_RING_MAX_FRAMES) {
            size_t exceso = (current_frames + frames) - AUDIO_RING_TARGET_FRAMES;
            size_t drop_samples = std::min(ring.size(), exceso * 2);
            ring.erase(ring.begin(), ring.begin() + drop_samples);
        }

        ring.insert(ring.end(), interleaved_data, interleaved_data + frames * 2);
    }

    double get_fill_ratio() {
        std::lock_guard<std::mutex> lock(ring_mtx);
        return double(ring.size() / 2) / double(AUDIO_RING_TARGET_FRAMES);
    }

private:
    bool open_device() {
        if (pcm) {
            snd_pcm_close(pcm);
            pcm = nullptr;
        }
        if (snd_pcm_open(&pcm, dev_name.c_str(), SND_PCM_STREAM_PLAYBACK, 0) < 0) {
            return false;
        }
        if (snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                               2, AUDIO_RATE, 1, 120000) < 0) {
            snd_pcm_close(pcm);
            pcm = nullptr;
            return false;
        }
        snd_pcm_prepare(pcm);
        std::vector<int16_t> prefill(ALSA_PERIOD_FRAMES * 4, 0);
        snd_pcm_writei(pcm, prefill.data(), ALSA_PERIOD_FRAMES * 2);
        return true;
    }

    void audio_loop() {
        std::vector<int16_t> period_buf(ALSA_PERIOD_FRAMES * 2, 0);

        while (running) {
            size_t got_frames = 0;
            {
                std::lock_guard<std::mutex> lock(ring_mtx);
                size_t avail_frames = ring.size() / 2;
                got_frames = std::min(avail_frames, ALSA_PERIOD_FRAMES);

                for (size_t i = 0; i < got_frames * 2; ++i) {
                    period_buf[i] = ring.front();
                    ring.pop_front();
                }
            }

            if (got_frames > 0) {
                last_l = period_buf[(got_frames - 1) * 2];
                last_r = period_buf[(got_frames - 1) * 2 + 1];
            }

            if (got_frames < ALSA_PERIOD_FRAMES) {
                synthetic_frames_fed += (ALSA_PERIOD_FRAMES - got_frames);
                for (size_t i = got_frames; i < ALSA_PERIOD_FRAMES; ++i) {
                    last_l = int16_t(float(last_l) * 0.96f);
                    last_r = int16_t(float(last_r) * 0.96f);
                    period_buf[2 * i]     = last_l;
                    period_buf[2 * i + 1] = last_r;
                }
            }

            if (!pcm) {
                if (!open_device()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                reopen_count++;
            }

            snd_pcm_sframes_t written = snd_pcm_writei(pcm, period_buf.data(), ALSA_PERIOD_FRAMES);
            if (written < 0) {
                underrun_recoveries++;
                int rec = snd_pcm_recover(pcm, written, 1);
                if (rec < 0) {
                    reopen_count++;
                    open_device();
                } else {
                    snd_pcm_writei(pcm, period_buf.data(), ALSA_PERIOD_FRAMES);
                }
            }
        }
    }
};

// ============================================================================
// SELF-HEALING REDSEA RDS RUNNER
// ============================================================================
class RedseaRunner {
    int pipe_stdin[2]  = {-1, -1};
    int pipe_stdout[2] = {-1, -1};
    std::atomic<pid_t> pid{-1};
    std::thread reader_thread;
    std::mutex proc_mtx;
    std::atomic<int64_t> ignore_until_ms{0};

    bool spawn() {
        close_pipes();
        if (pipe(pipe_stdin) < 0 || pipe(pipe_stdout) < 0) {
            close_pipes();
            return false;
        }

#ifdef F_SETPIPE_SZ
        fcntl(pipe_stdin[1],  F_SETPIPE_SZ, 65536);
        fcntl(pipe_stdout[0], F_SETPIPE_SZ, 4096);
#endif

        pid_t cpid = fork();
        if (cpid < 0) {
            close_pipes();
            return false;
        }

        if (cpid == 0) {
            dup2(pipe_stdin[0], STDIN_FILENO);
            dup2(pipe_stdout[1], STDOUT_FILENO);
            close(pipe_stdin[0]); close(pipe_stdin[1]);
            close(pipe_stdout[0]); close(pipe_stdout[1]);

            setenv("PATH", "/usr/local/bin:/usr/bin:/bin", 1);

            execlp("stdbuf", "stdbuf", "-oL", "redsea", "-r", "171000", "-x", nullptr);
            execlp("redsea", "redsea", "-r", "171000", "-x", nullptr);
            execl("/usr/local/bin/redsea", "redsea", "-r", "171000", "-x", nullptr);
            execl("/usr/bin/redsea", "redsea", "-r", "171000", "-x", nullptr);
            _exit(1);
        }

        close(pipe_stdin[0]);  pipe_stdin[0]  = -1;
        close(pipe_stdout[1]); pipe_stdout[1] = -1;

        int flags = fcntl(pipe_stdin[1], F_GETFL, 0);
        if (flags >= 0) fcntl(pipe_stdin[1], F_SETFL, flags | O_NONBLOCK);

        pid = cpid;
        return true;
    }

    void close_pipes() {
        if (pipe_stdin[0] >= 0)  { close(pipe_stdin[0]);  pipe_stdin[0]  = -1; }
        if (pipe_stdin[1] >= 0)  { close(pipe_stdin[1]);  pipe_stdin[1]  = -1; }
        if (pipe_stdout[0] >= 0) { close(pipe_stdout[0]); pipe_stdout[0] = -1; }
        if (pipe_stdout[1] >= 0) { close(pipe_stdout[1]); pipe_stdout[1] = -1; }
    }

    void reap() {
        pid_t cpid = pid.exchange(-1);
        if (cpid > 0) {
            kill(cpid, SIGTERM);
            for (int i = 0; i < 10; ++i) {
                if (waitpid(cpid, nullptr, WNOHANG) != 0) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            kill(cpid, SIGKILL);
            waitpid(cpid, nullptr, 0);
        }
    }

public:
    std::function<void(const std::string&, const std::string&, const std::string&, const std::string&)> on_rds_hex;
    std::atomic<uint64_t> restarts{0};

    bool start() {
        std::lock_guard<std::mutex> lock(proc_mtx);
        if (!spawn()) return false;
        reader_thread = std::thread(&RedseaRunner::read_loop, this);
        return true;
    }

    void flush_on_tune() {
        ignore_until_ms = now_ms() + RDS_FLUSH_WINDOW_MS;
    }

    void write_mpx(const int16_t* data, size_t samples) {
        int wfd = pipe_stdin[1];
        if (wfd < 0 || samples == 0) return;

        if (now_ms() < ignore_until_ms.load()) {
            static const std::vector<int16_t> zero_mpx(4096, 0);
            size_t to_write = std::min(samples, zero_mpx.size());
            ssize_t unused = write(wfd, zero_mpx.data(), to_write * sizeof(int16_t));
            (void)unused;
            return;
        }
        ssize_t unused = write(wfd, data, samples * sizeof(int16_t));
        (void)unused;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(proc_mtx);
            close_pipes();
            reap();
        }
        if (reader_thread.joinable()) reader_thread.join();
    }

private:
    void read_loop() {
        char buf[512];
        std::string line;
        line.reserve(256);

        while (running) {
            int rfd = pipe_stdout[0];
            if (rfd < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                if (!running) break;
                std::lock_guard<std::mutex> lock(proc_mtx);
                reap();
                if (spawn()) restarts++;
                continue;
            }

            ssize_t n = read(rfd, buf, sizeof(buf) - 1);
            if (n <= 0) {
                if (!running) break;
                {
                    std::lock_guard<std::mutex> lock(proc_mtx);
                    close_pipes();
                    reap();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            for (ssize_t i = 0; i < n; ++i) {
                if (buf[i] == '\n' || buf[i] == '\r') {
                    if (now_ms() >= ignore_until_ms.load() && line.size() >= 19) {
                        std::istringstream iss(line);
                        std::string bA, bB, bC, bD;
                        if (iss >> bA >> bB >> bC >> bD) {
                            if (bA.size() == 4 && bB.size() == 4 && bC.size() == 4 && bD.size() == 4) {
                                if (on_rds_hex) on_rds_hex(bA, bB, bC, bD);
                            }
                        }
                    }
                    line.clear();
                } else if (line.size() < 256) {
                    line += buf[i];
                }
            }
        }
    }
};

// ============================================================================
// TCP SERVER (XDRD PROTOCOL)
// ============================================================================
class TcpServer {
    int server_fd = -1;
    std::vector<int> clients;
    std::mutex clients_mtx;
    std::string password;

    static void configure_client_socket(int fd) {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));

        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        int keepalive = 1;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    }

public:
    std::function<void(const std::string&, int)> on_command;

    bool start(int port, const std::string& pass) {
        password = pass;
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) return false;

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(server_fd);
            server_fd = -1;
            return false;
        }
        if (listen(server_fd, 8) < 0) {
            close(server_fd);
            server_fd = -1;
            return false;
        }

        std::thread(&TcpServer::accept_loop, this).detach();
        return true;
    }

    void stop() {
        if (server_fd >= 0) {
            shutdown(server_fd, SHUT_RDWR);
            close(server_fd);
            server_fd = -1;
        }
        std::lock_guard<std::mutex> lock(clients_mtx);
        for (int fd : clients) {
            shutdown(fd, SHUT_RDWR);
            close(fd);
        }
        clients.clear();
    }

    void broadcast(const std::string& msg) {
        std::lock_guard<std::mutex> lock(clients_mtx);
        for (auto it = clients.begin(); it != clients.end();) {
            ssize_t sent = send(*it, msg.c_str(), msg.size(), MSG_NOSIGNAL);
            if (sent < 0) {
                shutdown(*it, SHUT_RDWR);
                close(*it);
                it = clients.erase(it);
            } else {
                ++it;
            }
        }
    }

    void send_to(int fd, const std::string& msg) {
        send(fd, msg.c_str(), msg.size(), MSG_NOSIGNAL);
    }

private:
    void accept_loop() {
        while (running && server_fd >= 0) {
            sockaddr_in caddr{};
            socklen_t clen = sizeof(caddr);
            int cfd = accept(server_fd, (struct sockaddr*)&caddr, &clen);
            if (cfd < 0) {
                if (!running) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            configure_client_socket(cfd);
            std::thread(&TcpServer::client_handler, this, cfd).detach();
        }
    }

    void client_handler(int cfd) {
        bool authed = password.empty();
        if (authed) {
            {
                std::lock_guard<std::mutex> lock(clients_mtx);
                clients.push_back(cfd);
            }
            send_to(cfd, "o1,0\n");
            if (on_command) on_command("INIT", cfd);
        } else {
            send_to(cfd, "s0000000000000000\n");
        }

        char buf[256];
        std::string line;
        line.reserve(256);

        while (running) {
            ssize_t n = recv(cfd, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            for (ssize_t i = 0; i < n; ++i) {
                if (buf[i] == '\n' || buf[i] == '\r') {
                    if (!line.empty()) {
                        if (!authed) {
                            authed = true;
                            {
                                std::lock_guard<std::mutex> lock(clients_mtx);
                                clients.push_back(cfd);
                            }
                            send_to(cfd, "a1\no1,0\n");
                            if (on_command) on_command("INIT", cfd);
                        } else if (on_command) {
                            on_command(line, cfd);
                        }
                        line.clear();
                    }
                } else if (line.size() < 256) {
                    line += buf[i];
                }
            }
        }

        std::lock_guard<std::mutex> lock(clients_mtx);
        clients.erase(std::remove(clients.begin(), clients.end(), cfd), clients.end());
        shutdown(cfd, SHUT_RDWR);
        close(cfd);
    }
};

// ============================================================================
// RTL-SDR STREAM & ASYNC TUNER HELPERS
// ============================================================================
std::mutex iq_mtx;
std::condition_variable iq_cv;
std::deque<std::vector<uint8_t>> iq_queue;
std::atomic<int64_t> last_usb_packet_ms{0};

void rtlsdr_callback(unsigned char* buf, uint32_t len, void* /*ctx*/) {
    if (!running || len == 0) return;
    last_usb_packet_ms = now_ms();
    std::lock_guard<std::mutex> lock(iq_mtx);
    if (iq_queue.size() >= MAX_IQ_QUEUE_SIZE) {
        iq_queue.pop_front();
    }
    iq_queue.emplace_back(buf, buf + len);
    iq_cv.notify_one();
}

void set_nearest_gain(rtlsdr_dev_t* dev, float target_db) {
    if (!dev) return;
    if (target_db < 0.0f) {
        rtlsdr_set_tuner_gain_mode(dev, 0);
        rtlsdr_set_agc_mode(dev, 1);
        return;
    }

    rtlsdr_set_agc_mode(dev, 0);
    rtlsdr_set_tuner_gain_mode(dev, 1);

    int count = rtlsdr_get_tuner_gains(dev, nullptr);
    if (count <= 0) return;

    std::vector<int> gains(count);
    rtlsdr_get_tuner_gains(dev, gains.data());

    int target_tenths = int(std::round(target_db * 10.0f));
    int best_gain = gains[0];
    int min_diff = std::abs(gains[0] - target_tenths);

    for (int g : gains) {
        int diff = std::abs(g - target_tenths);
        if (diff < min_diff) {
            min_diff = diff;
            best_gain = g;
        }
    }

    rtlsdr_set_tuner_gain(dev, best_gain);
}

bool apply_sdr_frequency(rtlsdr_dev_t* dev, uint32_t freq_hz, int& current_ds_mode, float gain_db, int agc_idx) {
    if (!dev) return false;
    std::lock_guard<std::mutex> lock(sdr_mtx);

    freq_hz = std::clamp(freq_hz, MIN_TUNABLE_FREQ_HZ, MAX_TUNABLE_FREQ_HZ);
    int desired_ds = (freq_hz < DIRECT_SAMPLING_MAX_HZ) ? DIRECT_SAMPLING_MODE : 0;

    if (desired_ds != current_ds_mode) {
        rtlsdr_set_direct_sampling(dev, desired_ds);
        current_ds_mode = desired_ds;
        if (desired_ds == 0) {
            if (agc_idx < 0) set_nearest_gain(dev, gain_db);
            else {
                const float g_map[] = {35.0f, 25.0f, 14.0f, 5.0f};
                set_nearest_gain(dev, g_map[std::clamp(agc_idx, 0, 3)]);
            }
        } else {
            rtlsdr_set_agc_mode(dev, 1);
        }
    }

    return (rtlsdr_set_center_freq(dev, freq_hz) == 0);
}

bool is_auto_am_band(uint32_t freq_hz) {
    if (freq_hz < 30000000) return true;
    if (freq_hz >= 118000000 && freq_hz <= 137000000) return true;
    return false;
}

bool parse_int_safe(const std::string& s, int64_t& out) {
    const char* p = s.c_str();
    while (*p && (*p < '0' || *p > '9') && *p != '-') ++p;
    if (!*p) return false;
    char* endp = nullptr;
    out = std::strtoll(p, &endp, 10);
    return (endp != p);
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================
int main(int argc, char** argv) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    std::string alsa_dev = (argc > 1) ? argv[1] : "plughw:Loopback,0,0";
    int tcp_port         = (argc > 2) ? std::stoi(argv[2]) : 7373;
    float gain_db        = (argc > 3) ? std::stof(argv[3]) : 25.0f;
    int ppm_error        = (argc > 4) ? std::stoi(argv[4]) : 0;

    ContinuousAlsaEngine alsa_engine;
    if (!alsa_engine.start(alsa_dev)) {
        std::cerr << "[ALSA] Failed to initialize continuous audio engine on: " << alsa_dev << "\n";
        return 1;
    }

    if (rtlsdr_open(&global_sdr, 0) < 0) {
        std::cerr << "[RTL-SDR] Failed to open device #0!\n";
        alsa_engine.stop();
        return 1;
    }

    rtlsdr_set_sample_rate(global_sdr, CAPTURE_RATE);
    if (ppm_error != 0) rtlsdr_set_freq_correction(global_sdr, ppm_error);

    std::atomic<uint32_t> current_freq(87500000);
    std::atomic<uint32_t> target_freq(87500000);
    std::atomic<bool>     tune_pending(false);
    std::atomic<bool>     dsp_reset_on_tune(false);
    std::atomic<int>      soft_mute_samples(0);

    std::atomic<bool>     force_mono(false);
    std::atomic<float>    target_bw_hz(190000.0f);
    std::atomic<bool>     bw_changed(false);
    std::atomic<int>      mod_override(-1);
    std::atomic<int>      agc_mode_idx(-1);

    std::atomic<uint32_t> scan_start_khz(87500);
    std::atomic<uint32_t> scan_stop_khz(108000);
    std::atomic<uint32_t> scan_step_khz(50);
    std::atomic<uint32_t> scan_bw_hz(56000);
    std::atomic<bool>     scan_requested(false);
    std::atomic<bool>     scanning_active(false);

    int current_ds_mode = 0;
    apply_sdr_frequency(global_sdr, current_freq.load(), current_ds_mode, gain_db, -1);
    set_nearest_gain(global_sdr, gain_db);
    rtlsdr_reset_buffer(global_sdr);

    std::mutex tune_cv_mtx;
    std::condition_variable tune_cv;
    std::thread tuner_worker([&]() {
        while (running) {
            {
                std::unique_lock<std::mutex> lock(tune_cv_mtx);
                tune_cv.wait_for(lock, std::chrono::milliseconds(100), [&] {
                    return tune_pending.load() || !running;
                });
            }
            if (!running) break;
            if (scanning_active.load()) continue;

            if (tune_pending.exchange(false)) {
                uint32_t nf = target_freq.load();
                apply_sdr_frequency(global_sdr, nf, current_ds_mode, gain_db, agc_mode_idx.load());
                current_freq = nf;
                soft_mute_samples = IF_RATE / 80;
                dsp_reset_on_tune = true;
            }
        }
    });

    firdecim_crcf decim         = firdecim_crcf_create_kaiser(DECIM_FACTOR, 16, 65.0f);
    firfilt_crcf  if_filter     = firfilt_crcf_create_kaiser(41, 190000.0f / (2.0f * IF_RATE), 60.0f, 0.0f);
    firfilt_crcf  rds_if_filter = firfilt_crcf_create_kaiser(41, 200000.0f / (2.0f * IF_RATE), 60.0f, 0.0f);
    freqdem       fmdem         = freqdem_create(75000.0f / IF_RATE);
    freqdem       rds_fmdem     = freqdem_create(75000.0f / IF_RATE);

    firfilt_rrrf lpr_filter      = firfilt_rrrf_create_kaiser(63, 15000.0f / IF_RATE, 60.0f, 0.0f);
    firfilt_rrrf lmr_filter      = firfilt_rrrf_create_kaiser(63, 15000.0f / IF_RATE, 60.0f, 0.0f);
    firfilt_rrrf am_audio_filter = firfilt_rrrf_create_kaiser(63, 6000.0f / IF_RATE, 60.0f, 0.0f);

    iirfilt_crcf pilot_metric_lpf = iirfilt_crcf_create_lowpass(2, 120.0f / IF_RATE);
    nco_crcf pilot_nco = nco_crcf_create(LIQUID_VCO);
    nco_crcf_set_frequency(pilot_nco, 2.0f * M_PI * 19000.0f / IF_RATE);
    nco_crcf_pll_set_bandwidth(pilot_nco, 0.0015f);

    msresamp_rrrf resamp_L   = msresamp_rrrf_create(float(AUDIO_RATE) / float(IF_RATE), 60.0f);
    msresamp_rrrf resamp_R   = msresamp_rrrf_create(float(AUDIO_RATE) / float(IF_RATE), 60.0f);
    msresamp_rrrf resamp_mpx = msresamp_rrrf_create(float(REDSEA_RATE) / float(IF_RATE), 60.0f);

    constexpr int FFT_SIZE = 1024;
    std::vector<std::complex<float>> fft_in(FFT_SIZE);
    std::vector<std::complex<float>> fft_out(FFT_SIZE);
    std::vector<float> fft_win(FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; ++i) {
        fft_win[i] = blackmanharris(i, FFT_SIZE);
    }
    fftplan scan_fft = fft_create_plan(FFT_SIZE, fft_in.data(), fft_out.data(), LIQUID_FFT_FORWARD, 0);

    float dt = 1.0f / AUDIO_RATE;
    float alpha_deemph = dt / (DEEMPHASIS_TAU + dt);
    float deemph_L = 0.0f, deemph_R = 0.0f;
    float dc_mpx = 0.0f;
    float dc_rds_mpx = 0.0f;
    float am_dc = 0.0f;
    float am_agc = 0.1f;

    TcpServer tcp;
    RedseaRunner redsea;

    std::mutex rds_state_mtx;
    std::string confirmed_pi = "0000";
    std::string candidate_pi = "0000";
    int candidate_pi_count   = 0;
    int rds_group_counter    = 0;

    redsea.on_rds_hex = [&](const std::string& bA, const std::string& bB, const std::string& bC, const std::string& bD) {
        uint32_t cf = current_freq.load();
        if (cf < 65000000 || cf > 108500000) return;

        std::lock_guard<std::mutex> lock(rds_state_mtx);

        if (bA != "----" && bA != "0000") {
            if (bA == confirmed_pi) {
                candidate_pi_count = 0;
            } else if (confirmed_pi == "0000") {
                if (bA == candidate_pi || (bB != "----" && bC != "----" && bD != "----")) {
                    confirmed_pi = bA;
                    tcp.broadcast("P" + confirmed_pi + "\n");
                } else {
                    candidate_pi = bA;
                    candidate_pi_count = 1;
                    return;
                }
            } else {
                if (bA == candidate_pi) {
                    candidate_pi_count++;
                    if (candidate_pi_count >= 3) {
                        confirmed_pi = bA;
                        candidate_pi_count = 0;
                        tcp.broadcast("P" + confirmed_pi + "\n");
                    } else {
                        return;
                    }
                } else {
                    candidate_pi = bA;
                    candidate_pi_count = 1;
                    return;
                }
            }
        }

        if (confirmed_pi == "0000" || bB == "----") return;

        uint8_t err = 0;
        std::string cC = bC;
        std::string cD = bD;

        if (bC == "----") { cC = "0000"; err |= (3 << 2); }
        if (bD == "----") { cD = "0000"; err |= 3; }

        if (bC == "----" && bD == "----") return;
        if ((bB[0] == '0') && bD == "----") return;

        tcp.broadcast("P" + confirmed_pi + "\n");

        std::ostringstream ss;
        ss << "R" << bB << cC << cD
           << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (int)err << "\n";
        tcp.broadcast(ss.str());

        if (++rds_group_counter % 15 == 1) {
            std::cout << "[RDS -> Webserver] P" << confirmed_pi << " | " << ss.str();
        }
    };

    if (!redsea.start()) {
        std::cerr << "[RDS] Warning: could not start redsea!\n";
    } else {
        std::cout << "[RDS] Dedicated wideband redsea pipeline started.\n";
    }

    tcp.on_command = [&](const std::string& cmd, int cfd) {
        if (cmd.empty()) return;
        if (cmd == "INIT") {
            tcp.send_to(cfd, "T" + std::to_string(current_freq / 1000) + "\n");
            tcp.send_to(cfd, std::string("B") + (force_mono ? "1" : "0") + "\n");
        } else if (cmd[0] == 'T') {
            int64_t f_val = 0;
            if (parse_int_safe(cmd.substr(1), f_val)) {
                if (f_val < 100) f_val = 87500;
                uint64_t hz = (f_val > 2000000) ? uint64_t(f_val) : uint64_t(f_val) * 1000ULL;
                hz = std::clamp<uint64_t>(hz, MIN_TUNABLE_FREQ_HZ, MAX_TUNABLE_FREQ_HZ);
                uint32_t f_khz = uint32_t(hz / 1000ULL);

                redsea.flush_on_tune();
                {
                    std::lock_guard<std::mutex> lock(rds_state_mtx);
                    confirmed_pi = "0000";
                    candidate_pi = "0000";
                    candidate_pi_count = 0;
                }
                target_freq = uint32_t(hz);
                tune_pending = true;
                tune_cv.notify_one();
                tcp.broadcast("T" + std::to_string(f_khz) + "\n");
            }
        } else if (cmd[0] == 'M') {
            int64_t m_val = 0;
            if (parse_int_safe(cmd.substr(1), m_val)) {
                mod_override = (m_val == 1) ? 1 : 0;
                tcp.broadcast("M" + std::to_string(m_val == 1 ? 1 : 0) + "\n");
            }
        } else if (cmd[0] == 'B') {
            if (cmd == "B0") {
                force_mono = false;
            } else if (cmd == "B1") {
                force_mono = !force_mono;
            }
            tcp.broadcast("B" + std::string(force_mono ? "1" : "0") + "\n");
        } else if (cmd[0] == 'F' || cmd[0] == 'W') {
            int64_t val = -1;
            if (!parse_int_safe(cmd.substr(1), val) || val <= 0) {
                target_bw_hz = 190000.0f;
            } else if (val < 50) {
                const float xdr_bw[] = {56000,64000,72000,84000,97000,114000,133000,151000,168000,184000,200000,217000,236000,254000};
                int idx = std::clamp(int(val), 0, 13);
                target_bw_hz = xdr_bw[idx];
            } else if (val <= 300) {
                target_bw_hz = std::clamp(float(val) * 1000.0f, 10000.0f, 240000.0f);
            } else {
                target_bw_hz = std::clamp(float(val), 10000.0f, 240000.0f);
            }
            bw_changed = true;
            tcp.broadcast(cmd + "\n");
        } else if (cmd[0] == 'S') {
            if (cmd.size() >= 2 && cmd[1] == 'a') {
                int64_t v = 87500;
                if (parse_int_safe(cmd.substr(2), v)) scan_start_khz = uint32_t(v);
            } else if (cmd.size() >= 2 && cmd[1] == 'b') {
                int64_t v = 108000;
                if (parse_int_safe(cmd.substr(2), v)) scan_stop_khz = uint32_t(v);
            } else if (cmd.size() >= 2 && cmd[1] == 's') {
                int64_t v = 50;
                if (parse_int_safe(cmd.substr(2), v) && v >= 10) scan_step_khz = uint32_t(v);
            } else if (cmd.size() >= 2 && cmd[1] == 'w') {
                int64_t v = 56000;
                if (parse_int_safe(cmd.substr(2), v) && v > 0) {
                    scan_bw_hz = (v <= 300) ? uint32_t(v * 1000) : uint32_t(v);
                }
            } else if (cmd == "S" || (cmd.size() >= 2 && (cmd[1] == 'm' || cmd[1] == 'S'))) {
                scan_requested = true;
            }
        } else if (cmd[0] == 'Z' || cmd[0] == 'G' || cmd[0] == 'A') {
            tcp.broadcast(cmd + "\n");
        }
    };

    if (!tcp.start(tcp_port, "")) {
        std::cerr << "[TCP] Failed to start server on port " << tcp_port << "\n";
        running = false;
        tune_cv.notify_all();
        if (tuner_worker.joinable()) tuner_worker.join();
        redsea.stop();
        rtlsdr_close(global_sdr);
        alsa_engine.stop();
        return 1;
    }

    std::atomic<uint64_t> usb_restarts{0};
    last_usb_packet_ms = now_ms();
    std::thread usb_thread([&]() {
        while (running) {
            int r = rtlsdr_read_async(global_sdr, rtlsdr_callback, nullptr, 16, 32768);
            if (!running) break;
            usb_restarts++;
            std::cerr << "[SELF-HEAL] RTL-SDR async stream exited (code " << r << "), recovering...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            {
                std::lock_guard<std::mutex> lock(sdr_mtx);
                rtlsdr_reset_buffer(global_sdr);
            }
        }
    });

    std::cout << "[OK] DSP bridge running (Zero-Underrun Audio + Non-Blocking Tuner)! TCP port: "
              << tcp_port << ", ALSA: " << alsa_dev << "\n";

    std::complex<float> pilot_lock_avg(0.0f, 0.0f);
    bool stereo_locked = false;
    float signal_power = 0.0f;
    float noise_power  = 0.001f;
    int   stat_timer   = 0;
    int64_t last_diag_ms = now_ms();

    std::vector<std::complex<float>> iq_in;
    std::vector<std::complex<float>> iq_decim;
    std::vector<float> rds_mpx_256;
    std::vector<float> audio_L_256;
    std::vector<float> audio_R_256;
    std::vector<float> mpx_171;
    std::vector<int16_t> mpx_pcm;
    std::vector<float> out_L;
    std::vector<float> out_R;
    std::vector<int16_t> pcm_out;

    while (running) {
        int64_t current_ms = now_ms();

        if (current_ms - last_usb_packet_ms.load() > USB_WATCHDOG_MS) {
            std::cerr << "[SELF-HEAL] USB stream timeout detected, triggering async restart...\n";
            last_usb_packet_ms = current_ms;
            rtlsdr_cancel_async(global_sdr);
        }

        if (current_ms - last_diag_ms >= DIAGNOSTICS_INTERVAL_MS) {
            last_diag_ms = current_ms;
            std::cout << "[DIAGNOSTICS] Status: OK | Freq: " << (current_freq.load() / 1000) << " kHz"
                      << " | AudioRing: " << int(alsa_engine.get_fill_ratio() * 100.0) << "%"
                      << " | ALSA Recoveries: " << alsa_engine.underrun_recoveries.load()
                      << " | USB Restarts: " << usb_restarts.load()
                      << " | RDS Restarts: " << redsea.restarts.load() << "\n";
        }

        if (scan_requested.exchange(false)) {
            scanning_active = true;
            uint32_t saved_freq = target_freq.load();
            uint32_t f_start = scan_start_khz.load();
            uint32_t f_stop  = scan_stop_khz.load();
            uint32_t f_step  = std::clamp(scan_step_khz.load(), 10u, 200u);
            int half_bins = std::clamp(int(scan_bw_hz.load() / 2500), 8, 35);

            std::ostringstream scan_ss;
            scan_ss << "U";
            bool first_pt = true;

            constexpr uint32_t WINDOW_SPAN_KHZ = 750;
            uint32_t cur_k = f_start;

            while (cur_k <= f_stop && running) {
                uint32_t center_khz = cur_k + (WINDOW_SPAN_KHZ / 2);
                apply_sdr_frequency(global_sdr, center_khz * 1000, current_ds_mode, gain_db, agc_mode_idx.load());

                std::this_thread::sleep_for(std::chrono::milliseconds(25));
                std::vector<uint8_t> sbuf;
                {
                    std::unique_lock<std::mutex> lock(iq_mtx);
                    iq_queue.clear();
                    iq_cv.wait_for(lock, std::chrono::milliseconds(45), [] { return !iq_queue.empty() || !running; });
                    if (!iq_queue.empty()) {
                        sbuf = std::move(iq_queue.back());
                        iq_queue.clear();
                    }
                }

                std::vector<float> psd(FFT_SIZE, 0.0f);
                int blocks_avg = 0;

                if (sbuf.size() >= FFT_SIZE * 2) {
                    size_t total_samples = sbuf.size() / 2;
                    std::complex<float> dc_avg(0.0f, 0.0f);
                    for (size_t i = 0; i < total_samples; ++i) {
                        dc_avg += std::complex<float>((sbuf[2 * i] - 127.4f) / 128.0f,
                                                      (sbuf[2 * i + 1] - 127.4f) / 128.0f);
                    }
                    dc_avg /= float(total_samples);

                    for (size_t offset = 0; offset + FFT_SIZE <= total_samples; offset += FFT_SIZE) {
                        for (int i = 0; i < FFT_SIZE; ++i) {
                            std::complex<float> iq((sbuf[2 * (offset + i)] - 127.4f) / 128.0f,
                                                   (sbuf[2 * (offset + i) + 1] - 127.4f) / 128.0f);
                            fft_in[i] = (iq - dc_avg) * fft_win[i];
                        }
                        fft_execute(scan_fft);
                        for (int i = 0; i < FFT_SIZE; ++i) {
                            psd[i] += std::norm(fft_out[i]);
                        }
                        blocks_avg++;
                    }
                }

                if (blocks_avg > 0) {
                    float norm_factor = 1.0f / (float(blocks_avg) * float(FFT_SIZE));
                    for (int i = 0; i < FFT_SIZE; ++i) psd[i] *= norm_factor;
                    psd[0] = 0.5f * (psd[1] + psd[FFT_SIZE - 1]);
                }

                uint32_t win_end_khz = std::min(f_stop, cur_k + WINDOW_SPAN_KHZ);
                while (cur_k <= win_end_khz) {
                    int offset_khz = int(cur_k) - int(center_khz);
                    float chan_power = 0.0f;
                    for (int b = -half_bins; b <= half_bins; ++b) {
                        int bin = (offset_khz + b + FFT_SIZE) % FFT_SIZE;
                        chan_power += psd[bin];
                    }
                    chan_power /= float(2 * half_bins + 1);

                    float rssi_dbf = 10.0f * std::log10(chan_power + 1e-10f) + 82.0f;
                    rssi_dbf = std::clamp(sanitize_float(rssi_dbf), 2.0f, MAX_SIGNAL_DBF);

                    if (!first_pt) scan_ss << ",";
                    first_pt = false;
                    scan_ss << cur_k << "=" << std::fixed << std::setprecision(1) << rssi_dbf;

                    cur_k += f_step;
                }
            }

            scan_ss << "\n";
            tcp.broadcast(scan_ss.str());

            redsea.flush_on_tune();
            apply_sdr_frequency(global_sdr, saved_freq, current_ds_mode, gain_db, agc_mode_idx.load());
            current_freq = saved_freq;
            tcp.broadcast("T" + std::to_string(saved_freq / 1000) + "\n");
            scanning_active = false;
            continue;
        }

        if (dsp_reset_on_tune.exchange(false)) {
            nco_crcf_set_frequency(pilot_nco, 2.0f * M_PI * 19000.0f / IF_RATE);
            pilot_lock_avg = {0.0f, 0.0f};
            stereo_locked = false;
        }

        if (bw_changed.exchange(false)) {
            firfilt_crcf_destroy(if_filter);
            float cutoff = std::clamp(target_bw_hz.load() / (2.0f * IF_RATE), 0.015f, 0.46f);
            if_filter = firfilt_crcf_create_kaiser(41, cutoff, 60.0f, 0.0f);
        }

        std::vector<uint8_t> raw_buf;
        {
            std::unique_lock<std::mutex> lock(iq_mtx);
            iq_cv.wait_for(lock, std::chrono::milliseconds(50), [] { return !iq_queue.empty() || !running; });
            if (!running) break;
            if (iq_queue.empty()) continue;
            raw_buf = std::move(iq_queue.front());
            iq_queue.pop_front();
        }

        size_t samples = raw_buf.size() / 2;
        size_t if_samples = samples / DECIM_FACTOR;
        if (if_samples == 0) continue;

        iq_in.resize(samples);
        iq_decim.resize(if_samples);

        for (size_t i = 0; i < samples; ++i) {
            iq_in[i] = std::complex<float>((raw_buf[2 * i] - 127.4f) / 128.0f,
                                           (raw_buf[2 * i + 1] - 127.4f) / 128.0f);
        }

        firdecim_crcf_execute_block(decim, iq_in.data(), if_samples, iq_decim.data());

        rds_mpx_256.resize(if_samples);
        audio_L_256.resize(if_samples);
        audio_R_256.resize(if_samples);

        uint32_t cf = current_freq.load();
        int m_ov = mod_override.load();
        bool use_am = (m_ov == 1) || (m_ov == -1 && is_auto_am_band(cf));
        bool in_fm_rds_band = (!use_am && cf >= 65000000 && cf <= 108500000);

        int mute_rem = soft_mute_samples.load();

        for (size_t i = 0; i < if_samples; ++i) {
            std::complex<float> s = sanitize_complex(iq_decim[i]);
            float p_raw = std::norm(s);

            if (in_fm_rds_band) {
                std::complex<float> s_rds;
                firfilt_crcf_push(rds_if_filter, s);
                firfilt_crcf_execute(rds_if_filter, &s_rds);
                float rds_raw = 0.0f;
                freqdem_demodulate(rds_fmdem, s_rds, &rds_raw);
                rds_raw = sanitize_float(rds_raw);
                dc_rds_mpx = sanitize_float(0.9995f * dc_rds_mpx + 0.0005f * rds_raw);
                rds_mpx_256[i] = rds_raw - dc_rds_mpx;
            }

            std::complex<float> s_filt;
            firfilt_crcf_push(if_filter, s);
            firfilt_crcf_execute(if_filter, &s_filt);
            s_filt = sanitize_complex(s_filt);
            float p_sig = std::norm(s_filt);

            signal_power = sanitize_float(0.9995f * signal_power + 0.0005f * p_sig, 0.0f);
            noise_power  = sanitize_float(0.9995f * noise_power  + 0.0005f * std::max(1e-7f, p_raw - p_sig), 0.001f);

            float out_l_sample = 0.0f;
            float out_r_sample = 0.0f;

            if (use_am) {
                stereo_locked = false;
                float env = std::abs(s_filt);
                am_dc = sanitize_float(0.999f * am_dc + 0.001f * env, 0.0f);
                float ac = env - am_dc;
                am_agc = sanitize_float(0.9995f * am_agc + 0.0005f * std::max(am_dc, 0.005f), 0.1f);
                float norm_am = std::clamp(ac / am_agc, -2.0f, 2.0f);

                float af = 0.0f;
                firfilt_rrrf_push(am_audio_filter, norm_am);
                firfilt_rrrf_execute(am_audio_filter, &af);
                af = sanitize_float(af);

                out_l_sample = af;
                out_r_sample = af;
            } else {
                float mpx_raw = 0.0f;
                freqdem_demodulate(fmdem, s_filt, &mpx_raw);
                mpx_raw = sanitize_float(mpx_raw);
                dc_mpx = sanitize_float(0.9995f * dc_mpx + 0.0005f * mpx_raw);
                float mpx = mpx_raw - dc_mpx;

                std::complex<float> p_nco;
                nco_crcf_cexpf(pilot_nco, &p_nco);
                std::complex<float> p_mix(mpx * p_nco.imag(), mpx * p_nco.real());

                std::complex<float> p_metric;
                iirfilt_crcf_execute(pilot_metric_lpf, p_mix, &p_metric);
                if (!std::isfinite(p_metric.real()) || !std::isfinite(p_metric.imag())) {
                    iirfilt_crcf_reset(pilot_metric_lpf);
                    p_metric = {0.0f, 0.0f};
                }
                pilot_lock_avg = sanitize_complex(0.999f * pilot_lock_avg + 0.001f * p_metric);

                float p_err = std::clamp(p_metric.imag() * 2.0f, -0.5f, 0.5f);
                nco_crcf_pll_step(pilot_nco, p_err);
                nco_crcf_step(pilot_nco);

                float p_freq = nco_crcf_get_frequency(pilot_nco);
                float p_nom  = 2.0f * M_PI * 19000.0f / IF_RATE;
                if (!std::isfinite(p_freq) || std::abs(p_freq - p_nom) > (2.0f * M_PI * 30.0f / IF_RATE)) {
                    nco_crcf_set_frequency(pilot_nco, p_nom);
                }

                float lock_level = pilot_lock_avg.real();
                if (!stereo_locked && lock_level > 0.022f) stereo_locked = true;
                else if (stereo_locked && lock_level < 0.012f) stereo_locked = false;

                float phi19 = nco_crcf_get_phase(pilot_nco);
                float carrier_38k = std::sin(2.0f * phi19);

                float lpr = 0.0f, lmr = 0.0f;
                firfilt_rrrf_push(lpr_filter, mpx);
                firfilt_rrrf_execute(lpr_filter, &lpr);

                firfilt_rrrf_push(lmr_filter, mpx * carrier_38k * 2.0f);
                firfilt_rrrf_execute(lmr_filter, &lmr);

                lpr = sanitize_float(lpr);
                lmr = sanitize_float(lmr);

                float stereo_blend = (!force_mono && stereo_locked)
                                   ? std::clamp((lock_level - 0.012f) * 28.0f, 0.0f, 0.85f)
                                   : 0.0f;
                lmr *= stereo_blend;

                out_l_sample = lpr + lmr;
                out_r_sample = lpr - lmr;
            }

            if (mute_rem > 0) {
                float atten = 1.0f - (float(mute_rem) / float(IF_RATE / 80 + 1));
                atten = std::clamp(atten, 0.15f, 1.0f);
                out_l_sample *= atten;
                out_r_sample *= atten;
                mute_rem--;
            }

            audio_L_256[i] = out_l_sample;
            audio_R_256[i] = out_r_sample;
        }
        soft_mute_samples = mute_rem;

        if (in_fm_rds_band) {
            mpx_171.resize(if_samples + 64);
            unsigned int num_mpx = 0;
            msresamp_rrrf_execute(resamp_mpx, rds_mpx_256.data(), if_samples, mpx_171.data(), &num_mpx);

            mpx_pcm.resize(num_mpx);
            for (unsigned int i = 0; i < num_mpx; ++i) {
                mpx_pcm[i] = (int16_t)std::clamp(sanitize_float(mpx_171[i]) * 14000.0f, -32767.0f, 32767.0f);
            }
            redsea.write_mpx(mpx_pcm.data(), num_mpx);
        }

        out_L.resize(if_samples + 64);
        out_R.resize(if_samples + 64);
        unsigned int num_L = 0, num_R = 0;
        msresamp_rrrf_execute(resamp_L, audio_L_256.data(), if_samples, out_L.data(), &num_L);
        msresamp_rrrf_execute(resamp_R, audio_R_256.data(), if_samples, out_R.data(), &num_R);

        unsigned int out_frames = std::min(num_L, num_R);
        pcm_out.resize(out_frames * 2);
        float gain_mult = use_am ? AM_AUDIO_GAIN : FM_AUDIO_GAIN;

        for (unsigned int i = 0; i < out_frames; ++i) {
            float sl = sanitize_float(out_L[i]);
            float sr = sanitize_float(out_R[i]);
            if (!use_am) {
                deemph_L = sanitize_float(deemph_L + alpha_deemph * (sl - deemph_L));
                deemph_R = sanitize_float(deemph_R + alpha_deemph * (sr - deemph_R));
                sl = deemph_L;
                sr = deemph_R;
            }
            float l = std::clamp(sl * gain_mult, -32767.0f, 32767.0f);
            float r = std::clamp(sr * gain_mult, -32767.0f, 32767.0f);
            pcm_out[2 * i]     = (int16_t)l;
            pcm_out[2 * i + 1] = (int16_t)r;
        }

        alsa_engine.push_stereo(pcm_out.data(), out_frames);

        if (++stat_timer >= 5) {
            stat_timer = 0;
            float snr_db = 10.0f * std::log10((signal_power + 1e-9f) / (noise_power + 1e-9f));
            float pwr_db = 10.0f * std::log10(signal_power + 1e-10f) + 85.0f;
            float level_dbf = 0.65f * pwr_db + 0.35f * (snr_db + SIGNAL_OFFSET_DBF);
            level_dbf = std::clamp(sanitize_float(level_dbf), 0.0f, MAX_SIGNAL_DBF);

            bool st = (!use_am) && (!force_mono) && stereo_locked;

            std::ostringstream ss;
            ss << "S" << (st ? "s" : "m") << std::fixed << std::setprecision(1) << level_dbf << "\n";
            tcp.broadcast(ss.str());
        }
    }

    running = false;
    tune_cv.notify_all();
    if (tuner_worker.joinable()) tuner_worker.join();

    tcp.stop();
    redsea.stop();
    if (global_sdr) rtlsdr_cancel_async(global_sdr);
    if (usb_thread.joinable()) usb_thread.join();
    alsa_engine.stop();

    fft_destroy_plan(scan_fft);
    firdecim_crcf_destroy(decim);
    firfilt_crcf_destroy(if_filter);
    firfilt_crcf_destroy(rds_if_filter);
    freqdem_destroy(fmdem);
    freqdem_destroy(rds_fmdem);
    firfilt_rrrf_destroy(lpr_filter);
    firfilt_rrrf_destroy(lmr_filter);
    firfilt_rrrf_destroy(am_audio_filter);
    iirfilt_crcf_destroy(pilot_metric_lpf);
    nco_crcf_destroy(pilot_nco);
    msresamp_rrrf_destroy(resamp_L);
    msresamp_rrrf_destroy(resamp_R);
    msresamp_rrrf_destroy(resamp_mpx);

    if (global_sdr) rtlsdr_close(global_sdr);
    return 0;
}