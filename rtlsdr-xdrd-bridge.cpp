#include <functional>
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <cstdint>
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
constexpr float    FM_AUDIO_GAIN           = 4200.0f;
constexpr float    AM_AUDIO_GAIN           = 7500.0f;

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

// ----------------------------------------------------------------------------
//                      STEREO DECODER CONFIGURATION
// ----------------------------------------------------------------------------

// Print stereo/pilot statistics once per second (useful for tuning thresholds)
constexpr bool     STEREO_DEBUG            = false;

// The stereo (L+R / L-R) part is processed at IF_RATE / STEREO_DECIM = 64 kHz
constexpr int      STEREO_DECIM            = 4;
constexpr uint32_t STEREO_RATE             = IF_RATE / STEREO_DECIM;
static_assert(IF_RATE % STEREO_DECIM == 0, "IF_RATE must be a multiple of STEREO_DECIM");

// Pilot PLL: corner of the low-pass that follows the quadrature mixer (Hz).
// Narrow => very low noise in the phase detector.
constexpr double   PILOT_MIXER_LPF_HZ      = 300.0;

// PLL loop natural frequency (Hz): wide while searching, narrow once locked
constexpr double   PLL_BW_ACQUIRE_HZ       = 40.0;
constexpr double   PLL_BW_TRACK_HZ         = 12.0;

// Max deviation of the PLL frequency from 19 kHz (Hz). Covers RTL ppm error + transmitter tolerance.
constexpr double   PLL_PULL_RANGE_HZ       = 25.0;

// Pilot amplitude thresholds (1.0 == 75 kHz deviation; broadcast pilot is ~0.07..0.10)
constexpr double   PILOT_LOCK_ON_AMP       = 0.040;
constexpr double   PILOT_LOCK_OFF_AMP      = 0.028;

// Stereo blend control by estimated L-R signal-to-noise ratio (dB):
//   below STEREO_SNR_MIN_DB  -> pure mono
//   above STEREO_SNR_FULL_DB -> full stereo
// If weak stations still hiss  -> raise both numbers (e.g. 22 / 34).
// If stereo drops too early    -> lower them   (e.g. 14 / 26).
constexpr double   STEREO_SNR_MIN_DB       = 10.0;
constexpr double   STEREO_SNR_FULL_DB      = 22.0;

// Blend smoothing: fast fall (to mono), slow rise (to stereo) - prevents "pumping"
constexpr float    STEREO_ATTACK_MS        = 20.0f;
constexpr float    STEREO_RELEASE_MS       = 250.0f;

// Dynamic high-cut applied to the L-R (difference) channel:
// fc = MIN + SPAN * blend^2. Weak signal -> L-R is band-limited, hiss disappears.
constexpr float    LR_HICUT_MIN_HZ         = 3000.0f;
constexpr float    LR_HICUT_SPAN_HZ        = 40000.0f;

// Manual trim of the 38 kHz carrier phase in radians (normally 0).
// Use only if separation is poor on ALL strong stations.
constexpr double   STEREO_PHASE_TRIM_RAD   = 0.0;

// Noise measurement band (empty part of MPX, above RDS, below the IF filter edge)
constexpr double   NOISE_BAND_CENTER_HZ    = 72500.0;
constexpr double   NOISE_BAND_WIDTH_HZ     = 15000.0;

// Converts power measured in that band to the noise power inside the 23..53 kHz L-R band
// (FM noise spectrum is parabolic: ~ f^2).
constexpr double   LR_NOISE_FACTOR         = 0.37;

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
// DSP HELPERS FOR THE STEREO DECODER
// ============================================================================

// Modified Bessel function I0 (for Kaiser window)
static double bessel_i0(double x) {
    double sum = 1.0, term = 1.0;
    const double q = x * x * 0.25;
    for (int k = 1; k < 60; ++k) {
        term *= q / (double(k) * double(k));
        sum += term;
        if (term < 1e-14 * sum) break;
    }
    return sum;
}

// Linear-phase Kaiser-windowed-sinc low-pass with EXACTLY unity DC gain.
// fc is the -6 dB point, normalized to the sample rate (0 .. 0.5).
static std::vector<float> design_kaiser_lowpass(int n, double fc, double atten_db) {
    double beta;
    if (atten_db > 50.0)      beta = 0.1102 * (atten_db - 8.7);
    else if (atten_db > 21.0) beta = 0.5842 * std::pow(atten_db - 21.0, 0.4) + 0.07886 * (atten_db - 21.0);
    else                      beta = 0.0;

    std::vector<double> h(n);
    const double M = (n - 1) / 2.0;
    const double i0b = bessel_i0(beta);
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        const double t = i - M;
        const double s = (std::fabs(t) < 1e-12) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * t) / (M_PI * t);
        const double r = (M > 0.0) ? t / M : 0.0;
        const double w = bessel_i0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
        h[i] = s * w;
        sum += h[i];
    }
    std::vector<float> out(n);
    for (int i = 0; i < n; ++i) out[i] = float(h[i] / sum);
    return out;
}

// Measures the DC gain of a liquid real FIR filter (used only for loudness compatibility)
static float measure_dc_gain(firfilt_rrrf f, unsigned int taps) {
    float y = 0.0f;
    for (unsigned int i = 0; i < taps * 2 + 8; ++i) {
        firfilt_rrrf_push(f, 1.0f);
        firfilt_rrrf_execute(f, &y);
    }
    return y;
}

// Double-precision biquad (narrow filters at 256 kHz need double precision)
struct Biquad {
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
    double z1 = 0.0, z2 = 0.0;

    void reset() { z1 = z2 = 0.0; }

    inline double process(double x) {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    void design_lowpass(double fs, double fc, double q) {
        const double w0 = 2.0 * M_PI * fc / fs;
        const double cw = std::cos(w0), sw = std::sin(w0);
        const double alpha = sw / (2.0 * q);
        const double a0 = 1.0 + alpha;
        b0 = ((1.0 - cw) * 0.5) / a0;
        b1 = (1.0 - cw) / a0;
        b2 = b0;
        a1 = (-2.0 * cw) / a0;
        a2 = (1.0 - alpha) / a0;
    }

    // Constant 0 dB peak gain band-pass (RBJ cookbook)
    void design_bandpass(double fs, double f0, double q) {
        const double w0 = 2.0 * M_PI * f0 / fs;
        const double cw = std::cos(w0), sw = std::sin(w0);
        const double alpha = sw / (2.0 * q);
        const double a0 = 1.0 + alpha;
        b0 = alpha / a0;
        b1 = 0.0;
        b2 = -alpha / a0;
        a1 = (-2.0 * cw) / a0;
        a2 = (1.0 - alpha) / a0;
    }
};

// ============================================================================
// 19 kHz PILOT TRACKER (PLL) + 38 kHz CARRIER GENERATOR + NOISE ESTIMATOR
//
//  * The mixer output is low-passed with a very narrow filter BEFORE the phase
//    detector, so audio / RDS / noise do not jitter the phase.
//  * The phase detector is amplitude-normalized (e = Im / |z|), so the loop gain
//    does not depend on the pilot level.
//  * The 38 kHz carrier is taken from EXACTLY the same phase that was used for
//    the pilot mixer in the same sample (the old code used the phase one sample
//    later = ~53 degrees of error on the 38 kHz carrier).
// ============================================================================
class PilotTracker {
    double fs = 1.0;
    double phase = 0.0;
    double w = 0.0, w_nom = 0.0, w_lim = 0.0;
    double kp_acq = 0.0, ki_acq = 0.0, kp_trk = 0.0, ki_trk = 0.0;
    double trim_c = 1.0, trim_s = 0.0;
    double k_amp = 0.0, k_err = 0.0, k_noise = 0.0;
    Biquad lp_re, lp_im, bp_noise;
    double re_avg = 0.0;       // average of Re(z) == pilot_amplitude / 2 when locked
    double err_var = 1.0;      // average of phase-error^2
    double noise_pow = 1e-4;   // power in the noise measurement band

    static void loop_gains(double fn_hz, double fs_hz, double& kp, double& ki) {
        const double wn = 2.0 * M_PI * fn_hz / fs_hz;
        kp = 2.0 * 0.7071 * wn;
        ki = wn * wn;
    }

public:
    bool locked = false;

    void init(double fs_hz) {
        fs = fs_hz;
        w_nom = 2.0 * M_PI * 19000.0 / fs;
        w_lim = 2.0 * M_PI * PLL_PULL_RANGE_HZ / fs;
        lp_re.design_lowpass(fs, PILOT_MIXER_LPF_HZ, 0.7071);
        lp_im = lp_re;
        bp_noise.design_bandpass(fs, NOISE_BAND_CENTER_HZ, NOISE_BAND_CENTER_HZ / NOISE_BAND_WIDTH_HZ);
        loop_gains(PLL_BW_ACQUIRE_HZ, fs, kp_acq, ki_acq);
        loop_gains(PLL_BW_TRACK_HZ,   fs, kp_trk, ki_trk);
        trim_c = std::cos(STEREO_PHASE_TRIM_RAD);
        trim_s = std::sin(STEREO_PHASE_TRIM_RAD);
        k_amp   = 1.0 / (0.040 * fs);   // 40 ms
        k_err   = 1.0 / (0.040 * fs);   // 40 ms
        k_noise = 1.0 / (0.150 * fs);   // 150 ms
        reset();
    }

    void reset() {
        w = w_nom;
        lp_re.reset();
        lp_im.reset();
        bp_noise.reset();
        re_avg = 0.0;
        err_var = 1.0;
        noise_pow = 1e-4;
        locked = false;
    }

    // Processes one MPX sample, returns sin(2*phi_pilot) - the 38 kHz subcarrier reference
    float step(float x_in) {
        const double x  = x_in;
        const double sn = std::sin(phase);
        const double cs = std::cos(phase);

        // Quadrature mixer: x = A*sin(theta)  ->  Re = A/2*cos(d), Im = A/2*sin(d), d = theta - phase
        const double re  = lp_re.process(x * sn);
        const double im  = lp_im.process(x * cs);
        const double mag = std::sqrt(re * re + im * im);
        const double e   = (mag > 1e-7) ? (im / mag) : 0.0;

        re_avg  += k_amp   * (re - re_avg);
        err_var += k_err   * (e * e - err_var);
        const double nb = bp_noise.process(x);
        noise_pow += k_noise * (nb * nb - noise_pow);

        // 38 kHz carrier from the SAME phase as the pilot mixer (sin(2*phase + trim))
        const double s2 = 2.0 * sn * cs;
        const double c2 = cs * cs - sn * sn;
        const double carrier = s2 * trim_c + c2 * trim_s;

        // Lock detector with hysteresis
        const double amp = 2.0 * re_avg;
        const double err_rms = std::sqrt(std::max(0.0, err_var));
        if (!locked) {
            if (amp > PILOT_LOCK_ON_AMP && err_rms < 0.30) locked = true;
        } else {
            if (amp < PILOT_LOCK_OFF_AMP || err_rms > 0.55) locked = false;
        }

        // PI loop
        const double kp = locked ? kp_trk : kp_acq;
        const double ki = locked ? ki_trk : ki_acq;
        w += ki * e;
        w = std::clamp(w, w_nom - w_lim, w_nom + w_lim);
        phase += w + kp * e;
        if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
        else if (phase < 0.0)    phase += 2.0 * M_PI;

        return float(carrier);
    }

    double pilot_amplitude() const { return 2.0 * re_avg; }

    // Estimated SNR (dB) of the L-R channel, using pilot power as the reference
    double snr_db() const {
        const double a = 2.0 * re_avg;
        if (a <= 1e-6) return -100.0;
        const double p = 0.5 * a * a;
        const double n = LR_NOISE_FACTOR * noise_pow + 1e-12;
        return 10.0 * std::log10(p / n);
    }
};

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

    // ---- IF stage / RDS wideband path / AM path (unchanged) ----
    firdecim_crcf decim         = firdecim_crcf_create_kaiser(DECIM_FACTOR, 16, 65.0f);
    firfilt_crcf  if_filter     = firfilt_crcf_create_kaiser(41, 190000.0f / (2.0f * IF_RATE), 60.0f, 0.0f);
    firfilt_crcf  rds_if_filter = firfilt_crcf_create_kaiser(41, 200000.0f / (2.0f * IF_RATE), 60.0f, 0.0f);
    freqdem       fmdem         = freqdem_create(75000.0f / IF_RATE);
    freqdem       rds_fmdem     = freqdem_create(75000.0f / IF_RATE);

    firfilt_rrrf am_audio_filter = firfilt_rrrf_create_kaiser(63, 6000.0f / IF_RATE, 60.0f, 0.0f);

    msresamp_rrrf resamp_L   = msresamp_rrrf_create(float(AUDIO_RATE) / float(IF_RATE), 60.0f);   // AM path
    msresamp_rrrf resamp_R   = msresamp_rrrf_create(float(AUDIO_RATE) / float(IF_RATE), 60.0f);   // AM path
    msresamp_rrrf resamp_mpx = msresamp_rrrf_create(float(REDSEA_RATE) / float(IF_RATE), 60.0f);

    // ---- NEW STEREO DECODER ----
    PilotTracker pilot;
    pilot.init(double(IF_RATE));

    // Stage 1: polyphase decimation 256 kHz -> 64 kHz for both the sum (L+R) and the
    // mixed-down difference (L-R) signals. Unity DC gain, stop band starts at ~45 kHz so that
    // 57 kHz RDS and the 76 kHz image cannot alias into the audio band.
    std::vector<float> taps_dec = design_kaiser_lowpass(49, 34500.0 / double(IF_RATE), 65.0);
    firdecim_rrrf dec_sum = firdecim_rrrf_create(STEREO_DECIM, taps_dec.data(), (unsigned int)taps_dec.size());
    firdecim_rrrf dec_dif = firdecim_rrrf_create(STEREO_DECIM, taps_dec.data(), (unsigned int)taps_dec.size());

    // Stage 2: sharp audio low-pass at 64 kHz: flat up to 15 kHz, > 60 dB down at 19 kHz
    // (pilot and the RDS-mixing product are removed completely).
    std::vector<float> taps_aud = design_kaiser_lowpass(65, 17000.0 / double(STEREO_RATE), 65.0);
    firfilt_rrrf aud_sum = firfilt_rrrf_create(taps_aud.data(), (unsigned int)taps_aud.size());
    firfilt_rrrf aud_dif = firfilt_rrrf_create(taps_aud.data(), (unsigned int)taps_aud.size());

    msresamp_rrrf resamp_fm_L = msresamp_rrrf_create(float(AUDIO_RATE) / float(STEREO_RATE), 60.0f);
    msresamp_rrrf resamp_fm_R = msresamp_rrrf_create(float(AUDIO_RATE) / float(STEREO_RATE), 60.0f);

    // Loudness compatibility with the previous version: the old L+R / L-R filters were liquid
    // Kaiser filters; whatever their DC gain was, FM_AUDIO_GAIN was tuned for it. We measure it
    // once and apply the same factor, so the volume stays exactly as before.
    float legacy_dc_gain = 1.0f;
    {
        firfilt_rrrf legacy = firfilt_rrrf_create_kaiser(63, 15000.0f / IF_RATE, 60.0f, 0.0f);
        legacy_dc_gain = measure_dc_gain(legacy, 63);
        firfilt_rrrf_destroy(legacy);
        if (!std::isfinite(legacy_dc_gain) || legacy_dc_gain < 0.05f || legacy_dc_gain > 50.0f) legacy_dc_gain = 1.0f;
    }
    const float fm_gain_scaled = FM_AUDIO_GAIN * legacy_dc_gain;

    float stereo_blend = 0.0f;       // current smoothed L-R mix amount (0 = mono, 1 = full stereo)
    float lr_hc1 = 0.0f, lr_hc2 = 0.0f;   // L-R dynamic high-cut states
    const float blend_a_att = 1.0f - std::exp(-1.0f / (STEREO_ATTACK_MS  * 0.001f * float(STEREO_RATE)));
    const float blend_a_rel = 1.0f - std::exp(-1.0f / (STEREO_RELEASE_MS * 0.001f * float(STEREO_RATE)));

    float dt = 1.0f / AUDIO_RATE;
    float alpha_deemph = dt / (DEEMPHASIS_TAU + dt);
    float deemph_L = 0.0f, deemph_R = 0.0f;
    float dc_mpx = 0.0f;
    float dc_rds_mpx = 0.0f;
    float am_dc = 0.0f;
    float am_agc = 0.1f;

    constexpr int FFT_SIZE = 1024;
    std::vector<std::complex<float>> fft_in(FFT_SIZE);
    std::vector<std::complex<float>> fft_out(FFT_SIZE);
    std::vector<float> fft_win(FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; ++i) {
        fft_win[i] = blackmanharris(i, FFT_SIZE);
    }
    fftplan scan_fft = fft_create_plan(FFT_SIZE, fft_in.data(), fft_out.data(), LIQUID_FFT_FORWARD, 0);

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
    std::cout << "[STEREO] PLL stereo decoder active (legacy gain factor " << legacy_dc_gain << ")\n";

    float signal_power = 0.0f;
    float noise_power  = 0.001f;
    int   stat_timer   = 0;
    int64_t last_diag_ms = now_ms();
    int64_t last_stereo_dbg_ms = now_ms();

    std::vector<std::complex<float>> iq_in;
    std::vector<std::complex<float>> iq_decim;
    std::vector<float> rds_mpx_256;
    std::vector<float> audio_L_256;      // AM path only
    std::vector<float> audio_R_256;      // AM path only
    std::vector<float> fm_mpx_256;       // FM: demodulated MPX
    std::vector<float> fm_sub_256;       // FM: MPX * 2 * sin(2*pilot_phase)  (L-R + images)
    std::vector<float> sum_dec;          // FM @ 64 kHz
    std::vector<float> dif_dec;
    std::vector<float> sum_flt;
    std::vector<float> dif_flt;
    std::vector<float> fm_L_64;
    std::vector<float> fm_R_64;
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

        if (STEREO_DEBUG && current_ms - last_stereo_dbg_ms >= 1000) {
            last_stereo_dbg_ms = current_ms;
            std::cout << "[STEREO] lock=" << (pilot.locked ? 1 : 0)
                      << " pilotAmp=" << std::fixed << std::setprecision(3) << pilot.pilot_amplitude()
                      << " snr=" << std::setprecision(1) << pilot.snr_db() << " dB"
                      << " blend=" << std::setprecision(2) << stereo_blend << "\n";
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
            pilot.reset();
            stereo_blend = 0.0f;
            lr_hc1 = lr_hc2 = 0.0f;
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
        fm_mpx_256.resize(if_samples);
        fm_sub_256.resize(if_samples);

        uint32_t cf = current_freq.load();
        int m_ov = mod_override.load();
        bool use_am = (m_ov == 1) || (m_ov == -1 && is_auto_am_band(cf));
        bool in_fm_rds_band = (!use_am && cf >= 65000000 && cf <= 108500000);

        int mute_rem = soft_mute_samples.load();
        const int mute_start = mute_rem;

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

            if (use_am) {
                float env = std::abs(s_filt);
                am_dc = sanitize_float(0.999f * am_dc + 0.001f * env, 0.0f);
                float ac = env - am_dc;
                am_agc = sanitize_float(0.9995f * am_agc + 0.0005f * std::max(am_dc, 0.005f), 0.1f);
                float norm_am = std::clamp(ac / am_agc, -2.0f, 2.0f);

                float af = 0.0f;
                firfilt_rrrf_push(am_audio_filter, norm_am);
                firfilt_rrrf_execute(am_audio_filter, &af);
                af = sanitize_float(af);

                float out_l_sample = af;
                float out_r_sample = af;

                if (mute_rem > 0) {
                    float atten = 1.0f - (float(mute_rem) / float(IF_RATE / 80 + 1));
                    atten = std::clamp(atten, 0.15f, 1.0f);
                    out_l_sample *= atten;
                    out_r_sample *= atten;
                    mute_rem--;
                }

                audio_L_256[i] = out_l_sample;
                audio_R_256[i] = out_r_sample;
            } else {
                // ---- FM demodulation -> MPX ----
                float mpx_raw = 0.0f;
                freqdem_demodulate(fmdem, s_filt, &mpx_raw);
                mpx_raw = sanitize_float(mpx_raw);
                dc_mpx = sanitize_float(0.9995f * dc_mpx + 0.0005f * mpx_raw);
                float mpx = mpx_raw - dc_mpx;

                // Pilot PLL returns sin(2*phase) - the 38 kHz reference phase-aligned with this sample
                float carrier_38k = pilot.step(mpx);

                fm_mpx_256[i] = mpx;
                fm_sub_256[i] = 2.0f * mpx * carrier_38k;
            }
        }

        // ------------------------------------------------------------------
        // FM stereo post-processing @ 64 kHz
        // ------------------------------------------------------------------
        size_t n64 = 0;
        if (!use_am) {
            n64 = if_samples / STEREO_DECIM;
            if (n64 > 0) {
                sum_dec.resize(n64);
                dif_dec.resize(n64);
                sum_flt.resize(n64);
                dif_flt.resize(n64);
                fm_L_64.resize(n64);
                fm_R_64.resize(n64);

                firdecim_rrrf_execute_block(dec_sum, fm_mpx_256.data(), (unsigned int)n64, sum_dec.data());
                firdecim_rrrf_execute_block(dec_dif, fm_sub_256.data(), (unsigned int)n64, dif_dec.data());
                firfilt_rrrf_execute_block(aud_sum, sum_dec.data(), (unsigned int)n64, sum_flt.data());
                firfilt_rrrf_execute_block(aud_dif, dif_dec.data(), (unsigned int)n64, dif_flt.data());

                // Target stereo amount from pilot quality and estimated L-R SNR
                float target_blend = 0.0f;
                if (!force_mono.load() && pilot.locked) {
                    double q = (pilot.snr_db() - STEREO_SNR_MIN_DB) / (STEREO_SNR_FULL_DB - STEREO_SNR_MIN_DB);
                    q = std::clamp(q, 0.0, 1.0);
                    target_blend = float(q * q * (3.0 - 2.0 * q));   // smoothstep
                }

                for (size_t j = 0; j < n64; ++j) {
                    const float a = (target_blend < stereo_blend) ? blend_a_att : blend_a_rel;
                    stereo_blend += a * (target_blend - stereo_blend);

                    // Dynamic high-cut on L-R: weak signal -> narrow, strong -> transparent
                    const float fc = LR_HICUT_MIN_HZ + LR_HICUT_SPAN_HZ * stereo_blend * stereo_blend;
                    const float hc_alpha = 1.0f - std::exp(-2.0f * float(M_PI) * fc / float(STEREO_RATE));
                    lr_hc1 += hc_alpha * (sanitize_float(dif_flt[j]) - lr_hc1);
                    lr_hc2 += hc_alpha * (lr_hc1 - lr_hc2);

                    const float S = sanitize_float(sum_flt[j]);
                    const float D = lr_hc2 * stereo_blend;

                    float l = S + D;
                    float r = S - D;

                    // Soft-mute after tuning (ramp measured in IF-rate samples)
                    const int rem = mute_start - int(j * STEREO_DECIM);
                    if (rem > 0) {
                        float atten = 1.0f - (float(rem) / float(IF_RATE / 80 + 1));
                        atten = std::clamp(atten, 0.15f, 1.0f);
                        l *= atten;
                        r *= atten;
                    }

                    fm_L_64[j] = l;
                    fm_R_64[j] = r;
                }
            }
            mute_rem = std::max(0, mute_start - int(if_samples));
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

        unsigned int num_L = 0, num_R = 0;
        if (use_am) {
            out_L.resize(if_samples + 64);
            out_R.resize(if_samples + 64);
            msresamp_rrrf_execute(resamp_L, audio_L_256.data(), if_samples, out_L.data(), &num_L);
            msresamp_rrrf_execute(resamp_R, audio_R_256.data(), if_samples, out_R.data(), &num_R);
        } else {
            if (n64 == 0) continue;
            out_L.resize(n64 + 64);
            out_R.resize(n64 + 64);
            msresamp_rrrf_execute(resamp_fm_L, fm_L_64.data(), (unsigned int)n64, out_L.data(), &num_L);
            msresamp_rrrf_execute(resamp_fm_R, fm_R_64.data(), (unsigned int)n64, out_R.data(), &num_R);
        }

        unsigned int out_frames = std::min(num_L, num_R);
        pcm_out.resize(out_frames * 2);
        float gain_mult = use_am ? AM_AUDIO_GAIN : fm_gain_scaled;

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

            // Stereo indicator for the client: pilot locked and the decoder is really mixing in L-R
            bool st = (!use_am) && (!force_mono) && pilot.locked && (stereo_blend > 0.30f);

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
    firfilt_rrrf_destroy(am_audio_filter);
    firdecim_rrrf_destroy(dec_sum);
    firdecim_rrrf_destroy(dec_dif);
    firfilt_rrrf_destroy(aud_sum);
    firfilt_rrrf_destroy(aud_dif);
    msresamp_rrrf_destroy(resamp_L);
    msresamp_rrrf_destroy(resamp_R);
    msresamp_rrrf_destroy(resamp_mpx);
    msresamp_rrrf_destroy(resamp_fm_L);
    msresamp_rrrf_destroy(resamp_fm_R);

    if (global_sdr) rtlsdr_close(global_sdr);
    return 0;
}
