# RTL-SDR to FM-DX Webserver Bridge (C++ / Linux)

An experimental, lightweight C++ backend bridge designed to connect **RTL-SDR** (including RTL-SDR Blog V4) directly to **fm-dx-webserver** on Linux (e.g., Debian/Ubuntu) via ALSA loopback and the XDRD TCP protocol, without requiring Windows layers like SDR# or virtual COM port emulators.

> ⚠️ **Status: Unstable / Experimental (WIP)**
> This project is built for tinkering, local debugging, and remote monitoring in low-RF-noise or rural environments. Due to RTL-SDR dynamic range limitations, it is not a direct replacement for dedicated high-end hardware like TEF6686 receivers in urban areas.

---

## Table of Contents

- [What Works (Features)](#what-works-features)
- [What is Unstable / Known Issues](#what-is-unstable--known-issues)
- [Dependencies](#dependencies)
- [Compilation & Build](#compilation--build)
- [How to Use & Connect to FM-DX Webserver](#how-to-use--connect-to-fm-dx-webserver)
  - [1. Set up ALSA Loopback (Audio Pipeline)](#1-set-up-alsa-loopback-audio-pipeline)
  - [2. Run the Bridge](#2-run-the-bridge)
  - [3. Configure fm-dx-webserver](#3-configure-fm-dx-webserver)

---

## What Works (Features)

- **Direct TCP Bridge:** Implements the XDRD text protocol required by `fm-dx-webserver` (supports tuning, bandwidth/IF selection, mono/stereo toggle, and fast spectrum scanning).
- **Multi-threaded Architecture:** Non-blocking background tuner worker and continuous zero-underrun ALSA audio engine prevent stream stalls or browser dropouts during heavy DSP operations.
- **Stereo & RDS Decoding:** Built-in `liquid-dsp` multirate chain with PLL pilot-tone stereo separation and an integrated self-healing `redsea` pipeline for live RDS / PI-code extraction.
- **Self-Healing Watchdogs:** Automatic USB stream timeout recovery, ALSA buffer recovery, and child-process restarting for `redsea`.

## What is Unstable / Known Issues

- Heavy CPU usage on very weak headless mini-PCs during fast spectrum scans (`U` command).
- Audio artifacts or temporary muting can occur if the USB bus is shared or experiences heavy interrupt jitter.
- AM/LW mode logic is experimental and heavily dependent on proper antenna matching / Direct Sampling configuration.

---

## Dependencies

Make sure you have the following development libraries and tools installed on your Linux system before compiling:

- **Build essentials:** `build-essential`, `g++`, `make`
- **Librtlsdr:** `librtlsdr-dev`
- **Liquid-DSP:** `libliquid-dev`
- **ALSA:** `libasound2-dev`
- **RDS Decoder:** `redsea` (must be available in system `PATH`)

### Installation on Debian / Ubuntu

```bash
sudo apt update
sudo apt install build-essential git cmake librtlsdr-dev libliquid-dev libasound2-dev
```

> **Note:** Ensure `redsea` is also installed (e.g., via cargo or package manager) for RDS support.

---

## Compilation & Build

Assuming your source file is located at `~/rtlsdr-xdrd-bridge.cpp`, compile it with optimizations using `g++`:

```bash
g++ -O3 -std=c++17 ~/rtlsdr-xdrd-bridge.cpp -o ~/rtlsdr-xdrd-bridge -lrtlsdr -lliquid -lasound -lpthread
```

---

## How to Use & Connect to FM-DX Webserver

### 1. Set up ALSA Loopback (Audio Pipeline)

Ensure your system has the ALSA loopback module loaded so the bridge can pipe audio to streaming tools (like `ffmpeg` or Icecast):

```bash
sudo modprobe snd-aloop
```

This typically creates a device named `plughw:Loopback,0,0`.

### 2. Run the Bridge

Execute the compiled binary, specifying the ALSA device, TCP port, gain (dB), and optional PPM correction:

```bash
~/rtlsdr-xdrd-bridge plughw:Loopback,0,0 7373 25.0 0
```

### 3. Configure fm-dx-webserver

In your `fm-dx-webserver` configuration file, point the receiver/tuner backend to connect to this bridge's TCP port (default `7373`):

- **Protocol:** XDRD / TCP
- **Host:** `127.0.0.1` (or your server IP)
- **Port:** `7373`

```json
{
  "tuner_host": "127.0.0.1",
  "tuner_port": 7373
}
```
