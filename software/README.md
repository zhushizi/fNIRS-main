# fNIRS Dual-Detector Five-Wavelength Software

This directory contains the host-side software for the **dual-detector, five-wavelength** fNIRS workflow.
It targets the **26-byte framed serial protocol** carried over an **Android TCP bridge**, and assumes two
receiver channels — `S1_D1` (long, 3.0 cm) and `S1_D2` (short, 1.0 cm) — each sampled under five emitter
wavelengths (`850 / 810 / 770 / 730 / 700 nm`).

The Android app owns the UART (start/stop), forwards raw serial bytes to the PC over TCP/JSON, and the PC
acts as a passive TCP **server** that decodes frames, runs analysis, and streams `HbO / HbR / Cyt / rSO₂`
results back to the phone for live plotting.

## Core Features

- **Framed serial protocol** (`protocol.py`)
  - `0x55 0xAA` frame header
  - fixed `26B` frame length
  - checksum validation
  - current firmware does not ACK; `send_frame_with_ack` skips the ACK wait

- **Android TCP bridge** (`online_android/tcp_bridge.py`)
  - PC listens on `HOST_TCP_DEFAULT_PORT` (default `9000`)
  - Android sends `serial_data` (base64-encoded raw frames); PC exposes a serial-like `read()`
  - PC sends back `live_analysis_batch` and `analysis_result`

- **Dual-detector acquisition**
  - one intensity value per data frame, tagged with detector id and wavelength code
  - raw capture stored in `all_groups.csv`

- **Processing**
  - per-(detector, wavelength) low-pass filtering
  - segment RMS by `(DetectorId, Wavelength)` runs
  - 2 detectors × 5 wavelengths pairing into an intensity wide table
  - intensities → OD changes → robust band-pass (with decimation)
  - **short-separation regression (SSR)**: short detector regressed out of the long detector
  - **stepwise generalized MBLL**: solve HbO/HbR first, then estimate Cyt from the residual projection
  - **rSO₂** estimation from ΔHbO/ΔHbR over a fixed baseline window

- **Online + offline**
  - online: background worker re-runs the full session each tick and streams full series to Android
  - offline: after capture finishes, a final pass writes `processed_output.csv`

## Protocol Summary

### Common Frame Layout

| Field | Size |
|------|------|
| Header | 2 bytes (`0x55 0xAA`) |
| Length | 1 byte (`0x1A`) |
| Type | 1 byte |
| Payload | 21 bytes |
| Checksum | 1 byte |

### Frame Types

- `0x01`: command frame
- `0x02`: data frame
- `0x03`: ACK frame

### Command Payload (`0x01`)

| Byte | Meaning |
|------|---------|
| 0 | start/stop (`0x00` stop, `0x01` start) |
| 1 | intensity (`0x00`~`0xFF`) |
| 2 | channel/command code |
| 3-20 | reserved, currently `0x00` |

### Data Payload (`0x02`)

| Byte | Meaning |
|------|---------|
| 0 | wavelength code (`0x00=off`, `0x01=850`, `0x02=810`, `0x03=770`, `0x04=730`, `0x05=700` nm) |
| 1 | detector id (`0x01=S1_D1`, `0x02=S1_D2`) |
| 2-5 | sampled value, 4-byte big-endian unsigned |
| 6-20 | reserved |

`Wavelength=0x00` (off) rows are dropped before wavelength pairing.

## Configuration

Edit `config.py` before running:

- `SERIAL_PORT`, `BAUD_RATE` (legacy single-channel serial scripts only)
- `HOST_TCP_LISTEN_HOST`, `HOST_TCP_DEFAULT_PORT` (Android TCP bridge)
- `DETECTOR_CHANNELS` (names, codes, source-detector distances)
- `WAVELENGTH_CHANNELS` (codes + emitter/MBLL wavelengths)
- `OUTPUT_CHANNEL` (`S1_D1` | `S1_D2` | `S1_D1_ssr`, default `S1_D1_ssr`)
- `BP_LOW_HZ` / `BP_HIGH_HZ` / `BP_ORDER` (OD band-pass)

`config.py` validates these tables on import (`_validate_wavelength_channels`, `_validate_output_channel`).

## Main Entry Point

### `fNIRS_processing.py`

Thin CLI wrapper. Parses TCP options and calls `fnirs_pipeline.run_pipeline`, which:

1. opens the Android TCP bridge and waits for a client,
2. captures raw frames into `all_groups.csv` while a background worker streams live analysis,
3. builds `interleaved_output.csv` (paired intensities),
4. runs SSR + generalized MBLL and writes `processed_output.csv`,
5. sends a final `analysis_result` to Android.

```bash
python fNIRS_processing.py                 # capture + online + offline
python fNIRS_processing.py --live_plot     # also mirror the Android stream on a PC plot
python fNIRS_processing.py --tcp_debug     # print TCP rx/tx logs
```

## CSV Outputs

Written under `result_table/<timestamp>/`.

### `all_groups.csv`

```text
Time (s),DetectorId,Channel,Wavelength,Value
```

### `interleaved_output.csv`

```text
Time (s),S1_D1_850,S1_D1_810,S1_D1_770,S1_D1_730,S1_D1_700,S1_D2_850,...,S1_D2_700
```

### `processed_output.csv`

Column prefix follows `OUTPUT_CHANNEL` (default `S1_D1_ssr`):

```text
Time,S1_D1_ssr_hbo,S1_D1_ssr_hbr,S1_D1_ssr_cyt
```

### `android_live_output.csv`

A copy of the series actually streamed to Android during online analysis.

## Module Layout

- `fnirs_pipeline/` — offline/online pipeline
  - `pipeline.py`: `run_pipeline` orchestration
  - `capture.py`: TCP capture + online session wiring
  - `preprocessing.py`: low-pass, RMS segmentation, wavelength-cycle aggregation
  - `mbll.py`: OD, band-pass, SSR, generalized MBLL, rSO₂ summary
- `online_android/` — Android-facing live analysis
  - `tcp_bridge.py`: Host TCP protocol (server side)
  - `worker.py` / `session.py` / `buffer.py`: background analysis lifecycle
  - `batch_builder.py` / `reporter.py` / `batch_recorder.py`: build, send, record live batches
  - `rso2.py`: rSO₂ from ΔHbO/ΔHbR
  - `live_plotter.py`: optional PC-side mirror plot

## Legacy / Single-Channel Scripts

These come from the earlier single-channel 660/940 design and are kept for reference or offline replay;
some still talk directly to a local serial port and are **not** part of the current TCP main path:
`adc_live.py`, `adc_animation.py`, `mBLL_animation.py`, `visualizer.py` (+ `index.html`),
`hbo_hbr_live.py`, `data_analysis.py`, `nirs_viewer.py`, `rso2_from_processed.py`.
