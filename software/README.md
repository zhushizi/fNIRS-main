# fNIRS Single-Channel Software

This directory contains the host-side software for the **single-channel** fNIRS workflow.  
It targets the new **26-byte framed serial protocol** and assumes one physical channel, `S1_D1`, sampled under two wavelengths (`660nm` and `940nm`).

## Core Features

- **Framed serial protocol**
  - `0x55 0xAA` frame header
  - fixed `26B` frame length
  - checksum validation
  - `0x03` ACK handling with retry support

- **Single-channel acquisition**
  - one intensity value per data frame
  - wavelength code and sensor id included in the payload
  - raw capture stored in `all_groups.csv`

- **Single-channel processing**
  - threshold filtering
  - low-pass filtering
  - segment RMS by wavelength code
  - 660/940 pairing
  - MBLL + CBSI

- **Visualization**
  - `adc_live.py`: live ADC plot for 660nm / 940nm
  - `adc_animation.py`: replay raw CSV
  - `mBLL_animation.py`: replay processed CSV
  - `visualizer.py`: lightweight control dashboard

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
| 1 | intensity high byte |
| 2 | intensity low byte |
| 3-20 | reserved, currently `0x00` |

### Data Payload (`0x02`)

| Byte | Meaning |
|------|---------|
| 0 | wavelength code (`0x00=660`, `0x01=940`) |
| 1 | sensor id (`0x00` for current S1_D1 deployment) |
| 2 | sampled value low byte |
| 3 | sampled value high byte |
| 4-19 | reserved |
| 20 | reserved, currently fixed `0x00` |

Here, the payload byte `0` wavelength code is also the effective
"emitter-state" meaning for the host side:

- `0x00` means the current sample belongs to `660nm`
- `0x01` means the current sample belongs to `940nm`

So downstream processing no longer depends on a separate emitter field.

### ACK Handling

- timeout: `10ms`
- retries: `2`

## Configuration

Edit `config.py` before running:

- `SERIAL_PORT`
- `BAUD_RATE`
- `TIMEOUT`
- `DEFAULT_INTENSITY_MA`
- `SOURCE_DETECTOR_DISTANCE_CM`

## Main Scripts

### `fNIRS_processing.py`

Captures raw frames, acknowledges incoming data frames, writes `all_groups.csv`, creates `interleaved_output.csv`, and computes `processed_output.csv`.

### `adc_live.py`

Starts the stream and displays 660nm / 940nm values live on one plot.

### `visualizer.py`

Runs a minimal Flask dashboard at `http://127.0.0.1:8050` for:

- start/stop commands
- intensity updates
- latest packet inspection
- ACK status feedback

Run demo mode with:

```bash
python visualizer.py demo
```

## CSV Outputs

### `all_groups.csv`

```text
Time (s),SensorId,S1_D1,Wavelength
```

### `interleaved_output.csv`

```text
Time (s),S1_D1_660,S1_D1_940
```

### `processed_output.csv`

```text
Time,S1_D1_hbo,S1_D1_hbr
```
