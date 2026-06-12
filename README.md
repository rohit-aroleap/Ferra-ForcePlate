# Ferra Force Plate

The **Ferra Balance** game, ported from the IMU wobble board to a **4-load-cell force plate**. An ESP32 reads four HX711 load-cell amplifiers, computes the **centre of pressure (CoP)**, and streams it over **BLE** to a tablet running a single-file HTML game. The plate suits people whose balance is too weak for the wobble board — and, because it also reports total weight, it can tell when nobody is standing on it.

```
4 × load cells ──HX711──► ESP32 ──BLE notify (16-byte frame)──► Tablet (index.html — the game)
                                ◄──── text commands (ZERO …) ───
```

This is the sibling of [IMU-BalanceBoard](https://github.com/rohit-aroleap/IMU-BalanceBoard): the dashboard is a fork of that game, and the BLE frame keeps the same 16-byte shape so the game code is shared almost verbatim. The only real difference is the input — **centre-of-pressure in centimetres** instead of tilt in degrees — plus a weight signal the wobble board never had.

## Repo layout

| Branch | Contents |
|---|---|
| `main` | `FerraForcePlate/` (ESP32 Arduino sketch) + `dashboard/index.html` (game source) + this README |
| `gh-pages` | `index.html` — the deployed game, served at the live URL below |

- **Live game:** https://rohit-aroleap.github.io/Ferra-ForcePlate/

## 1. Flash the firmware

The sketch is in `FerraForcePlate/`. With `arduino-cli` (esp32 core installed) and the plate connected over USB:

```
arduino-cli compile --fqbn "esp32:esp32:esp32:PartitionScheme=huge_app" -u -p COM<N> FerraForcePlate
```

Find the port with `Get-CimInstance Win32_PnPEntity | ? { $_.Name -match '\(COM\d+\)' -and $_.Name -notmatch 'Bluetooth' }` (the ESP32 shows as CP210x/CH340). The `huge_app` partition is required — Bluedroid BLE overflows the default 1.2 MB app partition.

On boot the serial log (115200 baud) prints the BLE device name and starts advertising.

## 2. Calibrate once (over USB serial)

Calibration is a serial-only, interactive 2-point wizard and only needs redoing when the hardware changes (values persist in EEPROM). Open a serial monitor at 115200 baud and send `c`; it asks for two known weights in grams and walks through all four cells. Other serial/BLE commands:

| Command | Effect |
|---|---|
| `c` | Full 2-point calibration (serial only) |
| `t` / `ZERO` | Tare all connected cells |
| `t1`–`t4` | Tare a single cell (1=FL 2=FR 3=BL 4=BR) |
| `s` / `STATUS` | Print status JSON |
| `start` / `stop` | Start / stop CoP streaming |
| `d` | Re-scan for load cells |
| `l` / `r` | One-shot calibrated grams / raw ADC |
| `x` / `x1`–`x4` | Reset calibration |

Once all connected cells are calibrated the plate streams automatically on BLE connect — no `start` needed.

## 3. Play

Open the live URL on a Chrome/Edge tablet (Web Bluetooth requires Chrome/Edge over HTTPS or `localhost`; `file://` won't work). Tap **Connect**, pick **FerraPlate**, step on. The same games as the wobble board run on your weight-shift: hold-the-zone, orbit, pop, and calm modes, with adaptive difficulty, ranks, daily challenge, and a shared cloud leaderboard.

If nobody is on the plate (total load < 15 kg) the game pauses and the dot parks at centre — so an empty plate can't run up a score.

## BLE contract

- Device name **`FerraPlate`**, one custom service (distinct UUID base `7e40000x` so it never clashes with the IMU board or the Ferra strength machine in the BLE picker).
- **Data** characteristic (`…0002`, NOTIFY): a 16-byte little-endian frame `{ uint32 ms, float copX_cm, float copY_cm, float weight_kg }`.
- **Command** characteristic (`…0003`, WRITE): plain-text commands (`ZERO`, `START`, `STOP`, `STATUS`).
- On connect the firmware requests a **7.5 ms connection interval** — this is the fix for the ~0.5 s dot lag the IMU board hit; without it the host picks a slow interval and the notify stream batches/drops.

## Centre-of-pressure & game scale

CoP from the four calibrated corner forces (FL/FR/BL/BR), plate geometry 339.411 × 339.411 mm:

```
copX = (plateW/2) · ((FR+BR) − (FL+BL)) / ΣF     // +x = lean right
copY = (plateH/2) · ((FL+FR) − (BL+BR)) / ΣF     // +y = lean forward
```

reported in **cm**, lightly EMA-smoothed on-device. The game plot is **±12 cm**; the ring/score constants are the IMU board's proportions rescaled ×0.4 (degrees → cm). Because the score rate is a ratio (`SCORE_REF / radius`), the rescale preserves the difficulty and scoring curve exactly — leaderboard scores stay comparable in meaning.

**These cm values are first-guess and meant to be tuned on the physical plate.** The knobs live together near the top of the game-logic block in `dashboard/index.html`:

- `maxAngle` (plot half-size, cm)
- `START_RING_DEG`, `MIN_RING_DEG`, `MAX_RING_DEG`, `SCORE_REF_DEG` (ring sizes, cm — names keep the `_DEG` suffix to avoid churn; read them as cm)
- `RANKS[].max` and the `rateSkill()` thresholds (rank ring sizes, cm)
- `STAND_MIN_KG` (step-off threshold)
- On the firmware side: `COP_EMA_ALPHA` and `WEIGHT_ON_THRESHOLD_G` in `FerraForcePlate/config.h`

## CSV format

Recording downloads `plate_<timestamp>.csv`:

```
timestamp, elapsed_s, device_ms, cop_x_cm, cop_y_cm, weight_kg, game_phase, ring_cm, inside, score
```

## Hardware

- ESP32 DevKit V1 (any BLE-capable ESP32).
- 4 × HX711 amplifiers — DOUT/CLK pins `16/4, 17/5, 25/18, 26/19` for FL/FR/BL/BR (see `FerraForcePlate/config.h`).
- 4 load cells in a 339.411 mm-square plate. HX711 RATE pin tied HIGH → 80 SPS; firmware samples at 40 Hz.
