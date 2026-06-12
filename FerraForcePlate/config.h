#pragma once

// ---------------------------------------------------------------------------
// HX711 pin assignments — 4 load cell corners
//
// Corner layout (viewed from above):
//   f0 = front-left    f1 = front-right
//   f2 = back-left     f3 = back-right
// ---------------------------------------------------------------------------

// NOTE: the FL and BR cables are crossed on this build (measured 2026-06-12:
// loading the physical front-left corner moved the channel on pins 26/19 and
// vice versa), so the pin map swaps those two corners in firmware instead of
// re-crimping connectors.
#define HX711_DOUT_0  26    // Front-left  DATA
#define HX711_CLK_0   19    // Front-left  CLK

#define HX711_DOUT_1  17    // Front-right DATA
#define HX711_CLK_1    5    // Front-right CLK

#define HX711_DOUT_2  25    // Back-left   DATA
#define HX711_CLK_2   18    // Back-left   CLK

#define HX711_DOUT_3  16    // Back-right  DATA
#define HX711_CLK_3    4    // Back-right  CLK

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

// HX711 RATE pin tied HIGH → 80 SPS hardware rate.
// We target 40 Hz in firmware to allow for processing headroom. 40 Hz is
// plenty for the balance game (the IMU board ran ~77 Hz off a 208 Hz IMU;
// the load cells are the rate limiter here, not the game).
#define SAMPLE_RATE_HZ   40
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)   // 25 000 µs

// ---------------------------------------------------------------------------
// Serial
// ---------------------------------------------------------------------------
#define SERIAL_BAUD  115200

// ---------------------------------------------------------------------------
// Load cell connectivity
// ---------------------------------------------------------------------------
#define NUM_CELLS                  4
#define LOADCELLS_CHANNEL_COUNT    4

// ---------------------------------------------------------------------------
// Calibration (EEPROM persistent storage)
// ---------------------------------------------------------------------------
#define CAL_NVS_NAMESPACE   "cal"
#define CAL_NVS_KEY         "data3"   // bumped from "data2" to invalidate stale zero-offset blob
#define CAL_NVS_MODE_KEY    "mode"
#define CAL_VALID_MARKER    0xCA
#define CAL_OFFSET_MIN      -8000000L
#define CAL_OFFSET_MAX       8000000L
#define CAL_SCALE_MIN        0.001f
#define CAL_SCALE_MAX        100000.0f
#define CAL_TARE_SAMPLES     20

// ---------------------------------------------------------------------------
// Force plate geometry
// ---------------------------------------------------------------------------
// Straight-line distance between left and right RSL301 cell mounting points (mm)
#define PLATE_WIDTH_MM   339.411f
// Straight-line distance between front and back RSL301 cell mounting points (mm)
#define PLATE_HEIGHT_MM  339.411f

// ---------------------------------------------------------------------------
// Centre-of-pressure → game stream
// ---------------------------------------------------------------------------
// The game treats the plate's centre-of-pressure (CoP) the way the IMU board
// treated tilt: a live (x, y) dot. We stream CoP in CENTIMETRES so the
// dashboard's plot/scoring constants are human-sized numbers.
//
// EMA smoothing on-device (matches the IMU board's α=0.3). The dashboard also
// has its own filter, but a little on-device smoothing keeps the dot calm.
#define COP_EMA_ALPHA            0.30f

// Below this total load we treat the plate as "nobody standing": CoP is
// meaningless (divide-by-small-number), so we hold the dot at centre (0,0)
// and let the dashboard pause the game. 15 kg comfortably rejects an empty
// plate / a bag set down, while admitting the lightest realistic user.
#define WEIGHT_ON_THRESHOLD_G    15000.0f

// ---------------------------------------------------------------------------
// BLE — the tablet connects here (Web Bluetooth). DISTINCT UUID base from the
// IMU board (6e40000x) and the Ferra strength machine (also 6e40000x) so the
// browser picker never confuses devices in a gym full of Ferra BLE hardware.
// Frame contract (32-byte LE; the first 16 bytes match the IMU board's struct
// so the dashboard parser reads them unchanged): uint32 ms, float copX_cm,
// float copY_cm, float weight_kg, then the four per-cell loads
// float fl_kg, fr_kg, bl_kg, br_kg.
// ---------------------------------------------------------------------------
#define BLE_DEVICE_NAME      "FerraPlate"
#define BLE_SERVICE_UUID     "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_DATA_CHAR_UUID   "7e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify (32-byte frame)
#define BLE_CMD_CHAR_UUID    "7e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write  (text commands)
#define BLE_INFO_CHAR_UUID   "7e400004-b5a3-f393-e0a9-e50e24dcca9e"  // notify (text lines: [INFO]/[CAL]/[STATUS])

// Connection-interval request (units of 1.25 ms). THE fix for ~0.5 s dot lag
// on the IMU board: hosts default slow (Windows ~47 ms, Android ~22 ms) and
// batch/drop our notify stream. Request 7.5 ms; the central has final say.
#define BLE_CONN_INTERVAL_MIN  6      // 6 * 1.25ms = 7.5 ms
#define BLE_CONN_INTERVAL_MAX  6      // hard request
#define BLE_CONN_LATENCY       0
#define BLE_CONN_TIMEOUT       400    // 400 * 10ms = 4000 ms supervision timeout
