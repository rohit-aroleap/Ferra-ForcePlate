#pragma once
#include <Arduino.h>

/**
 * Minimal HX711 24-bit ADC driver for a single load cell channel.
 *
 * The HX711 uses a proprietary serial protocol:
 *   - Pull CLK low and wait for DOUT to go low (ready signal)
 *   - Clock out 24 bits MSB-first by toggling CLK high/low
 *   - Send 1-3 additional CLK pulses to set the next channel/gain
 *
 * This driver uses Channel A at 128x gain (25 CLK pulses total).
 * Includes timeout protection (200ms) and interrupt-safe critical sections.
 */
class HX711 {
public:
  /**
   * @param doutPin  GPIO pin connected to HX711 DOUT
   * @param clkPin   GPIO pin connected to HX711 CLK (PD_SCK)
   */
  HX711(uint8_t doutPin, uint8_t clkPin);

  /** Configure GPIO pins only (does not wait for ready). */
  void beginGpio();

  /** Wait for the first reading to be available (call after beginGpio). */
  void waitReady();

  /** Configure GPIO pins and wait for the first reading to be available. */
  void begin();

  /**
   * Returns true if the HX711 has a new reading ready.
   * DOUT goes low when data is available.
   */
  bool isReady() const;

  /**
   * Block until a new reading is ready, then return the raw 24-bit signed value.
   * Applies tare offset. Returns 0 on timeout.
   */
  long read();

  /**
   * Non-blocking read. Returns last value if no new data is available.
   * More suitable for timed loops.
   */
  long readIfReady(bool &newData);

  /**
   * Tare: capture the current zero reading.
   * Call once after power-on when the plate is unloaded.
   */
  void tare(uint8_t times = 10);

  /** Return the raw (pre-tare) value for diagnostic purposes. */
  long readRaw();

  /** True if the last read attempt timed out (HX711 not responding). */
  bool timedOut() const;

  /**
   * True if the sensor was confirmed as connected during begin().
   * Uses variance across PROBE_SAMPLES reads — floating pins have no variance.
   */
  bool isConnected() const;

  /** Set the tare offset directly (used by CalibrationManager). */
  void setOffset(long offset);

  /** Get the current tare offset. */
  long getOffset() const;

private:
  uint8_t _dout;
  uint8_t _clk;
  long    _offset = 0;
  bool    _timedOut = false;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  static const unsigned long READY_TIMEOUT_US = 200000;  // 200ms

  long _readOnce();
};

/**
 * Array of 4 HX711 sensors for the 4-corner force plate.
 *
 * Corner layout (viewed from above):
 *   f0 = front-left   f1 = front-right
 *   f2 = back-left    f3 = back-right
 */
class HX711Array {
public:
  HX711Array(
    uint8_t dout0, uint8_t clk0,
    uint8_t dout1, uint8_t clk1,
    uint8_t dout2, uint8_t clk2,
    uint8_t dout3, uint8_t clk3
  );

  /** Initialize all 4 sensors (GPIO + wait for ready). Does NOT tare. */
  void begin();

  /**
   * Read all 4 sensors sequentially (blocking).
   * Blocks until all sensors have fresh data.
   * @param f Output array of 4 values [f0, f1, f2, f3]
   */
  void readAll(long f[4]);

  /**
   * Read all 4 sensors in parallel using non-blocking polls.
   * Polls all channels simultaneously and returns when ALL have a new reading.
   * @param f Output array of 4 values [f0, f1, f2, f3]
   */
  void readAllReady(long f[4]);

  /**
   * Non-blocking read of a single sensor by index.
   * Returns the latest value if new data is available, otherwise returns 0
   * and sets newData = false. Never blocks.
   * @param index  Sensor index 0-3
   * @param newData  Set to true if a fresh reading was available
   */
  long readIfReady(uint8_t index, bool &newData);

  /** Tare all 4 sensors. */
  void tare();

  /** Set tare offset for a specific sensor. */
  void setOffset(uint8_t index, long offset);

  /** Get tare offset for a specific sensor. */
  long getOffset(uint8_t index) const;

  /** True if any sensor timed out on the last readAll(). */
  bool anyTimedOut() const;

  /**
   * Returns the number of sensors that responded successfully after begin().
   * A sensor that timed out during begin() is considered not connected.
   */
  uint8_t connectedCount() const;

  /** Read a single raw ADC value (no tare offset) from sensor at index. */
  long getRaw(uint8_t index);

  /** True if the sensor at index timed out on its last read attempt. */
  bool isTimedOut(uint8_t index) const;

  /** True if the sensor at index was confirmed connected after begin()/rescan(). */
  bool isConnected(uint8_t index) const;

  /**
   * Re-run begin() on all sensors to re-detect which are connected.
   * Call connectedCount() after this to get the updated count.
   */
  void rescan();

private:
  HX711 _sensors[4];
};
