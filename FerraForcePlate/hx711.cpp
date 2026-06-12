#include "hx711.h"

// ---------------------------------------------------------------------------
// HX711 (single channel)
// ---------------------------------------------------------------------------

HX711::HX711(uint8_t doutPin, uint8_t clkPin)
  : _dout(doutPin), _clk(clkPin) {}

void HX711::beginGpio() {
  // INPUT_PULLUP keeps a disconnected DOUT pin HIGH (not ready).
  // A real HX711 has an open-drain DOUT that actively pulls LOW when data is
  // ready — it drives LOW against the pull-up just fine. This means the timeout
  // check below is sufficient for detection; no variance probe is needed.
  pinMode(_dout, INPUT_PULLUP);
  pinMode(_clk,  OUTPUT);
  digitalWrite(_clk, LOW);
  _timedOut = false;
}

void HX711::waitReady() {
  // Wait for the first conversion to complete (with timeout)
  unsigned long start = micros();
  while (!isReady()) {
    if (micros() - start > READY_TIMEOUT_US) {
      _timedOut = true;
      return;
    }
    delay(1);
  }
  _timedOut = false;
}

void HX711::begin() {
  beginGpio();
  waitReady();
}

bool HX711::isReady() const {
  return digitalRead(_dout) == LOW;
}

long HX711::_readOnce() {
  // Wait for DOUT to go low (ready) with timeout
  unsigned long start = micros();
  while (!isReady()) {
    if (micros() - start > READY_TIMEOUT_US) {
      _timedOut = true;
      return 0;
    }
  }
  _timedOut = false;

  long value = 0;

  // Critical section: protect the bit-bang sequence from WiFi/BT interrupts.
  // If CLK stays HIGH for >60µs the HX711 enters power-down mode.
  // The full sequence takes ~50µs — safe to disable interrupts.
  portENTER_CRITICAL(&_mux);

  // Clock in 24 bits, MSB first
  for (int i = 0; i < 24; i++) {
    digitalWrite(_clk, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(_dout);
    digitalWrite(_clk, LOW);
    delayMicroseconds(1);
  }

  // 25th pulse selects Channel A / Gain 128 for next conversion
  digitalWrite(_clk, HIGH);
  delayMicroseconds(1);
  digitalWrite(_clk, LOW);
  delayMicroseconds(1);

  portEXIT_CRITICAL(&_mux);

  // Sign-extend 24-bit to 32-bit (two's complement)
  if (value & 0x800000) {
    value |= (long)0xFF000000;
  }

  return value;
}

long HX711::readRaw() {
  return _readOnce();
}

long HX711::read() {
  return _readOnce() - _offset;
}

long HX711::readIfReady(bool &newData) {
  if (!isReady()) {
    newData = false;
    return 0;
  }
  newData = true;
  return _readOnce() - _offset;
}

void HX711::tare(uint8_t times) {
  long sum = 0;
  for (uint8_t i = 0; i < times; i++) {
    sum += _readOnce();
    if (_timedOut) return;  // abort if sensor not responding
  }
  _offset = sum / times;
}

bool HX711::timedOut() const {
  return _timedOut;
}

bool HX711::isConnected() const {
  return !_timedOut;
}

void HX711::setOffset(long offset) {
  _offset = offset;
}

long HX711::getOffset() const {
  return _offset;
}

// ---------------------------------------------------------------------------
// HX711Array (4 sensors)
// ---------------------------------------------------------------------------

HX711Array::HX711Array(
  uint8_t dout0, uint8_t clk0,
  uint8_t dout1, uint8_t clk1,
  uint8_t dout2, uint8_t clk2,
  uint8_t dout3, uint8_t clk3
) : _sensors{ {dout0, clk0}, {dout1, clk1}, {dout2, clk2}, {dout3, clk3} }
{}

void HX711Array::begin() {
  // Configure GPIO for all sensors first so they all start their first
  // conversion at roughly the same time, then wait for each to become ready.
  // Sequential begin() calls would let early sensors consume the shared
  // post-power-on ~400ms window, causing later sensors to time out.
  for (int i = 0; i < 4; i++) {
    _sensors[i].beginGpio();
  }
  for (int i = 0; i < 4; i++) {
    _sensors[i].waitReady();
  }
  // NOTE: tare() removed — CalibrationManager handles offset initialization
}

void HX711Array::readAll(long f[4]) {
  for (int i = 0; i < 4; i++) {
    f[i] = _sensors[i].read();
  }
}

void HX711Array::readAllReady(long f[4]) {
  bool done[4] = {false, false, false, false};
  int remaining = 4;
  while (remaining > 0) {
    for (int i = 0; i < 4; i++) {
      if (!done[i]) {
        bool newData = false;
        long v = _sensors[i].readIfReady(newData);
        if (newData) {
          f[i] = v;
          done[i] = true;
          remaining--;
        }
      }
    }
    yield();
  }
}

long HX711Array::readIfReady(uint8_t index, bool &newData) {
  if (index >= 4) { newData = false; return 0; }
  return _sensors[index].readIfReady(newData);
}

void HX711Array::tare() {
  for (int i = 0; i < 4; i++) {
    _sensors[i].tare(10);
  }
}

void HX711Array::setOffset(uint8_t index, long offset) {
  if (index < 4) _sensors[index].setOffset(offset);
}

long HX711Array::getOffset(uint8_t index) const {
  if (index < 4) return _sensors[index].getOffset();
  return 0;
}

bool HX711Array::anyTimedOut() const {
  for (int i = 0; i < 4; i++) {
    if (_sensors[i].timedOut()) return true;
  }
  return false;
}

uint8_t HX711Array::connectedCount() const {
  uint8_t count = 0;
  for (int i = 0; i < 4; i++) {
    if (_sensors[i].isConnected()) count++;
  }
  return count;
}

long HX711Array::getRaw(uint8_t index) {
  if (index >= 4) return 0;
  return _sensors[index].readRaw();
}

bool HX711Array::isTimedOut(uint8_t index) const {
  if (index >= 4) return true;
  return !_sensors[index].isConnected();
}

bool HX711Array::isConnected(uint8_t index) const {
  if (index >= 4) return false;
  return _sensors[index].isConnected();
}

void HX711Array::rescan() {
  for (int i = 0; i < 4; i++) {
    _sensors[i].beginGpio();
  }
  for (int i = 0; i < 4; i++) {
    _sensors[i].waitReady();
  }
}
