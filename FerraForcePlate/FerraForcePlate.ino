#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "hx711.h"
#include "ble_stream.h"

// ─── EEPROM Layout ───────────────────────────────────────────────────────────
// Magic(1) + CalData×4(40) + postingMode(1) = 42 bytes
#define EEPROM_SIZE    64
#define EEPROM_MAGIC   0xAD   // bump to invalidate stale data on new flash
#define MAGIC_ADDR     0
#define CAL_BASE_ADDR  1      // 10 bytes per cell × 4 = 40 bytes
#define POSTING_ADDR   41

struct CalData {
    bool  calibrated;
    float scaleFactor;
    long  offset;
    bool  twoPoint;
};

// Sample counter for the once-per-second [STATS] line (read in ble_stream.cpp).
volatile uint32_t g_sample_count = 0;

// Broadcast a human-readable line. BLE carries only binary frames + accepts
// commands, so info/status/calibration text goes to Serial only — `pio device
// monitor` (or any 115200 serial terminal) remains the full debugging view.
static void broadcast(const char* line) { Serial.println(line); }
static void broadcast(const String& line) { Serial.println(line.c_str()); }

static void samplerTask(void* pv);

// ─── Globals ─────────────────────────────────────────────────────────────────
HX711Array sensors(
    HX711_DOUT_0, HX711_CLK_0,
    HX711_DOUT_1, HX711_CLK_1,
    HX711_DOUT_2, HX711_CLK_2,
    HX711_DOUT_3, HX711_CLK_3
);

static CalData calData[NUM_CELLS];
static bool    cellConnected[NUM_CELLS];
static int     connectedCount = 0;
static bool    postingMode    = false;
static bool    calInProgress  = false;

// Auto-rescan: the Arduino loop sets this flag when fewer than 4 cells are
// connected; the sampler task (sole owner of the HX711 bus) picks it up at the
// top of its loop, runs sensors.rescan(), reapplies calibration, clears it.
static volatile bool autoRescanRequested = false;
static const uint32_t AUTO_RESCAN_PERIOD_MS = 3000;

// ─── Cell names ──────────────────────────────────────────────────────────────
static const char* CELL_NAMES[NUM_CELLS]     = {"FL", "FR", "BL", "BR"};
static const char* CELL_POSITIONS[NUM_CELLS] = {
    "FRONT-LEFT  (corner 1)",
    "FRONT-RIGHT (corner 2)",
    "BACK-LEFT   (corner 3)",
    "BACK-RIGHT  (corner 4)"
};

// ─── EEPROM helpers ──────────────────────────────────────────────────────────
static int calAddr(int idx) { return CAL_BASE_ADDR + idx * 10; }

static void eepromWriteCalData(int idx) {
    int a = calAddr(idx);
    EEPROM.write(a, calData[idx].calibrated ? 1 : 0);  a++;
    EEPROM.put(a,  calData[idx].scaleFactor);            a += 4;
    EEPROM.put(a,  calData[idx].offset);                 a += 4;
    EEPROM.write(a, calData[idx].twoPoint ? 1 : 0);
    EEPROM.commit();
}

static void eepromReadCalData(int idx) {
    int a = calAddr(idx);
    calData[idx].calibrated  = EEPROM.read(a) == 1;     a++;
    EEPROM.get(a, calData[idx].scaleFactor);              a += 4;
    EEPROM.get(a, calData[idx].offset);                   a += 4;
    calData[idx].twoPoint    = EEPROM.read(a) == 1;
}

static void initEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(MAGIC_ADDR) != EEPROM_MAGIC) {
        EEPROM.write(MAGIC_ADDR, EEPROM_MAGIC);
        for (int i = 0; i < NUM_CELLS; i++) {
            calData[i] = {false, 1.0f, 0L, false};
            eepromWriteCalData(i);
        }
        postingMode = false;
        EEPROM.write(POSTING_ADDR, 0);
        EEPROM.commit();
        Serial.println(F("[INFO] Fresh EEPROM — calibration initialised."));
    } else {
        for (int i = 0; i < NUM_CELLS; i++) eepromReadCalData(i);
        postingMode = EEPROM.read(POSTING_ADDR) == 1;
        Serial.println(F("[INFO] Calibration data loaded from EEPROM."));
    }
}

// ─── Apply calibration to HX711 array ────────────────────────────────────────
static void applyCalibration(int idx) {
    if (calData[idx].calibrated) {
        sensors.setOffset(idx, calData[idx].offset);
    } else {
        sensors.setOffset(idx, 0);
    }
}

// ─── Status JSON (Serial only) ────────────────────────────────────────────────
static void printStatus() {
    String s = "[STATUS] {\"cells\":[";
    for (int i = 0; i < NUM_CELLS; i++) {
        if (i) s += ',';
        s += "{\"id\":";      s += i;
        s += ",\"name\":\""; s += CELL_NAMES[i]; s += '"';
        s += ",\"connected\":"; s += cellConnected[i] ? "true" : "false";
        s += ",\"calibrated\":"; s += calData[i].calibrated ? "true" : "false";
        s += ",\"twoPoint\":"; s += calData[i].twoPoint ? "true" : "false";
        s += ",\"scale\":"; s += String(calData[i].scaleFactor, 4);
        s += ",\"offset\":"; s += calData[i].offset;
        s += '}';
    }
    s += "],\"postingMode\":";
    s += postingMode ? "true" : "false";
    s += ",\"ble\":";
    s += bleStream.isConnected() ? "true" : "false";
    s += ",\"heap\":"; s += (uint32_t)ESP.getFreeHeap();
    s += '}';
    broadcast(s);
}

// ─── Connection detection ─────────────────────────────────────────────────────
static void detectConnectedCells() {
    broadcast("[INFO] Scanning for load cells...");
    connectedCount = 0;
    sensors.rescan();
    for (int i = 0; i < NUM_CELLS; i++) {
        cellConnected[i] = sensors.isConnected(i);
        if (cellConnected[i]) {
            connectedCount++;
            applyCalibration(i);
        }
    }
    printStatus();
}

// ─── All connected cells calibrated? ─────────────────────────────────────────
static bool allConnectedCalibrated() {
    for (int i = 0; i < NUM_CELLS; i++) {
        if (cellConnected[i] && !calData[i].calibrated) return false;
    }
    return connectedCount > 0;
}

// ─── Tare ─────────────────────────────────────────────────────────────────────
static void tareCell(int idx) {
    if (!cellConnected[idx]) {
        broadcast(String("[INFO] ") + CELL_NAMES[idx] + " not connected.");
        return;
    }
    sensors.setOffset(idx, 0);
    long sum = 0;
    for (int s = 0; s < 20; s++) {
        long f[4]; sensors.readAll(f);
        sum += f[idx];
        delay(25);
    }
    long newOffset = sum / 20;
    sensors.setOffset(idx, newOffset);
    calData[idx].offset = newOffset;
    if (calData[idx].calibrated) eepromWriteCalData(idx);
    broadcast(String("[INFO] Tared ") + CELL_NAMES[idx] + " offset=" + newOffset);
}

// ─── Reset calibration ────────────────────────────────────────────────────────
static void resetCalibration(int idx) {
    calData[idx] = {false, 1.0f, 0L, false};
    sensors.setOffset(idx, 0);
    eepromWriteCalData(idx);
    postingMode = false;
    EEPROM.write(POSTING_ADDR, 0);
    EEPROM.commit();
    broadcast(String("[INFO] Calibration reset: ") + CELL_NAMES[idx]);
}

// ─── Serial helpers (calibration wizard is serial-only) ───────────────────────
static void flushSerial() {
    while (Serial.available()) Serial.read();
}

static String readLine(unsigned long timeoutMs = 60000) {
    flushSerial();
    String s = "";
    unsigned long t0 = millis();
    while (true) {
        if (millis() - t0 > timeoutMs) return "";
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (s.length() > 0) return s;
            } else {
                s += c;
            }
        }
    }
}

static void waitForAck(unsigned long timeoutMs = 60000) {
    flushSerial();
    unsigned long t0 = millis();
    while (!Serial.available()) {
        if (millis() - t0 > timeoutMs) return;
        delay(20);
    }
    flushSerial();
}

static float readFloat(unsigned long timeoutMs = 60000) {
    return readLine(timeoutMs).toFloat();
}

// ─── Sample average for one cell ─────────────────────────────────────────────
static long sampleCell(int idx, int n = 20) {
    long sum = 0;
    for (int s = 0; s < n; s++) {
        long f[4]; sensors.readAll(f);
        sum += f[idx];
        delay(25);
    }
    return sum / n;
}

// ─── Full 2-point calibration sequence (all connected cells) ─────────────────
//   1. Ask for weight1 and weight2 values upfront (grams)
//   2. Zero (tare) ALL connected cells at once
//   3. For each cell: place weight1 → measure, place weight2 → measure
static void calibrateAllCells() {
    calInProgress = true;

    Serial.println(F("[CAL] ════════════════════════════════════════"));
    Serial.println(F("[CAL] CALIBRATION SEQUENCE — 2-point, all cells"));
    Serial.println(F("[CAL] You will need two known weights."));
    Serial.println(F("[CAL] Enter WEIGHT 1 in grams (e.g. 500):"));
    float weight1 = readFloat(120000);
    if (weight1 <= 0) {
        Serial.println(F("[CAL] ERR: invalid weight — aborting.")); calInProgress = false; return;
    }
    Serial.print(F("[CAL] Weight 1 = ")); Serial.print(weight1, 1); Serial.println(F("g"));

    Serial.println(F("[CAL] Enter WEIGHT 2 in grams (must differ from weight 1, e.g. 1000):"));
    float weight2 = readFloat(120000);
    if (weight2 <= 0 || fabsf(weight2 - weight1) < 1.0f) {
        Serial.println(F("[CAL] ERR: invalid or identical weight — aborting.")); calInProgress = false; return;
    }
    Serial.print(F("[CAL] Weight 2 = ")); Serial.print(weight2, 1); Serial.println(F("g"));
    Serial.println(F("[CAL] ────────────────────────────────────────"));

    Serial.println(F("[CAL] STEP 1/3 — ZERO all cells"));
    Serial.println(F("[CAL]   Remove ALL weights from the force plate."));
    Serial.println(F("[CAL]   Send any key when ready."));
    waitForAck(120000);

    Serial.println(F("[CAL] Measuring zero load on all cells (20 samples each)..."));
    long rawZero[NUM_CELLS] = {0};
    for (int i = 0; i < NUM_CELLS; i++) {
        if (!cellConnected[i]) continue;
        sensors.setOffset(i, 0);
        rawZero[i] = sampleCell(i);
        sensors.setOffset(i, rawZero[i]);
        Serial.print(F("[CAL]   ")); Serial.print(CELL_NAMES[i]);
        Serial.print(F(" zero_raw=")); Serial.println(rawZero[i]);
    }
    Serial.println(F("[CAL] All cells zeroed."));
    Serial.println(F("[CAL] ────────────────────────────────────────"));

    long rawW1[NUM_CELLS] = {0};
    long rawW2[NUM_CELLS] = {0};

    for (int i = 0; i < NUM_CELLS; i++) {
        if (!cellConnected[i]) continue;
        char tag[12]; sprintf(tag, "[CAL:%s]", CELL_NAMES[i]);

        Serial.println(F("[CAL] ────────────────────────────────────────"));
        Serial.print(tag); Serial.print(F(" STEP 2/3 — Place WEIGHT 1 ("));
        Serial.print(weight1, 1); Serial.print(F("g) on "));
        Serial.print(CELL_POSITIONS[i]); Serial.println(F("."));
        Serial.print(tag); Serial.println(F(" Send any key when ready."));
        waitForAck(120000);

        Serial.print(tag); Serial.println(F(" Measuring weight 1 (20 samples)..."));
        rawW1[i] = sampleCell(i);  // offset already set to rawZero, so this is tare-relative
        float scale1 = (float)rawW1[i] / weight1;
        Serial.print(tag); Serial.print(F(" raw=")); Serial.print(rawW1[i]);
        Serial.print(F(" scale1=")); Serial.println(scale1, 4);

        Serial.print(tag); Serial.print(F(" STEP 3/3 — Place WEIGHT 2 ("));
        Serial.print(weight2, 1); Serial.print(F("g) on "));
        Serial.print(CELL_POSITIONS[i]); Serial.println(F("."));
        Serial.print(tag); Serial.println(F(" Send any key when ready."));
        waitForAck(120000);

        Serial.print(tag); Serial.println(F(" Measuring weight 2 (20 samples)..."));
        rawW2[i] = sampleCell(i);
        float scale2 = (float)rawW2[i] / weight2;
        float diff = fabsf(scale1 - scale2) / ((scale1 + scale2) / 2.0f) * 100.0f;
        Serial.print(tag); Serial.print(F(" raw=")); Serial.print(rawW2[i]);
        Serial.print(F(" scale2=")); Serial.print(scale2, 4);
        Serial.print(F(" diff=")); Serial.print(diff, 2); Serial.println(F("%"));
        if (diff > 5.0f) {
            Serial.print(tag); Serial.println(F(" WARN: scales differ >5% — check cell and weights"));
        }

        float finalScale = (scale1 + scale2) / 2.0f;

        float verify = (float)rawW2[i] / finalScale;
        Serial.print(tag); Serial.print(F(" VERIFY (weight2 still on plate): "));
        Serial.print(verify, 2); Serial.print(F("g  (expected: "));
        Serial.print(weight2, 1); Serial.println(F("g)"));

        calData[i] = {true, finalScale, rawZero[i], true};
        eepromWriteCalData(i);
        Serial.print(tag); Serial.println(F(" DONE — saved to EEPROM."));
    }

    Serial.println(F("[CAL] ════════════════════════════════════════"));
    Serial.println(F("[CAL] Calibration complete for all connected cells."));
    calInProgress = false;
}

// ─── One-shot read helpers (serial debugging) ────────────────────────────────
static void printCalibratedValues() {
    long f[4]; sensors.readAll(f);
    broadcast("[LIVE] Calibrated values (grams):");
    float total = 0;
    for (int i = 0; i < NUM_CELLS; i++) {
        float val = 0.0f;
        if (cellConnected[i] && calData[i].calibrated && calData[i].scaleFactor != 0.0f) {
            val = (float)f[i] / calData[i].scaleFactor;
        }
        total += val;
        String line = "  "; line += CELL_NAMES[i];
        line += " ("; line += CELL_POSITIONS[i]; line += "): ";
        if (!cellConnected[i])           line += "NOT CONNECTED";
        else if (!calData[i].calibrated) line += "NOT CALIBRATED";
        else { line += String(val, 2); line += 'g'; }
        broadcast(line);
    }
    broadcast("  TOTAL: " + String(total, 2) + "g");
}

static void printRawValues() {
    long f[4]; sensors.readAll(f);
    broadcast("[LIVE] Raw ADC values:");
    for (int i = 0; i < NUM_CELLS; i++) {
        String line = "  "; line += CELL_NAMES[i];
        line += " ("; line += CELL_POSITIONS[i]; line += "): ";
        if (!cellConnected[i]) line += "NOT CONNECTED";
        else line += String(f[i]);
        broadcast(line);
    }
}

// ─── Command handler (serial + BLE) ───────────────────────────────────────────
static void handleCommand(String cmd) {
    cmd.trim(); cmd.toLowerCase();

    if (cmd == "h" || cmd == "help") {
        broadcast("[INFO] ════════════════════════════════════════");
        broadcast("[INFO] Available commands:");
        broadcast("[INFO]   h/help   - Show this help message");
        broadcast("[INFO]   s/status - Print full status JSON");
        broadcast("[INFO]   d        - Re-scan for connected load cells");
        broadcast("[INFO]   l        - Print live calibrated values (grams)");
        broadcast("[INFO]   r        - Print raw ADC values once");
        broadcast("[INFO]   start    - Start streaming CoP frames over BLE");
        broadcast("[INFO]   stop     - Stop streaming");
        broadcast("[INFO]   t/zero   - Tare all connected cells");
        broadcast("[INFO]   t1-t4    - Tare single cell (1=FL 2=FR 3=BL 4=BR)");
        broadcast("[INFO]   c        - Full 2-point calibration sequence (serial)");
        broadcast("[INFO]   x        - Reset calibration for all cells");
        broadcast("[INFO]   x1-x4    - Reset calibration for single cell");
        broadcast("[INFO] ════════════════════════════════════════");

    } else if (cmd == "s" || cmd == "status") {
        printStatus();

    } else if (cmd == "l") {
        printCalibratedValues();

    } else if (cmd == "r") {
        printRawValues();

    } else if (cmd == "start") {
        if (!allConnectedCalibrated()) {
            broadcast("[INFO] ERR: not all connected cells are calibrated. Run 'c' first.");
        } else {
            postingMode = true;
            EEPROM.write(POSTING_ADDR, 1);
            EEPROM.commit();
            broadcast("[INFO] Streaming started.");
        }

    } else if (cmd == "stop") {
        postingMode = false;
        EEPROM.write(POSTING_ADDR, 0);
        EEPROM.commit();
        broadcast("[INFO] Streaming stopped.");

    } else if (cmd == "d") {
        detectConnectedCells();

    } else if (cmd == "t" || cmd == "zero") {
        for (int i = 0; i < NUM_CELLS; i++) tareCell(i);
        printStatus();

    } else if (cmd == "c") {
        if (connectedCount == 0) {
            broadcast("[INFO] No connected cells to calibrate.");
        } else {
            calibrateAllCells();
            if (allConnectedCalibrated()) {
                postingMode = true;
                EEPROM.write(POSTING_ADDR, 1);
                EEPROM.commit();
                broadcast("[INFO] All cells calibrated — streaming enabled.");
            }
            printStatus();
        }

    } else if (cmd == "x") {
        for (int i = 0; i < NUM_CELLS; i++) resetCalibration(i);
        printStatus();

    } else if (cmd.length() == 2) {
        int idx = cmd[1] - '1';
        if (idx < 0 || idx >= NUM_CELLS) {
            broadcast("[INFO] ERR: invalid cell index, use 1-4"); return;
        }
        if (cmd[0] == 't') {
            tareCell(idx); printStatus();
        } else if (cmd[0] == 'x') {
            resetCalibration(idx); printStatus();
        } else {
            broadcast("[INFO] Unknown command: " + cmd);
        }

    } else {
        broadcast("[INFO] Unknown command: " + cmd);
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(800);
    Serial.println(F("[INFO] Ferra Force Plate — ESP32 (BLE)"));

    initEEPROM();
    detectConnectedCells();

    if (connectedCount == 0) {
        Serial.println(F("[INFO] No cells detected. Send 'd' to re-scan."));
    } else if (allConnectedCalibrated()) {
        // Stream automatically once calibrated so the game just works on
        // connect — no need to send 'start' from the tablet.
        postingMode = true;
        EEPROM.write(POSTING_ADDR, 1);
        EEPROM.commit();
        Serial.println(F("[INFO] Calibrated — will stream CoP on BLE connect."));
    } else {
        postingMode = false;
        Serial.println(F("[INFO] Uncalibrated cells detected. Use 'c' to calibrate (over serial)."));
    }

    bleStream.begin();
    bleStream.setClientConnectedCallback([]() { printStatus(); });

    // Pin the HX711 sampler to core 1. The BLE controller + host run on core 0
    // by default; isolating sampling on the other core keeps the high-priority
    // radio tasks from jittering the bit-bang timing (this was the lesson from
    // the WiFi build, adapted: there WiFi was the noisy neighbour, here it's BT).
    BaseType_t ok = xTaskCreatePinnedToCore(
        samplerTask, "hx711_sampler", 4096, nullptr,
        configMAX_PRIORITIES - 2,
        nullptr, 1 /* core 1 */);
    if (ok != pdPASS) {
        Serial.println(F("[INIT] FATAL: sampler task creation failed."));
        while (true) { delay(1000); }
    }
    Serial.println(F("[INIT] HX711 sampler pinned to core 1."));
}

// ─── Sampler task (core 1, high priority) ────────────────────────────────────
// Polls all 4 HX711s; when all have fresh data and the sample period elapsed,
// computes calibrated grams → centre-of-pressure (cm) + total weight (kg),
// EMA-smooths the CoP, and streams the 16-byte frame over BLE.
static void samplerTask(void* /*pv*/) {
    long     pendingF[4]     = {0, 0, 0, 0};
    bool     pendingReady[4] = {false, false, false, false};
    uint32_t lastSampleUs    = 0;
    float    copXf = 0.0f, copYf = 0.0f;
    bool     emaInit = false;

    for (;;) {
        // Auto-rescan request from the Arduino loop. Runs only on this task so
        // the HX711 bit-bang protocol stays single-threaded.
        if (autoRescanRequested && !calInProgress) {
            autoRescanRequested = false;
            sensors.rescan();
            int newCount = 0;
            for (int i = 0; i < NUM_CELLS; i++) {
                bool wasConnected = cellConnected[i];
                cellConnected[i] = sensors.isConnected(i);
                if (cellConnected[i]) {
                    newCount++;
                    if (!wasConnected) applyCalibration(i);
                }
            }
            connectedCount = newCount;
            pendingReady[0] = pendingReady[1] = pendingReady[2] = pendingReady[3] = false;
        }

        bool active = postingMode && !calInProgress && connectedCount > 0
                   && bleStream.isConnected();

        if (active) {
            for (int i = 0; i < NUM_CELLS; i++) {
                if (!pendingReady[i]) {
                    bool newData = false;
                    long v = sensors.readIfReady(i, newData);
                    if (newData) { pendingF[i] = v; pendingReady[i] = true; }
                }
            }

            bool allReady = pendingReady[0] && pendingReady[1]
                         && pendingReady[2] && pendingReady[3];
            const uint32_t nowUs = micros();
            if (allReady && (nowUs - lastSampleUs >= SAMPLE_PERIOD_US)) {
                if (nowUs - lastSampleUs > 2 * SAMPLE_PERIOD_US) {
                    lastSampleUs = nowUs;
                } else {
                    lastSampleUs += SAMPLE_PERIOD_US;
                }
                pendingReady[0] = pendingReady[1] = pendingReady[2] = pendingReady[3] = false;

                // Calibrated grams per corner.
                float vals[NUM_CELLS] = {0.0f, 0.0f, 0.0f, 0.0f};
                float total = 0.0f;
                for (int i = 0; i < NUM_CELLS; i++) {
                    if (cellConnected[i] && calData[i].calibrated && calData[i].scaleFactor != 0.0f) {
                        vals[i] = (float)pendingF[i] / calData[i].scaleFactor;
                    }
                    total += vals[i];
                }
                // Guard: a calibrated cell that produced an exact-0 reading means
                // a stale/glitched sample — skip rather than emit a bad CoP.
                bool skipSample = false;
                for (int i = 0; i < NUM_CELLS; i++) {
                    if (cellConnected[i] && calData[i].calibrated && vals[i] == 0.0f) {
                        skipSample = true; break;
                    }
                }
                if (skipSample) { vTaskDelay(1); continue; }

                float weightKg = total / 1000.0f;
                float copX, copY;
                if (total < WEIGHT_ON_THRESHOLD_G) {
                    // Nobody standing — centre the dot and reset the smoother
                    // so it snaps clean when the next person steps on.
                    copX = 0.0f; copY = 0.0f;
                    copXf = 0.0f; copYf = 0.0f; emaInit = false;
                } else {
                    float fl = vals[0], fr = vals[1], bl = vals[2], br = vals[3];
                    float copXmm = (PLATE_WIDTH_MM  * 0.5f) * ((fr + br) - (fl + bl)) / total;
                    float copYmm = (PLATE_HEIGHT_MM * 0.5f) * ((fl + fr) - (bl + br)) / total;
                    float rawX = copXmm / 10.0f;   // → cm
                    float rawY = copYmm / 10.0f;
                    if (!emaInit) {
                        copXf = rawX; copYf = rawY; emaInit = true;
                    } else {
                        copXf = COP_EMA_ALPHA * rawX + (1.0f - COP_EMA_ALPHA) * copXf;
                        copYf = COP_EMA_ALPHA * rawY + (1.0f - COP_EMA_ALPHA) * copYf;
                    }
                    copX = copXf; copY = copYf;
                }

                g_sample_count++;
                bleStream.sendFrame(millis(), copX, copY, weightKg);
            }
        } else {
            pendingReady[0] = pendingReady[1] = pendingReady[2] = pendingReady[3] = false;
            emaInit = false;
        }

        // 1 tick (~1 ms). HX711 at 80 SPS produces a ready edge every ~12.5 ms,
        // so a 1 ms poll never misses an edge.
        vTaskDelay(1);
    }
}

// ─── Loop (core 1) — command I/O + BLE housekeeping ──────────────────────────
void loop() {
    // Non-blocking Serial command accumulation.
    {
        static char    serialBuf[64];
        static uint8_t serialLen = 0;
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                if (serialLen > 0) {
                    serialBuf[serialLen] = '\0';
                    handleCommand(String(serialBuf));
                    serialLen = 0;
                }
            } else if (serialLen < sizeof(serialBuf) - 1) {
                serialBuf[serialLen++] = c;
            }
        }
    }

    // Drain commands received over BLE (queued by the cmd char write callback).
    String bCmd = bleStream.readLine();
    if (bCmd.length() > 0) handleCommand(bCmd);

    // 1 Hz [STATS] + connect-callback dispatch.
    bleStream.poll();

    // Periodic auto-rescan whenever fewer than 4 cells are visible. The sampler
    // task performs the actual rescan to keep the HX711 bus single-threaded.
    static uint32_t lastRescanMs = 0;
    uint32_t nowMs = millis();
    if (connectedCount < NUM_CELLS && !calInProgress && !autoRescanRequested
        && (nowMs - lastRescanMs) >= AUTO_RESCAN_PERIOD_MS) {
        lastRescanMs = nowMs;
        autoRescanRequested = true;
    }

    vTaskDelay(1);
}
