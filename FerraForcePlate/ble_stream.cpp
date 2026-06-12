#include "ble_stream.h"
#include "config.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Sample counter owned by main.cpp — used in the [STATS] line.
extern volatile uint32_t g_sample_count;

BleStream bleStream;

// ─── 32-byte frame. The first 16 bytes match the IMU board's struct so the
//     dashboard's DataView parser reads them unchanged; the four per-cell
//     loads follow. ESP32 is little-endian; sent verbatim. ──────────────────
struct __attribute__((packed)) PlateFrame {
    uint32_t ms;
    float    copX;      // cm
    float    copY;      // cm
    float    weight;    // kg
    float    cells[4];  // kg per corner: FL, FR, BL, BR
};
static_assert(sizeof(PlateFrame) == 32, "frame must be 32 bytes");

// ─── BLE handles + shared state ──────────────────────────────────────────────
static BLEServer*         s_server   = nullptr;
static BLECharacteristic* s_dataChar = nullptr;
static BLECharacteristic* s_cmdChar  = nullptr;

static volatile bool s_connected     = false;
static volatile bool s_justConnected = false;   // raised in onConnect, cleared by poll()

static std::function<void()> s_onClientConnected;

// Single-slot command buffer — commands arrive at human cadence. Written by
// the BLE stack task (onWrite, core 0), read by loop() (core 1).
static portMUX_TYPE s_cmdMux  = portMUX_INITIALIZER_UNLOCKED;
static char         s_cmdBuf[64];
static volatile bool s_cmdReady = false;

// ─── [STATS] / diagnostics ───────────────────────────────────────────────────
static uint32_t s_txFrames    = 0;
static uint32_t s_lastTxMs    = 0;
static uint32_t s_maxGapMs    = 0;
static uint32_t s_lastStatsMs = 0;

// ─── Server callbacks: connect / disconnect ──────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param) override {
        s_connected     = true;
        s_justConnected = true;
        // Request a fast connection interval (THE fix for ~0.5 s dot lag).
        // The central has final say; we log the request, the negotiated value
        // shows up host-side in the dashboard's BLE diagnostics card.
        server->updateConnParams(param->connect.remote_bda,
                                 BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX,
                                 BLE_CONN_LATENCY,       BLE_CONN_TIMEOUT);
        Serial.printf("[BLE] central connected — requested %.1f ms interval\n",
                      BLE_CONN_INTERVAL_MIN * 1.25f);
    }
    // Keep the no-arg overload too so older cores still compile.
    void onConnect(BLEServer* /*server*/) override {
        s_connected     = true;
        s_justConnected = true;
    }
    void onDisconnect(BLEServer* /*server*/) override {
        s_connected = false;
        Serial.println("[BLE] central disconnected — re-advertising");
        BLEDevice::startAdvertising();   // immediately discoverable again
    }
};

// ─── Command characteristic write callback ───────────────────────────────────
class CmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        uint8_t* data = c->getData();
        size_t   len  = c->getLength();
        if (!data || len == 0) return;
        if (len > sizeof(s_cmdBuf) - 1) len = sizeof(s_cmdBuf) - 1;

        char tmp[sizeof(s_cmdBuf)];
        memcpy(tmp, data, len);
        tmp[len] = '\0';
        // Strip trailing CR/LF.
        for (int i = (int)len - 1; i >= 0 && (tmp[i] == '\n' || tmp[i] == '\r'); i--) {
            tmp[i] = '\0';
        }

        portENTER_CRITICAL(&s_cmdMux);
        strncpy(s_cmdBuf, tmp, sizeof(s_cmdBuf));
        s_cmdReady = true;
        portEXIT_CRITICAL(&s_cmdMux);
    }
};

// ─── Public API ───────────────────────────────────────────────────────────────
bool BleStream::begin() {
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(64);   // 32-byte frames need ATT MTU ≥ 35; a host that skips
                             // the MTU exchange truncates notifies to 20 B and the
                             // dashboard simply hides the per-cell values

    s_server = BLEDevice::createServer();
    s_server->setCallbacks(new ServerCallbacks());

    BLEService* svc = s_server->createService(BLE_SERVICE_UUID);

    s_dataChar = svc->createCharacteristic(
        BLE_DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    s_dataChar->addDescriptor(new BLE2902());   // lets the central enable notifications

    s_cmdChar = svc->createCharacteristic(
        BLE_CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    s_cmdChar->setCallbacks(new CmdCallbacks());

    svc->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println(F("========================================"));
    Serial.printf( "  BLE device:   %s\n", BLE_DEVICE_NAME);
    Serial.printf( "  Service UUID: %s\n", BLE_SERVICE_UUID);
    Serial.println(F("  Advertising — connect with Web Bluetooth."));
    Serial.println(F("========================================"));
    return true;
}

void BleStream::sendFrame(uint32_t ms, float copX, float copY, float weight,
                          const float cellsKg[4]) {
    if (!s_connected || !s_dataChar) return;

    PlateFrame f{ms, copX, copY, weight,
                 {cellsKg[0], cellsKg[1], cellsKg[2], cellsKg[3]}};
    s_dataChar->setValue((uint8_t*)&f, sizeof(f));
    s_dataChar->notify();

    if (s_lastTxMs != 0) {
        uint32_t gap = ms - s_lastTxMs;
        if (gap > s_maxGapMs) s_maxGapMs = gap;
    }
    s_lastTxMs = ms;
    s_txFrames++;
}

String BleStream::readLine() {
    if (!s_cmdReady) return String();
    char out[sizeof(s_cmdBuf)];
    portENTER_CRITICAL(&s_cmdMux);
    strncpy(out, s_cmdBuf, sizeof(out));
    s_cmdReady = false;
    portEXIT_CRITICAL(&s_cmdMux);
    return String(out);
}

bool BleStream::isConnected() const { return s_connected; }

void BleStream::setClientConnectedCallback(std::function<void()> cb) {
    s_onClientConnected = std::move(cb);
}

void BleStream::poll() {
    // Fire the connect callback on the main loop (keeps the BLE stack callback
    // itself light — no EEPROM / Serial-heavy work in stack context).
    if (s_justConnected) {
        s_justConnected = false;
        if (s_onClientConnected) s_onClientConnected();
    }

    uint32_t now = millis();
    if (now - s_lastStatsMs >= 1000) {
        static uint32_t lastSamples = 0;
        uint32_t samples = g_sample_count - lastSamples;
        lastSamples = g_sample_count;
        Serial.printf("[STATS] tx=%lu/s samples=%lu/s maxGap=%lums clients=%u\n",
                      (unsigned long)s_txFrames,
                      (unsigned long)samples,
                      (unsigned long)s_maxGapMs,
                      (unsigned)(s_connected ? 1 : 0));
        s_txFrames    = 0;
        s_maxGapMs    = 0;
        s_lastStatsMs = now;
    }
}
