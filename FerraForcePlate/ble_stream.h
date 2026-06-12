#pragma once

#include <Arduino.h>
#include <functional>

/**
 * BleStream — streams force-plate centre-of-pressure over BLE to the tablet.
 *
 * Replaces the old WiFi/WebSocket transport. A GATT peripheral advertises as
 * "FerraPlate" with one custom service (UUID base 7e40000x):
 *   - DATA characteristic (NOTIFY): a 16-byte little-endian frame
 *       { uint32 ms, float copX_cm, float copY_cm, float weight_kg }
 *     — same shape the IMU board used (uint32 ms + 3× float32), so the
 *     dashboard's frame parser is unchanged.
 *   - CMD characteristic (WRITE): plain-text commands from the central
 *     (ZERO / START / STOP / STATUS …), queued and drained by readLine().
 *
 * The big lag lesson from the IMU board carries over: on connect we request a
 * 7.5 ms connection interval (see config.h), otherwise the host picks a slow
 * one and the notify stream batches/drops, lagging the dot ~0.5 s.
 *
 * Threading: sendFrame() is called from the HX711 sampler task (core 1); the
 * BLE stack runs its own tasks (core 0). The command queue is the only shared
 * mutable state and is guarded by a portMUX spinlock.
 */
class BleStream {
public:
    // Init the BLE stack, build the GATT DB, start advertising. Returns true
    // once the peripheral is up. Call once from setup().
    bool begin();

    // Pack and notify the 16-byte frame. No-op if no central is connected.
    // Safe to call from any task.
    void sendFrame(uint32_t ms, float copX, float copY, float weight);

    // Pop the next queued text command from the central (non-blocking).
    // Returns "" if none pending.
    String readLine();

    // True while a central is connected.
    bool isConnected() const;

    // Register a callback fired (from poll(), on the main loop) when a central
    // connects — used to re-send initial state.
    void setClientConnectedCallback(std::function<void()> cb);

    // 1 Hz [STATS] line + connect-callback dispatch. Call from loop().
    void poll();
};

extern BleStream bleStream;
