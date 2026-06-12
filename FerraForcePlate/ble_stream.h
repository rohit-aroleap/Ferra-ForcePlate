#pragma once

#include <Arduino.h>
#include <functional>

/**
 * BleStream — streams force-plate centre-of-pressure over BLE to the tablet.
 *
 * Replaces the old WiFi/WebSocket transport. A GATT peripheral advertises as
 * "FerraPlate" with one custom service (UUID base 7e40000x):
 *   - DATA characteristic (NOTIFY): a 32-byte little-endian frame
 *       { uint32 ms, float copX_cm, float copY_cm, float weight_kg,
 *         float fl_kg, float fr_kg, float bl_kg, float br_kg }
 *     — the first 16 bytes match the IMU board's frame (uint32 ms + 3×
 *     float32) so the dashboard's parser reads them unchanged; the four
 *     per-cell loads are appended for the Advanced panel.
 *   - CMD characteristic (WRITE): plain-text commands from the central
 *     (ZERO / START / STOP / STATUS / CAL …), queued and drained by readLine().
 *   - INFO characteristic (NOTIFY): the same human-readable [INFO]/[CAL]/
 *     [STATUS] lines that go to Serial, so the dashboard can drive the
 *     calibration wizard. Lines are chunked to 20-byte notifies (safe at any
 *     MTU) with a trailing '\n' marking end-of-line for reassembly.
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

    // Pack and notify the 32-byte frame. No-op if no central is connected.
    // Safe to call from any task. cellsKg = per-corner load {FL, FR, BL, BR}.
    void sendFrame(uint32_t ms, float copX, float copY, float weight,
                   const float cellsKg[4]);

    // Notify a human-readable text line to the central on the INFO
    // characteristic (chunked, '\n'-terminated). No-op when disconnected.
    // Call from the loop task only — not from the sampler.
    void sendInfo(const char* line);

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
