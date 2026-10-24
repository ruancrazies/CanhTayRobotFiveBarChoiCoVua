// Every firmware tunable lives here (SPEC rule 7); pin numbers live in pins.h, the wire protocol in
// protocol.h. Plain constants, no Arduino include, so host tests can include it.
//
// TODO(confirm) marks a value the SPEC leaves open (SPEC 5 and 13). The value written is a PLACEHOLDER
// so the code can run; it is not a decision. A value of 0 marked "unset" means "not decided": the stage
// that needs it must refuse to run until it is set.
#ifndef CHESSBOT_CONFIG_H
#define CHESSBOT_CONFIG_H

#include <stdint.h>

namespace chessbot {
namespace config {

// ---- Identity ---------------------------------------------------------------------------------
// FW= of the INFO reply: one token, no spaces, no '='.
constexpr char kFirmwareVersion[] = "4a-0.1.0";

// ---- Serial -----------------------------------------------------------------------------------
// TODO(confirm): SPEC 5 and 13 mark the baud rate [TO CONFIRM], suggested 115200. platformio.ini
// monitor_speed must match (tests/test_firmware_layout.py checks it).
constexpr uint32_t kBaudRate = 115200;
constexpr uint16_t kSerialRxBufferBytes = 512;  // UART receive buffer; a few full lines
// Longest accepted line, terminator excluded. Same limit as the PC side (SerialSettings.max_line_bytes).
constexpr uint16_t kMaxLineBytes = 256;
constexpr uint16_t kMaxResponseBytes = 128;  // longest line the firmware sends, terminator excluded

// ---- Command queue and heartbeat --------------------------------------------------------------
// TODO(confirm): the SPEC does not give the queue depth (5.3: "once the real queue depth is known").
constexpr uint8_t kQueueDepth = 8;
// TODO(confirm): the SPEC does not give the heartbeat threshold (5.3, 10.2). It must stay well above the
// PC's PING interval. PLACEHOLDER.
constexpr uint32_t kHeartbeatTimeoutMs = 2000;

// ---- Boot levels (SPEC 4.3, 10.2) -------------------------------------------------------------
// true = GPIO HIGH. TB6600 common-anode wiring: HIGH or floating = driver enabled, LOW = disabled.
constexpr bool kTb6600EnaLevelEnabled = true;
constexpr bool kTb6600EnaLevelDisabled = false;
// Z motor (L298 ENA, IN1, IN2): LOW at boot, so Z cannot move before a command arrives.
constexpr bool kZMotorPinsBootLevel = false;

// ---- PWM (SPEC 10.2: LEDC, separate channels and timers) --------------------------------------
constexpr uint32_t kZMotorPwmHz = 10000;  // "about 10 kHz"
constexpr uint32_t kServoPwmHz = 50;
// Arduino-ESP32 2.x maps LEDC channel c to timer (c / 2) % 4, so channels 0 and 2 use different timers.
constexpr uint8_t kZMotorLedcChannel = 0;
constexpr uint8_t kServoLedcChannel = 2;
constexpr uint8_t kZMotorPwmResolutionBits = 8;
constexpr uint8_t kServoPwmResolutionBits = 16;
constexpr uint16_t kServoPulseMinUs = 500;   // 0 degrees   (SPEC 4.3)
constexpr uint16_t kServoPulseMaxUs = 2500;  // 180 degrees (SPEC 4.3)

// ---- Z axis -----------------------------------------------------------------------------------
// Per-move timeout = factor x full-travel time (SPEC 4.3; same value as config/hardware.yaml).
constexpr float kZTimeoutFactor = 1.5f;
// TODO(confirm): full Z travel time, measured at assembly (SPEC 13). unset.
constexpr uint32_t kZFullTravelMs = 0;

// ---- Inputs -----------------------------------------------------------------------------------
// Limit-switch debounce, SPEC 4.3 allows 5 to 10 ms (config/hardware.yaml debounce_ms_min/max). The
// low end is used so debounce does not delay the stop.
constexpr uint8_t kDebounceMs = 5;

// ---- Motion limits ----------------------------------------------------------------------------
// TODO(confirm): maximum acceleration and speed (SPEC 13). unset.
constexpr float kMaxAccelDegPerS2 = 0.0f;
constexpr float kMaxSpeedDegPerS = 0.0f;
// TODO(confirm): TB6600 DIR opto is slow; direction-change delay "as the TB6600 requires" (SPEC 10.2). unset.
constexpr uint16_t kDirectionChangeDelayUs = 0;
// TODO(confirm): lost-step threshold, "start at 3 degrees" (SPEC 13).
constexpr float kStepLossThresholdDeg = 3.0f;

// ---- Sensors ----------------------------------------------------------------------------------
// SENSOR_FAULT when r = V_OUT / V_A2 is below the first or above the second (SPEC 4.3).
constexpr float kSensorRMin = 0.02f;
constexpr float kSensorRMax = 0.98f;

}  // namespace config
}  // namespace chessbot

#endif  // CHESSBOT_CONFIG_H
