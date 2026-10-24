// ESP32 pin map: docs/SPEC.md section 4.2. The only place GPIO numbers are written.
// Plain constants, no Arduino include, so host tests can include it.
#ifndef CHESSBOT_PINS_H
#define CHESSBOT_PINS_H

#include <stdint.h>

namespace chessbot {
namespace pins {

// TB6600 #1, joint 1 (alpha). Common-anode wiring (SPEC 4.3): GPIO HIGH or floating = driver ENABLED.
constexpr uint8_t kTb6600Ena1 = 13;  // D13 ENA-
constexpr uint8_t kTb6600Dir1 = 14;  // D14 DIR-
constexpr uint8_t kTb6600Pul1 = 27;  // D27 PUL-

// TB6600 #2, joint 2 (beta).
constexpr uint8_t kTb6600Ena2 = 26;  // D26 ENA-
constexpr uint8_t kTb6600Dir2 = 25;  // D25 DIR-
constexpr uint8_t kTb6600Pul2 = 33;  // D33 PUL-

// L298 (Z motor).
constexpr uint8_t kL298Ena = 4;   // D4  ENA, Z motor PWM
constexpr uint8_t kL298In1 = 18;  // D18 IN1, raise direction
constexpr uint8_t kL298In2 = 19;  // D19 IN2, lower direction

// ADS1115 on I2C (address 0x48, ADDR to GND).
constexpr uint8_t kI2cSda = 21;  // D21
constexpr uint8_t kI2cScl = 22;  // D22
constexpr uint8_t kAds1115I2cAddress = 0x48;

// SG90 gripper servo, 50 Hz PWM on its own LEDC channel/timer.
constexpr uint8_t kServoPwm = 32;  // D32

// Z limit switches (normally closed, input only, external 10 kOhm pull-down).
constexpr uint8_t kLimitTop = 35;     // D35 limit switch #1 (TOP)
constexpr uint8_t kLimitBottom = 34;  // D34 limit switch #2 (BOTTOM)

}  // namespace pins
}  // namespace chessbot

#endif  // CHESSBOT_PINS_H
