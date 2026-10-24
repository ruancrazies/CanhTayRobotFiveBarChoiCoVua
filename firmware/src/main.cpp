// Stage 4a: serial command skeleton. docs/SPEC.md sections 4.2, 4.3, 5 and 10.2.
//
// Handles INFO, PING, ENABLE, DISABLE, ESTOP and RESET; every other command parses and answers error:1
// until its stage (see lib/chessbot_core/src/command_processor.cpp). No stepper, Z, servo or ADS1115 code
// exists yet, so nothing here can produce a pulse or move Z.
//
// All logic lives in lib/chessbot_core (no Arduino includes, unit tested on the host). This file only
// connects it to the pins and to Serial.
#include <Arduino.h>
#include <string.h>

// The project headers are included by a path relative to this file, not by bare name: PlatformIO appends the
// project's -I flags after the framework's include directories, so an SDK header called config.h or
// protocol.h could be found first. (A quoted include is looked up next to this file before any -I directory.)
#include "../include/config.h"
#include "../include/pins.h"
#include "../include/protocol.h"
#include "chessbot_core.h"

namespace {

using chessbot::CommandProcessor;
using chessbot::HardwareOps;
using chessbot::LineFramer;
using chessbot::ResponseSink;

// The level is written BEFORE the pin becomes an output, so the pad never shows a stale level (on the
// ESP32, digitalWrite() on a pin that is not an output yet loads the output register).
void drive_pin(uint8_t pin, bool high) {
  digitalWrite(pin, high ? HIGH : LOW);
  pinMode(pin, OUTPUT);
}

void write_drivers_enable(bool enabled) {
  const bool high = enabled ? chessbot::config::kTb6600EnaLevelEnabled : chessbot::config::kTb6600EnaLevelDisabled;
  drive_pin(chessbot::pins::kTb6600Ena1, high);
  drive_pin(chessbot::pins::kTb6600Ena2, high);
}

void drive_z_motor_pins_idle() {
  drive_pin(chessbot::pins::kL298Ena, chessbot::config::kZMotorPinsBootLevel);
  drive_pin(chessbot::pins::kL298In1, chessbot::config::kZMotorPinsBootLevel);
  drive_pin(chessbot::pins::kL298In2, chessbot::config::kZMotorPinsBootLevel);
}

// SPEC 10.2 "Boot": the very first thing setup() does. TB6600 ENA pins HIGH = drivers enabled, so the
// motors hold position (SPEC 4.3: HIGH or floating = enabled). The Z motor pins are LOW, so Z cannot move
// before a command arrives. No pulse is emitted: the PUL and DIR pins are not touched in stage 4a.
void hold_boot_levels() {
  write_drivers_enable(true);
  drive_z_motor_pins_idle();
}

class SerialSink : public ResponseSink {
 public:
  void write_line(const char* line) override {
    Serial.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
    Serial.write(static_cast<uint8_t>(chessbot::protocol::kLineTerminator));  // '\n', not println()'s "\r\n"
  }
};

class BoardHardware : public HardwareOps {
 public:
  void set_drivers_enabled(bool enabled) override { write_drivers_enable(enabled); }

  // ESTOP / heartbeat timeout: stop every motion output, NEVER the TB6600 ENA pins (SPEC 10.2).
  // Stage 4a has no joint pulses and no Z motion to stop; the Z pins are driven to their idle level again
  // so the stop is already wired for the stages that add motion.
  void halt_motion_outputs() override { drive_z_motor_pins_idle(); }
};

SerialSink g_sink;
BoardHardware g_hardware;
CommandProcessor g_processor(&g_sink, &g_hardware);  // the constructor touches no pin
LineFramer g_framer;

}  // namespace

void setup() {
  hold_boot_levels();  // first, before anything else (SPEC 10.2)
  Serial.setRxBufferSize(chessbot::config::kSerialRxBufferBytes);
  Serial.begin(chessbot::config::kBaudRate);
  // No boot banner: every line on the wire must be a protocol response the PC can parse.
}

void loop() {
  while (Serial.available() > 0) {
    const int received = Serial.read();
    if (received < 0) {
      break;
    }
    switch (g_framer.feed(static_cast<uint8_t>(received))) {
      case LineFramer::Result::kLine:
        g_processor.handle_line(g_framer.text(), millis());
        break;
      case LineFramer::Result::kError:
        g_processor.handle_bad_frame();
        break;
      case LineFramer::Result::kNone:
        break;
    }
  }
  // The heartbeat watchdog (g_processor.tick(millis())) is NOT enforced in stage 4a: nothing moves yet, and
  // SPEC 12 puts the heartbeat test after the motors. The state and the rule are in the core and unit
  // tested; the stage that enables the heartbeat adds the call here.
}
