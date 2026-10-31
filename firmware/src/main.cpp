// Stages 4a and 4b-1 (S1): serial commands and joint motion. docs/SPEC.md sections 4.2, 4.3, 5 and 10.2.
//
// Handles INFO, PING, ENABLE, DISABLE, ESTOP, RESET (4a) and MOVE, ACCEL, WAIT, POS (S1); every other command parses
// and answers error:1 until its stage (see lib/chessbot_core/src/command_processor.cpp). The two joints are driven
// by FastAccelStepper 0.33.14. No Z, servo or ADS1115 code exists yet, so Z cannot move.
//
// The TB6600 ENA pins are NOT given to the library: FastAccelStepper::setEnablePin() drives the pin to the disabled
// level when it is called (0.33.14 source), which would release a joint at boot. This file drives ENA itself and only
// calls setAutoEnable(false), so the library never touches an ENA pin (SPEC 4.3, 10.2).
//
// All logic lives in lib/chessbot_core (no Arduino includes, unit tested on the host). This file only
// connects it to the pins and to Serial.
#include <Arduino.h>
#include <FastAccelStepper.h>
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
// before a command arrives. No pulse is emitted at boot: the steppers are connected right after this, and only a
// MOVE makes them step.
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

FastAccelStepperEngine g_engine;
FastAccelStepper* g_joint[chessbot::kJointCount] = {nullptr, nullptr};

// Connect the two joint steppers: PUL pin, DIR pin with its polarity and the TB6600 direction-change delay, and no
// automatic enable. The ENA pins are left to drive_pin() above (see the header comment of this file).
void connect_joint_steppers() {
  const uint8_t pul[chessbot::kJointCount] = {chessbot::pins::kTb6600Pul1, chessbot::pins::kTb6600Pul2};
  const uint8_t dir[chessbot::kJointCount] = {chessbot::pins::kTb6600Dir1, chessbot::pins::kTb6600Dir2};
  const bool dir_high_counts_up[chessbot::kJointCount] = {chessbot::config::kJoint1DirHighCountsUp,
                                                           chessbot::config::kJoint2DirHighCountsUp};
  g_engine.init();
  for (uint8_t joint = 0; joint < chessbot::kJointCount; ++joint) {
    g_joint[joint] = g_engine.stepperConnectToPin(pul[joint]);
    if (g_joint[joint] != nullptr) {
      g_joint[joint]->setDirectionPin(dir[joint], dir_high_counts_up[joint], chessbot::config::kDirectionChangeDelayUs);
      g_joint[joint]->setAutoEnable(false);
    }
  }
}

void force_stop_joints() {
  for (FastAccelStepper* stepper : g_joint) {
    if (stepper != nullptr) {
      stepper->forceStop();
    }
  }
}

class BoardHardware : public HardwareOps {
 public:
  void set_drivers_enabled(bool enabled) override { write_drivers_enable(enabled); }

  // ESTOP / heartbeat timeout: stop every motion output, NEVER the TB6600 ENA pins (SPEC 10.2). The joint pulses stop
  // at once (forceStop: no deceleration ramp; at the assumed speed ceiling that is a few hundred microsteps per
  // second, far below what the motor can stop from). The Z pins are driven to their idle level again.
  void halt_motion_outputs() override {
    force_stop_joints();
    drive_z_motor_pins_idle();
  }

  // DISABLE and RESET: stop the joint pulses, leave Z alone.
  void stop_joint_pulses() override { force_stop_joints(); }

  // One MOVE: each joint that has to run gets its own speed and acceleration (so both arrive together) and its
  // absolute target step count. Any refusal by the library stops both joints and reports failure.
  bool start_joint_move(const chessbot::JointMove& move) override {
    for (uint8_t joint = 0; joint < chessbot::kJointCount; ++joint) {
      if (!move.runs[joint]) {
        continue;
      }
      FastAccelStepper* stepper = g_joint[joint];
      if (stepper == nullptr || stepper->setSpeedInHz(move.speed_hz[joint]) != 0 ||
          stepper->setAcceleration(static_cast<int32_t>(move.accel_steps_s2[joint])) != 0 ||
          stepper->moveTo(move.target_steps[joint]) != MOVE_OK) {
        force_stop_joints();
        return false;
      }
    }
    return true;
  }

  bool joints_running() override {
    for (FastAccelStepper* stepper : g_joint) {
      if (stepper != nullptr && stepper->isRunning()) {
        return true;
      }
    }
    return false;
  }

  int32_t joint_steps_issued(uint8_t joint) override { return g_joint[joint] != nullptr ? g_joint[joint]->getCurrentPosition() : 0; }
};

SerialSink g_sink;
BoardHardware g_hardware;
CommandProcessor g_processor(&g_sink, &g_hardware);  // the constructor touches no pin
LineFramer g_framer;

}  // namespace

void setup() {
  hold_boot_levels();  // first, before anything else (SPEC 10.2)
  connect_joint_steppers();
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
  g_processor.poll_motion();  // a finished MOVE leaves the queue, the next one starts, a WAIT gets its done
  // The heartbeat watchdog (g_processor.tick(millis())) is still NOT enforced: SPEC 12 puts the heartbeat test after
  // the motors, and the owner scheduled it for stage 4b-5. The state and the rule are in the core and unit tested.
  // Until then a MOVE keeps running if the PC goes silent: keep a hand on the power switch (BRINGUP.md says how to stop).
}
