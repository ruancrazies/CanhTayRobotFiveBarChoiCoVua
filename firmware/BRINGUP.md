# Firmware bring-up, stage 4a

Stage 4a is the serial skeleton: `INFO`, `PING`, `ENABLE`, `DISABLE`, `ESTOP`, `RESET`. No stepper, Z, servo
or ADS1115 code exists yet. Every other command is understood but answers `error:1`.

Read SPEC section 14 before powering the motors. For this stage: motors disconnected from the arm (or the
arm in a safe position), TB6600 current set low, 24 V fused, your hand near the power switch.

## 1. Build

```
pip install platformio==6.2.0        # the version this project was set up with
cd firmware
pio run -e esp32dev
```

All versions are pinned in `platformio.ini`. Several pins could not be checked when they were written (the
PlatformIO registry was unreachable): **the first `pio run` is also the check of those pins.** If it fails
while resolving a package, paste the whole output back; do not loosen the pin by hand.

Host-side checks (no board needed):

```
pio test -e native                    # reference runner for the unit tests (Unity 2.6.1)
firmware/tools/run_host_tests.sh      # plain g++/clang++: unit tests, main.cpp against an Arduino stub,
                                      # every header compiled alone, SDK-style include shadowing, strict warnings
pytest tests/test_firmware_headers.py tests/test_firmware_layout.py
```

None of these compiles for the ESP32. They imitate what broke the first ESP32 build (libraries in `lib/` do not
see `include/`, PlatformIO appends project `-I` flags after the SDK's, `Arduino.h` defines macros such as `bit`),
but only `pio run -e esp32dev` on your machine proves the firmware builds. If it still fails, paste the first
error; if it names an Arduino macro or an SDK header, add that name to `firmware/tools/host_main/Arduino.h` so
the host checks catch it next time.

## 2. Flash

```
pio run -e esp32dev -t upload
```

If the upload hangs on "Connecting...", hold the BOOT button until it starts. Add `--upload-port <port>`
if more than one serial device is plugged in.

## 3. Monitor

```
pio device monitor -e esp32dev --echo
```

Baud 115200 (`kBaudRate` in `include/config.h`), Enter sends LF. `--echo` shows what you type, because the
firmware does not echo. Resetting the board (the monitor does this when it opens the port) prints a few
`ets ...` / `rst:...` lines from the ESP32 boot ROM before the firmware starts. Those are not protocol lines.
After that the firmware prints **nothing** until you send a command.

## 4. What to type and what you must see

Type each line, press Enter. Replies are exact.

| Type | Expected reply | Also check |
| --- | --- | --- |
| *(nothing, after reset)* | no output | multimeter: D13 and D26 at 3.3 V (drivers enabled, shafts hold); D4, D18, D19 at 0 V |
| `INFO` | `INFO FW=4a-0.1.0 PROTO=1` then `ok` | |
| `PING` | `ok` | |
| `DISABLE` | `ok` | D13 and D26 drop to 0 V; shafts turn freely by hand (SPEC 14 step 4) |
| `ENABLE` | `ok` | D13 and D26 back to 3.3 V; shafts hold |
| `ESTOP` | `!alarm:ESTOP` (no `ok`) | D13 and D26 do **not** change: drivers stay enabled |
| `ENABLE`, `DISABLE`, `MOVE A=1 B=2 F=3` after ESTOP | `error:5` each | |
| `PING`, `INFO`, `ESTOP` after ESTOP | `ok`, `INFO ...` + `ok`, `!alarm:ESTOP` | still answered in an ESTOP |
| `RESET` | `ok` | ESTOP cleared; `ENABLE` now answers `ok` again |
| `FOO` | `error:1` | unknown command |
| `ping` | `error:1` | names are case sensitive |
| `PING x` | `error:2` | unknown parameter |
| `GRIP S=181` | `error:2` | parameter out of range |
| `POS`, `HOME`, `MOVE A=1 B=2 F=3`, `ZUP`, `ZDOWN`, `ZSTOP`, `ACCEL S=1`, `GRIP S=90`, `WAIT` | `error:1` | stubs: parsed, not implemented in 4a |
| an empty line | no reply | |

If every row matches, stage 4a is done. If a row does not, paste the lines you typed and the lines you got.

## 5. Not in this stage

* The heartbeat is **not enforced**: no `!alarm:HB_TIMEOUT` is ever sent in 4a. `PING` only arms the
  watchdog inside the core (unit tested). The stage that tests the heartbeat adds one call in `loop()`.
* Z, joint pulses, limit switches, servo and sensors are untouched. D4, D18 and D19 are held LOW, so Z cannot
  move; PUL and DIR pins are not driven.
* Stage 4b starts only after you confirm the table above on the board.

## 6. Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| Garbage characters | monitor baud differs from `kBaudRate` (and `monitor_speed`) |
| No reply to anything | line ending is not LF (`monitor_eol = LF`), or wrong port |
| `error:1` for every line | a tab or non-ASCII character in the line; the protocol is printable ASCII only |
| Boot lines at the start | normal: ESP32 boot ROM, before the firmware |
