# Firmware bring-up, stage 4a

Stage 4a is the serial skeleton: `INFO`, `PING`, `ENABLE`, `DISABLE`, `ESTOP`, `RESET`. No stepper, Z, servo
or ADS1115 code exists yet. `HOME` and `MOVE` are still stubs: they are understood and pass through the gates of
SPEC 5.2 (`error:5`, `error:6`, `error:4`, `error:3`), and a `HOME` that passes them answers `error:1`. Every other
command is understood but answers `error:1`.

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
                                      # every header compiled alone, SDK-style include shadowing, strict warnings,
                                      # and the wire-compat check below
python3 firmware/tools/wire_compat/wire_compat.py   # C++ core vs Python ESP32 simulator vs the SPEC 5.1 rules
pytest tests/test_firmware_headers.py tests/test_firmware_layout.py tests/test_wire_compat.py
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
| `ENABLE` and `MOVE A=1 B=2 F=3` while in ESTOP | `error:5` each | D13 and D26 do not change; `ENABLE` is refused until `RESET` |
| `DISABLE` while in ESTOP | `ok` | D13 and D26 drop to 0 V: `DISABLE` is accepted in an ESTOP (SPEC 5.1); the ESTOP flag stays set |
| `ENABLE` again while still in ESTOP | `error:5` | D13 and D26 stay at 0 V |
| `ESTOP` again while in ESTOP | `!alarm:ESTOP` (no `ok`) | accepted and harmless: same reply, nothing else changes |
| `PING`, `INFO` while in ESTOP | `ok`, `INFO ...` + `ok` | still answered in an ESTOP (so are `POS` and `ZSTOP`, stubs in 4a: see the last table row) |
| `RESET` | `ok` | ESTOP cleared; the drivers stay as they were (still disabled after the `DISABLE` above) |
| `ENABLE` after `RESET` | `ok` | D13 and D26 back to 3.3 V |
| `MOVE A=1 B=2 F=3` (drivers enabled, no ESTOP) | `error:4` | not homed; the arm cannot be homed in 4a, so this is the only answer a `MOVE` can get here (never `error:1`) |
| `HOME`, `HOME Z`, `HOME AB` (drivers enabled) | `error:1` | stub: the line passed every gate and nothing runs |
| `DISABLE`, then `MOVE A=1 B=2 F=3` | `ok`, then `error:6` | D13 and D26 at 0 V and they stay there: a refusal never touches a pin. `error:6` (drivers disabled) comes **before** `error:4` |
| (drivers disabled) `HOME`, `HOME AB`, `HOME Z AB` | `error:6` each | a `HOME` that moves the joints needs the drivers. `HOME` never enables them, so D13 and D26 stay at 0 V |
| (drivers disabled) `HOME Z` | `error:1` | **not** `error:6`: a Z-only `HOME` needs no joint driver (it is the stub in 4a) |
| (drivers disabled) `MOVE A=1`, `FOO` | `error:2`, `error:1` | parse errors come before `error:6` |
| `ESTOP`, then `MOVE A=1 B=2 F=3`, `HOME`, `HOME Z` (drivers disabled or not) | `!alarm:ESTOP`, then `error:5` each | `error:5` comes before `error:6`; send `RESET` and `ENABLE` afterwards |
| `FOO` | `error:1` | unknown command |
| `ping` | `error:1` | names are case sensitive |
| `PING x` | `error:2` | unknown parameter |
| `GRIP S=181` | `error:2` | parameter out of range |
| `POS`, `ZUP`, `ZDOWN`, `ZSTOP`, `ACCEL S=1`, `GRIP S=90`, `WAIT` (not in an ESTOP) | `error:1` | stubs: parsed, not implemented in 4a (`HOME` is in the rows above). In an ESTOP only `POS` and `ZSTOP` still answer `error:1`; `ZUP`, `ZDOWN`, `ACCEL`, `GRIP` and `WAIT` answer `error:5` |
| an empty line | no reply | |

If every row matches, stage 4a is done. If a row does not, paste the lines you typed and the lines you got.

Rules behind the ESTOP rows (SPEC 5.1): while the ESTOP flag is set only `POS`, `INFO`, `PING`, `ZSTOP`,
`ESTOP`, `RESET` and `DISABLE` are accepted; every other command, `ENABLE` included, answers `error:5`.
`DISABLE` also clears the homed flag (homing is mandatory afterwards), stops the joint pulses and clears the whole
motion queue. The order of checks for every line is SPEC 5.2: unknown command (`error:1`), bad parameter
(`error:2`), ESTOP (`error:5`), drivers disabled (`error:6`), not homed (`error:4`), queue full (`error:3`). The homed
flag, a full queue and a queued `MOVE` cannot exist in stage 4a (`HOME` and `MOVE` are stubs), so those are covered
by the unit tests (`firmware/test/test_processor`), not by this table: see section 5.

## 5. What only becomes observable on hardware in stage 4b

The core already implements these rules and the host tests prove them, but nothing on the board can show them until
`HOME`, `MOVE`, `WAIT` and the joint pulses exist. Stage 4b's bring-up must check each row. Expected replies
(SPEC 5.1 "DISABLE rules" and "Cancelled done"):

| Do this (4b) | Expected | Why it cannot be seen in 4a |
| --- | --- | --- |
| `ENABLE`, `HOME`, wait for `done`, then `MOVE ...` | `ok`, `ok`, `done`, `ok` | no homing yet |
| after that homing: `DISABLE`, then `MOVE ...` | `ok`, then `error:6` (not `error:4`) | the homed flag was never set in 4a |
| `ENABLE`, then `MOVE ...` | `ok`, then `error:4` | `ENABLE` does not give the homing back |
| `DISABLE`, then `HOME` | `ok`, then `error:6`; D13 and D26 stay at 0 V | `HOME` never enables the drivers |
| `DISABLE`, then `HOME Z` | `ok`, `ok`, then `done` when Z reaches the top; the drivers stay disabled | Z homing does not exist yet |
| queue some `MOVE`s with a slow `F`, then `DISABLE` | `ok` per `MOVE`, `ok` for `DISABLE`; the shafts stop at once, the rest of the queue never runs | no pulses yet |
| `MOVE` (slow), `WAIT`, then `DISABLE` while `WAIT` is pending | `ok`, `ok` for `WAIT`, `ok` for `DISABLE`, and then **no `done` at all**: the `WAIT` is cancelled with the queue | no `WAIT` yet |
| the same, then `WAIT` again | `ok`, `done` at once | an empty queue is drained |
| `HOME` running, then `DISABLE` | `ok` for `HOME`, `ok` for `DISABLE`, **no `done`**, the joints are not homed | no `HOME` yet |
| `HOME Z` running, then `DISABLE` | `ok`, `ok`, then `done` (Z needs no joint driver and carries on) | no Z yet |
| `ZDOWN` or `ZUP` running, then `DISABLE` | `ok`, `ok`, then `done` (`DISABLE` does not stop Z) | no Z yet |
| `MOVE` queue full (`kQueueDepth` slow `MOVE`s, then one more) | `error:3`; the extra `MOVE` is not queued | needs the homed flag |
| `ESTOP`, `RESET`, a heartbeat timeout, or a new `HOME` while a `WAIT` is pending | no `done` for that `WAIT` (same rule as `DISABLE`) | no `WAIT`, no heartbeat in 4a |
| `MOVE` with Z not at TOP; `HOME AB` with Z not at TOP | `error:4` | the Z state does not exist yet |

## 6. Not in this stage

* The heartbeat is **not enforced**: no `!alarm:HB_TIMEOUT` is ever sent in 4a. `PING` only arms the
  watchdog inside the core (unit tested). The stage that tests the heartbeat adds one call in `loop()`.
* Z, joint pulses, limit switches, servo and sensors are untouched. D4, D18 and D19 are held LOW, so Z cannot
  move; PUL and DIR pins are not driven.
* Stage 4b starts only after you confirm the table above on the board.

## 7. Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| Garbage characters | monitor baud differs from `kBaudRate` (and `monitor_speed`) |
| No reply to anything | line ending is not LF (`monitor_eol = LF`), or wrong port |
| `error:1` for every line | a tab or non-ASCII character in the line; the protocol is printable ASCII only |
| Boot lines at the start | normal: ESP32 boot ROM, before the firmware |
