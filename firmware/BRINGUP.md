# Firmware bring-up, stage 4a

Stage 4a is the serial skeleton: `INFO`, `PING`, `ENABLE`, `DISABLE`, `ESTOP`, `RESET`. Sections 1 to 7 are the stage 4a
checks (they still hold; the `MOVE` lines in them use values inside the limits). Stage 4b-1 S1 adds joint motion
(`MOVE`, `ACCEL`, `WAIT`, `POS`): section 8; S2 adds the Z axis (`ZUP`, `ZDOWN`, `ZSTOP`): section 9. `HOME` is still a stub: it is understood, passes the gates of
SPEC 5.2 (`error:5`, `error:6`) and answers `error:1`. The servo and the sensors do not exist yet; the commands for
them are understood but answer `error:1`.

Read SPEC section 14 before powering the motors. For this stage: motors disconnected from the arm (or the
arm in a safe position), TB6600 current set low, 24 V fused, your hand near the power switch.

## 1. Build

```
pip install platformio==6.2.0        # the version this project was set up with
cd firmware
pio run -e esp32dev
```

`include/pins.h` and `include/config.h` are GENERATED from `config/settings.yaml`. To change a pin, the baud rate or any
other value, edit `config/settings.yaml`, then run (from the repository root)

```
python -m tools.gen_all               # rewrites pins.h, config.h and docs/CONFIG_REFERENCE.md
python -m tools.gen_all --check       # writes nothing; exit 1 when a generated file is stale
```

and commit the result. `monitor_speed` in `platformio.ini` must be changed by hand to the same `serial.baud`
(a test fails when they differ).

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

Baud 115200 (`serial.baud` in `config/settings.yaml`, which generates `kBaudRate` in `include/config.h`), Enter sends LF. `--echo` shows what you type, because the
firmware does not echo. Resetting the board (the monitor does this when it opens the port) prints a few
`ets ...` / `rst:...` lines from the ESP32 boot ROM before the firmware starts. Those are not protocol lines.
After that the firmware prints **nothing** until you send a command.

## 4. What to type and what you must see

Type each line, press Enter. Replies are exact.

| Type | Expected reply | Also check |
| --- | --- | --- |
| *(nothing, after reset)* | no output | multimeter: D13 and D26 at 3.3 V (drivers enabled, shafts hold); D4, D18, D19 at 0 V |
| `INFO` | `INFO FW=4b1-s2 PROTO=1` then `ok` | |
| `PING` | `ok` | |
| `DISABLE` | `ok` | D13 and D26 drop to 0 V; shafts turn freely by hand (SPEC 14 step 4) |
| `ENABLE` | `ok` | D13 and D26 back to 3.3 V; shafts hold |
| `ESTOP` | `!alarm:ESTOP` (no `ok`) | D13 and D26 do **not** change: drivers stay enabled |
| `ENABLE` and `MOVE A=90 B=45 F=20` while in ESTOP | `error:5` each | D13 and D26 do not change; `ENABLE` is refused until `RESET` |
| `DISABLE` while in ESTOP | `ok` | D13 and D26 drop to 0 V: `DISABLE` is accepted in an ESTOP (SPEC 5.1); the ESTOP flag stays set |
| `ENABLE` again while still in ESTOP | `error:5` | D13 and D26 stay at 0 V |
| `ESTOP` again while in ESTOP | `!alarm:ESTOP` (no `ok`) | accepted and harmless: same reply, nothing else changes |
| `PING`, `INFO` while in ESTOP | `ok`, `INFO ...` + `ok` | still answered in an ESTOP (so are `POS` and `ZSTOP`, stubs in 4a: see the last table row) |
| `RESET` | `ok` | ESTOP cleared; the drivers stay as they were (still disabled after the `DISABLE` above) |
| `ENABLE` after `RESET` | `ok` | D13 and D26 back to 3.3 V |
| `MOVE A=90 B=45 F=20` (drivers enabled, no ESTOP) | `error:4` | not homed; the arm cannot be homed in 4a, so this is the only answer a `MOVE` can get here (never `error:1`) |
| `HOME`, `HOME Z`, `HOME AB` (drivers enabled) | `error:1` | stub: the line passed every gate and nothing runs |
| `DISABLE`, then `MOVE A=90 B=45 F=20` | `ok`, then `error:6` | D13 and D26 at 0 V and they stay there: a refusal never touches a pin. `error:6` (drivers disabled) comes **before** `error:4` |
| (drivers disabled) `HOME`, `HOME AB`, `HOME Z AB` | `error:6` each | a `HOME` that moves the joints needs the drivers. `HOME` never enables them, so D13 and D26 stay at 0 V |
| (drivers disabled) `HOME Z` | `error:1` | **not** `error:6`: a Z-only `HOME` needs no joint driver (it is the stub in 4a) |
| (drivers disabled) `MOVE A=1`, `FOO` | `error:2`, `error:1` | parse errors come before `error:6` |
| `ESTOP`, then `MOVE A=90 B=45 F=20`, `HOME`, `HOME Z` (drivers disabled or not) | `!alarm:ESTOP`, then `error:5` each | `error:5` comes before `error:6`; send `RESET` and `ENABLE` afterwards |
| `FOO` | `error:1` | unknown command |
| `ping` | `error:1` | names are case sensitive |
| `PING x` | `error:2` | unknown parameter |
| `GRIP S=181` | `error:2` | parameter out of range |
| `GRIP S=90` (not in an ESTOP) | `error:1` | stub: parsed, not implemented yet (`HOME` is in the rows above; `MOVE`, `ACCEL`, `WAIT`, `POS` exist from S1, section 8, and `ZUP`, `ZDOWN`, `ZSTOP` from S2, section 9). In an ESTOP `ZUP`, `ZDOWN` and `GRIP` answer `error:5`, `ZSTOP` answers `ok` |
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
| `MOVE` with Z not at TOP; `HOME AB` with Z not at TOP | `error:4` | checked in S2: section 9 |

## 6. Not in this stage

* The heartbeat is **not enforced**: no `!alarm:HB_TIMEOUT` is ever sent in 4a. `PING` only arms the
  watchdog inside the core (unit tested). The stage that tests the heartbeat adds one call in `loop()`.
* Z, joint pulses, limit switches, servo and sensors are untouched. D4, D18 and D19 are held LOW, so Z cannot
  move; PUL and DIR pins are not driven.
* Stage 4b starts only after you confirm the table above on the board.

## 7. Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| Garbage characters | monitor baud differs from `serial.baud` (and `kBaudRate`, `monitor_speed`) |
| No reply to anything | line ending is not LF (`monitor_eol = LF`), or wrong port |
| `error:1` for every line | a tab or non-ASCII character in the line; the protocol is printable ASCII only |
| Boot lines at the start | normal: ESP32 boot ROM, before the firmware |

## 8. Stage 4b-1, S1: joint motion

New in this step: `MOVE`, `ACCEL`, `WAIT` and `POS` work, and both joints are driven by FastAccelStepper 0.33.14
(pinned in `platformio.ini`; `pio run -e esp32dev` is also the first check of that pin). `INFO` now says
`FW=4b1-s2`. **Not in S1:** `HOME` (still `error:1`), the servo, the AS5600 angles, and the heartbeat: a missing
`PING` still stops nothing until stage 4b-5. (Z, the L298 and the limit switches arrive in S2: section 9.)

**Before you start.** Motors off the arm, or the arm held so nothing can hit anything; TB6600 current low; your hand on
the 24 V switch. `MOVE` is `error:4` until the arm is homed and `HOME` does not exist yet, so for this bench test only:
in `config/settings.yaml` set `motion.bench_move_without_homing: true`, run `python -m tools.gen_all`, build and
flash. **Set it back to `false` afterwards.** Step counting starts at 0 at power-up, and 0 is angle 0: the soft
limits (alpha 60 to 200, beta -20 to 115) are checked on the angle you type, not on where the arm really is, so on
the bench a `MOVE` can drive a joint into its mechanical stop. Start with small distances.

Limits used below (all `config/settings.yaml`, all assumed until you confirm them): speed at most 30 deg/s,
acceleration at most 60 deg/s^2, 3200 steps per revolution of the motor (16 microsteps), gear ratio 1, so
8.889 steps per degree. DIR polarity is assumed: `electrical.tb6600.joint1_dir_high_counts_up` and
`joint2_dir_high_counts_up` are `true` (DIR HIGH = angle up).

Type each line, press Enter.

| Type | Expected reply | Also check |
| --- | --- | --- |
| `INFO` | `INFO FW=4b1-s2 PROTO=1`, `ok` | |
| `POS` | `POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP`, `ok` | SA, SB are always 0. With the S2 firmware `Z=` follows the limit switches (section 9): hold the TOP switch pressed (or leave the arm at the top) for the rows below, otherwise it reads `Z=MID` and every `MOVE` is `error:4` |
| `MOVE A=90 B=45 F=20` | `error:6` if you sent `DISABLE` before; otherwise `ok` | with the bench setting `false`: `error:4` |
| `ENABLE`, then `MOVE A=90 B=45 F=20` | `ok`, `ok` | both shafts turn and **arrive together** (A: 800 steps, B: 400 steps at 8.889/deg); listen for stalls and missed steps |
| `WAIT` (while it moves) | `ok`, then `done` when the motion has ended | `done` comes after the shafts stop, not before |
| `POS` (after it arrived) | `POS A=90 B=45 SA=0 SB=0 NA=800 NB=400 Z=TOP`, `ok` | NA and NB are the steps issued |
| `MOVE A=60 B=-20 F=20` | `ok` | the way back; `POS` then says `A=60 B=-20 ... NA=533 NB=-178` |
| `MOVE A=90 B=45 F=31` | `error:2` | above the speed ceiling; nothing moves |
| `MOVE A=59 B=45 F=20`, `MOVE A=90 B=116 F=20` | `error:2` each | outside the joint soft limits; nothing moves |
| `ACCEL S=61` then `ACCEL S=30` | `error:2`, `ok` | a slower `MOVE` after `ACCEL S=30` should ramp up and down more gently |
| `MOVE A=1` | `error:2` | missing parameter |
| nine `MOVE`s in a row with `F=1` (alternate `A=90` and `A=100`; queue depth 8, the running one counts) | `ok` for the first 8, `error:3` for the 9th | the 9th is not queued; finish with `ESTOP`, `RESET` |
| `MOVE A=200 B=100 F=2`, then `DISABLE` | `ok`, `ok` | **the shafts stop at once**, D13 and D26 drop to 0 V, the rest of the queue never runs |
| `MOVE ...` (slow), `WAIT`, then `DISABLE` while `WAIT` is pending | `ok`, `ok`, `ok`, and then **no `done`** | the `WAIT` is cancelled with the queue |
| `ENABLE`, `MOVE ...` (slow), then `ESTOP` | `ok`, `ok`, `!alarm:ESTOP` (no `ok`) | the shafts stop at once (a hard stop, no deceleration in S1); D13 and D26 stay at 3.3 V |
| `MOVE ...` while in ESTOP | `error:5` | until `RESET` |
| `RESET`, `MOVE ...` (slow), then `RESET` while it moves | `ok`, `ok`, `ok` | the shafts stop; no `done` for a `WAIT` that was pending |
| `MOVE A=90 B=45 F=20` when the joints are already there | `ok` | nothing moves, `WAIT` answers `ok`, `done` at once |

**How to stop safely.** Any time: `ESTOP` (the shafts stop, the drivers stay enabled so the joints hold), or
`DISABLE` (the shafts stop and are released: an arm that is not held will fall), or the 24 V switch. There is no
heartbeat in S1, so the firmware will carry on with a queued `MOVE` if the PC disappears: keep `F` small and your
hand on the switch. If a joint turns the wrong way, send `ESTOP`, then swap the matching
`joint1_dir_high_counts_up` / `joint2_dir_high_counts_up` value in `config/settings.yaml`, run `python -m tools.gen_all`
and flash again.

What to report: the lines you typed and the lines you got, the direction each joint turned for a positive angle,
whether the two joints stopped at the same moment, and any stall, whine or missed step. `POS` after a `MOVE` that
a stall interrupted still shows the steps that were issued, not where the shaft is.

## 9. Stage 4b-1, S2: the Z axis

New in this step: `ZUP`, `ZDOWN` and `ZSTOP` work, the two limit switches are read (D35 = TOP, D34 = BOTTOM), `POS` reports
`Z=TOP`, `Z=BOTTOM` or `Z=MID`, and `MOVE` needs Z at TOP (`error:4` otherwise, also with the bench setting). `INFO` says
`FW=4b1-s2`. **Not in S2:** `HOME` (still `error:1`), the servo, the AS5600, and the heartbeat (a missing `PING` still
stops nothing until stage 4b-5).

**Before you start (SPEC 14 steps 5 and 6).** First the switches with the motor unplugged, then the motor with the carriage
free to move and your hand on the L298 supply switch. A Z that falls when power is cut can be damaged: hold it or keep it
off the mechanism until you trust the direction and the stops. The first motor run should use a **short timeout**: in
`config/settings.yaml` set `electrical.z_axis.full_travel_ms` to a small value such as `1000` (the timeout is 1.5 times it),
run `python -m tools.gen_all`, build and flash; put the real measured travel time back afterwards (SPEC 13).

Values in use (all assumed until you confirm them): travel 5000 ms, timeout factor 1.5 (so 7.5 s), PWM 10 kHz, duty 40 % ramping
to 70 % over 500 ms and dropping to 40 % after 80 % of the travel time, debounce 8 ms. Switches are normally closed: pressed
or a broken wire reads actuated.

| Do this | Expected | Also check |
| --- | --- | --- |
| reset the board, motor unplugged | no output | D4, D18, D19 at 0 V (as in section 4) |
| `INFO` | `INFO FW=4b1-s2 PROTO=1`, `ok` | |
| `POS`, no switch pressed | `POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=MID`, `ok` | Z is MID at boot until a switch says otherwise |
| press and hold the TOP switch, `POS` | `Z=TOP` | |
| release the TOP switch with no command | `!alarm:LIMIT_Z` | Z leaving TOP without a command is an emergency stop (the ESTOP flag is set): send `RESET` |
| press and hold the BOTTOM switch, `POS`, release it | `Z=BOTTOM`, then no alarm after the release | only TOP has the release rule |
| press both switches | `!alarm:LIMIT_Z` (once, not repeated) | `RESET` and release both |
| unplug the TOP switch wire, `POS` | `Z=TOP` | a broken wire reads like a pressed switch (fail-safe); plug it back |
| `MOVE A=90 B=45 F=20` (bench setting on) with Z not at TOP | `error:4` | with Z at TOP it is `ok` |
| (TOP held) `ZUP` | `ok`, `done` | nothing moves: Z is already at TOP |
| motor plugged in, TOP held, `ZDOWN`, then release TOP | `ok`; the motor turns, no alarm for the release | **note which way it turns.** IN1 = raise, IN2 = lower: if `ZDOWN` raises, swap the two motor wires at the L298 output |
| press BOTTOM while it runs | the motor stops at once; `done` about 10 ms later | D4 steady 3.3 V and D18, D19 both 0 V: the electric brake (ENA on, IN1 equal IN2) |
| `POS` | `Z=BOTTOM` | |
| `ZUP`, press TOP while it runs | `ok`, the motor stops at once, `done` | the brake holds until the next Z command |
| `ZDOWN` and press no switch | after 1.5 times `full_travel_ms` the motor stops and `!alarm:Z_TIMEOUT` is sent once; **no** `done` | the motor stays braked. `RESET` |
| `ZDOWN`, then `ZSTOP` while it runs | `ok`, `ok`; the motor stops; no `done` | and no `LIMIT_Z` when you send it right away |
| `ZDOWN`, then `ESTOP` while it runs | `ok`, `!alarm:ESTOP`; the motor stops; no `done` | `RESET` afterwards |
| `ZDOWN`, then `DISABLE` while it runs | `ok`, `ok`; the motor carries on and `done` arrives at the bottom | `DISABLE` never stops Z (SPEC 5.1) |
| `ZDOWN`, then `ZUP` while it runs | `ok`, `ok`; the motor reverses; one `done` at the top | a new Z command replaces the running one. Avoid on a loaded axis |
| `DISABLE`, `ZDOWN` | `ok`, `ok` | Z needs no joint driver |
| `ESTOP`, then `ZUP` | `!alarm:ESTOP`, `error:5` | `ZSTOP` is still `ok` in an ESTOP |
| press both switches while a Z move runs | the motor stops at once, `!alarm:LIMIT_Z` only (no `done`) | |

**How to stop safely.** `ZSTOP` brakes Z; `ESTOP` brakes Z and stops the joints (the drivers stay enabled); the L298 supply switch
cuts everything. The brake only holds while the L298 is powered: cutting its supply releases Z. Cutting a switch wire stops the
motor in the direction of that switch and is read as "arrived" (`done`), which is the intended fail-safe. There is no
heartbeat in S2: a Z move that is running carries on until a switch or the timeout stops it, even if the PC disappears.

What to report: the lines you typed and the lines you got, which way `ZDOWN` turned the motor, the measured travel time
(from `ZDOWN` to `done`), whether the stop at a switch felt hard (a jerk) or soft, the voltage on D4, D18, D19 during the
run and during the stop, and any `LIMIT_Z` or `Z_TIMEOUT` you did not expect.
