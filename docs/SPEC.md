# Vibe Coding Spec: Five-Bar Chess Robot

English build spec for AI coding agents (Claude, Cursor, etc.) and the human operator. It covers a five-bar chess-playing robot arm with computer vision. All code, identifiers, comments, logs, docs and protocol text are in English. **Only user-facing GUI strings are Vietnamese** (see section 11).

## 1. How to use this spec

- Keep this file in the repo (docs/SPEC.md) or in the Claude Project knowledge. Section 2 is the condensed rule set to use as project instructions.
- Work phase by phase (section 12). Give the AI one phase plus the sections it needs, never the whole system at once.
- Every phase has a definition of done. Do not start the next phase until it is met.
- Touch real hardware only from phase 4. Before that, use the ESP32 simulator (phase 3).
- Any value marked \[TO CONFIRM\] (section 13) must live in a config file and be asked about, never guessed.

## 2. Invariant rules for the AI

1. **Single source of truth for the game:** only GameState (wrapping python-chess) may change the position. Camera results are accepted only when they match a legal move, and never overwrite the position.
2. **core/ never imports gui/.** The GUI only listens to Event Bus events and sends requests back through the Event Bus. The FSM decides whether to accept a request.
3. **Separate game state (GameState) from operating phase (FSM).**
4. **Move Detector runs only in WAIT\_HUMAN.** The robot arm must never be mistaken for a human hand.
5. **PC computes, ESP32 does real time and safety.** Never put safety logic only on the PC.
6. **Protocol is declared once:** config/protocol\_commands.py (PC) and include/protocol.h (firmware), with a test that cross-checks both.
7. **No hard-coded hardware constants.** Everything lives in config/hardware.yaml (PC) and include/pins.h, include/config.h (firmware).
8. **Every dangerous action has a timeout and a stop path:** Z has a timeout, Serial has a heartbeat, ESTOP is always available.
9. **Every module has tests** (pytest on the PC; pure-logic tests on firmware where possible). A feature is not done without tests.
10. **Code style:** type hints, leveled logging (no print), English identifiers and comments.
11. **Language policy:** all user-visible GUI text is Vietnamese and lives only in desktop\_app/gui/strings\_vi.py. Internal enums, state names, alarm codes, config keys, logs and protocol stay English. Never put Vietnamese in core/, hardware/, kinematics/ or vision/.

## 3. Repository layout

```
chess-robot/
├─ docs/SPEC.md
├─ desktop_app/
│  ├─ main.py
│  ├─ core/                  # must not import gui/
│  │  ├─ event_bus.py
│  │  ├─ game_state.py
│  │  ├─ fsm.py
│  │  ├─ move_planner.py
│  │  ├─ engine.py           # Stockfish via UCI
│  │  └─ health.py
│  ├─ vision/
│  │  ├─ camera_manager.py
│  │  ├─ calibration.py
│  │  ├─ yolo_detector.py
│  │  ├─ opencv_fallback.py
│  │  └─ move_detector.py
│  ├─ kinematics/
│  │  ├─ five_bar.py         # FK, IK, Jacobian, singularity margin
│  │  ├─ trajectory.py
│  │  ├─ dimension.py        # geometry, square centers, park/capture zones
│  │  └─ correction_map.py
│  ├─ hardware/
│  │  ├─ protocol.py
│  │  ├─ serial_link.py
│  │  ├─ robot_commander.py
│  │  └─ esp32_simulator.py  # simulator for testing without hardware
│  ├─ gui/                   # PySide6: tabs Play, Log, Calibration, Maintenance
│  │  └─ strings_vi.py       # ALL Vietnamese UI text lives here
│  └─ utils/theme.py
├─ config/                   # hardware.yaml, difficulty_levels.json, protocol_commands.py, grasp_profiles.json
├─ data/                     # games/, models/yolo/, calibration/, correction_map.json
├─ tests/
└─ firmware/                 # PlatformIO, ESP32
   ├─ platformio.ini
   ├─ include/               # pins.h, config.h, protocol.h
   └─ src/                   # main.cpp, serial_cmd.cpp, motion.cpp, zaxis.cpp, gripper.cpp, sensors.cpp, safety.cpp
```

## 4. Hardware constants (source of truth)

### 4.1 Geometry and motion

| Symbol | Meaning | Value |
| --- | --- | --- |
| l | Distance between the two motor axes | 100 mm |
| m | Driving link length | 200 mm |
| n | Driven link length | 250 mm |
| k | Motor axes to near edge of board (border included) | 150 mm |
| s | Square size | 31 mm |
| border | Board border | 26 mm |
| h | Z travel | 70 mm |
| f | TB6600 microstepping | 16 (SW1 OFF, SW2 OFF, SW3 ON) |
| i | Gear ratio, motor to driving link | \[TO CONFIRM\], default 1 |
| pulses/rev | 200 × f × i | 3200 when i = 1 |
| θ | Joint angle resolution | 0.1125° when i = 1 |

**Coordinate frame:** origin O at the midpoint of the two motor axes, +Y toward the board. Motor #1 sits at (−l/2, 0) and turns by angle α; motor #2 sits at (+l/2, 0) and turns by angle β. The 300 mm square board is centered on the Y axis, board center at (0, 300). Rank 1 is nearest the robot, file A is on the left (x < 0). Square center: `x = −108.5 + 31·file` (A = 0), `y = 191.5 + 31·(rank − 1)`.

### 4.2 ESP32 pin map

| Pin | Connected to | Note |
| --- | --- | --- |
| D13 | TB6600 #1 ENA- | joint 1 (α) |
| D14 | TB6600 #1 DIR- |  |
| D27 | TB6600 #1 PUL- |  |
| D26 | TB6600 #2 ENA- | joint 2 (β) |
| D25 | TB6600 #2 DIR- |  |
| D33 | TB6600 #2 PUL- |  |
| D4 | L298 ENA | Z motor PWM, about 10 kHz |
| D18 | L298 IN1 | raise direction |
| D19 | L298 IN2 | lower direction |
| D21 / D22 | ADS1115 SDA / SCL | address 0x48 (ADDR to GND) |
| D32 | SG90 PWM | 50 Hz, separate LEDC channel/timer |
| D35 | Limit switch #1 (TOP) | input only, external 10 kΩ pull-down |
| D34 | Limit switch #2 (BOTTOM) | input only, external 10 kΩ pull-down |

### 4.3 Electrical conventions

- **TB6600:** common-anode wiring. The three (+) inputs go to 3V3 and the three (−) inputs go to GPIOs; a LOW GPIO makes the opto conduct. **Active ENA means driver OFF:** GPIO HIGH or floating = driver enabled; GPIO LOW = driver disabled. Therefore `setEnablePin(pin, false)`, `setAutoEnable(false)`, and DISABLE pulls the GPIO LOW. Configure these pins at the very start of setup().
- **Limit switches (NC):** COM to 3V3, NC to the signal pin, 10 kΩ pull-down. Normally HIGH; actuated or wire broken = LOW (fail-safe). Debounce 5–10 ms in firmware.
- **AS5600 (analog, full range 0–100% of Vdd, default OUTS = 00, not programmed):** ADS1115 A0 = AS5600 #2 (β), A1 = AS5600 #1 (α), A2 = Vdd, PGA ±4.096 V. Compute `r = V_OUT / V_A2` and sensor angle `θ = k·r·360° + b`. k and b are per sensor and stored in data/calibration/encoder\_calibration.json. If `r < 0.02` or `r > 0.98`, raise `!alarm:SENSOR_FAULT`. System error is about ±1°, so use it only for lost-step detection and homing.
- **Z axis:** DC motor through L298, IN1 = raise, IN2 = lower. Only two positions: TOP (limit #1, homing reference) and BOTTOM (limit #2). A limit switch stops the motor inside the interrupt. Timeout per move is about 1.5 × the full-travel time.
- **SG90 servo:** pulse 500–2500 µs for 0–180°, separate 5 V supply, common GND with the ESP32.

## 5. Serial protocol

Line-based text, G-code style, each command ends with a newline. Format: `NAME KEY=value ...`, for example `MOVE A=135.4 B=44.6 F=60`. Angles in degrees, lengths in mm, decimal point is a dot. Baud rate \[TO CONFIRM\], suggested 115200.

### 5.1 Commands

| Command | Function | Response |
| --- | --- | --- |
| HOME \[Z\] \[AB\] | Homing. No argument: Z first (raise until limit #1), then the two joints using AS5600 | ok, then done |
| MOVE A=<α> B=<β> F=\<speed> | Move both joints to target angles. No Z parameter | ok when queued; error:4 if not homed or Z not at TOP; error:2 if out of limits |
| ZUP | Raise Z until limit #1 triggers | ok, then done |
| ZDOWN | Lower Z until limit #2 triggers | ok, then done |
| ZSTOP | Stop and brake Z immediately | ok |
| ACCEL S=\<deg/s²> | Acceleration for following segments | ok |
| GRIP S=<0–180> | Gripper servo angle | ok |
| ENABLE / DISABLE | Enable / disable both TB6600 drivers | ok |
| WAIT | Wait for the motion queue to drain | ok, then done |
| POS | Commanded angles, AS5600 angles, steps issued, Z state (TOP / BOTTOM / MID) | one line, then ok |
| INFO | Firmware and protocol version | one line, then ok |
| PING | Heartbeat from the PC | ok |
| ESTOP | Stop joint pulses (drivers stay enabled), stop and brake Z, set ESTOP flag | !alarm:ESTOP |
| RESET | Clear ESTOP and alarms; homing is mandatory afterwards | ok |

### 5.2 Responses

- `ok`: valid command accepted and queued.
- `done`: motion queue finished (after HOME or WAIT).
- `error:<code>`: 1 unrecognized; 2 missing or out-of-range parameter; 3 queue full; 4 not homed; 5 in ESTOP.
- `!alarm:<code>` sent asynchronously: `ESTOP`, `LIMIT_Z` (both Z switches active, or Z at TOP with the top switch released without a command), `Z_TIMEOUT`, `SENSOR_FAULT`; plus `STEP_LOSS` and `HB_TIMEOUT` (proposed).

### 5.3 Rules for the PC side

- Keep only a few unacknowledged commands in flight so the ESP32 queue always has work without overflowing.
- Send PING periodically. If the ESP32 misses heartbeats beyond the threshold it stops safely (section 10).
- After ESTOP, lost steps, HB\_TIMEOUT or SENSOR\_FAULT, homing is mandatory.

## 6. Kinematics and trajectory

### 6.1 Formulas

```
Elbows:  A = (−l/2 + m·cos α,  m·sin α)
         B = (+l/2 + m·cos β,  m·sin β)

FK:      AB = |B − A|; require 0 < AB < 2n, otherwise raise an error
         w  = sqrt(n² − AB²/4)
         P  = (A + B)/2 + w·( (yA − yB)/AB ,  (xB − xA)/AB )

IK:      PD = hypot(x + l/2, y);   PC = hypot(x − l/2, y)
         require m − n ≤ PC, PD ≤ m + n, otherwise out of reach
         α = atan2(y, x + l/2) + acos((m² − n² + PD²) / (2·m·PD))
         β = atan2(y, x − l/2) − acos((m² − n² + PC²) / (2·m·PC))
         (elbows-out configuration only; never use elbows-in)

Steps:   N = 200·f·i·(angle − angle0) / 360
```

### 6.2 Conditions and test vectors

- **Singularity margin:** `μ = min(∠APB, 180° − ∠APB, φA, 180° − φA, φB, 180° − φB)`, where φ is the angle between the driving and driven link of the same branch. Require `μ ≥ 15°` at every pick point and every trajectory point.
- **Round trip:** FK(IK(P)) = P within 1e-6 mm for all 64 square centers.
- **Test vectors** (tolerance ±0.1°):

| Square | x, y (mm) | α | β |
| --- | --- | --- | --- |
| A1 | −108.5; 191.5 | 184.3° | 62.9° |
| H1 | 108.5; 191.5 | 117.1° | −4.3° |
| A8 | −108.5; 408.5 | 124.6° | 96.5° |
| H8 | 108.5; 408.5 | 83.5° | 55.4° |

- Over the 64 squares: α spans 83.5° to 184.3°; β spans −4.3° to 96.5°; the smallest μ is about 26° (squares A8, H8); the Jacobian condition number is between 1.03 and 2.79. Tests must confirm these numbers.

### 6.3 Trajectory

- The gripper path is a straight line from source to target, split into segments no longer than Δs (configurable, default 2 mm). Solve IK at each point and send `MOVE`; both joints finish together.
- **First version:** send plain MOVE commands at a fixed speed F, then measure straightness and vibration on the hardware. Only if needed upgrade to a trapezoidal velocity profile over the whole path (firmware uses FastAccelStepper low-level addQueueEntry), or send time-uniform joint points.
- Every trajectory point is checked for workspace and singularity. On any violation, report an error **before sending any command**.

### 6.4 Pick-and-place sequence

`MOVE` above the source square (Z at TOP) → `ZDOWN` → `GRIP` close using the piece's grasp angle (config/grasp\_profiles.json) → `ZUP` → `MOVE` above the target square → `ZDOWN` → `GRIP` open → `ZUP`.

- **Capture:** remove the captured piece to the capture zone (off the board) first, then move the capturing piece.
- **En passant:** the captured pawn is on a different square than the target; remove it to the capture zone, then move.
- **Castling:** move the king first, then the rook.
- **Promotion:** move the pawn off the board, take the new piece from the reserve zone and place it on the promotion square.
- Capture zone, reserve zone and park position (outside the camera view) are defined in dimension.py and pass the same workspace and μ ≥ 15° checks. \[TO CONFIRM: coordinates\]

## 7. GameState and FSM

### 7.1 GameState

Wraps `chess.Board`; the only place allowed to change the position.

- `commit_move(move, source)`: validates, updates, records history, emits `game_updated`. `source` is `human` or `engine`.
- `preview_move(move)`: returns a copy of the board after a trial move without changing the real position; used to compute the expected position for VERIFY.
- `pending_move`: the engine move, saved together with `current.fen` to data/games/pending.json from ENGINE\_THINK until VERIFY succeeds; only then `commit_move(source=engine)`. If the app starts and finds a pending\_move, it enters SETUP\_CHECK, compares the camera with the position before and after that move, then redoes or cancels it.
- Provides FEN to Stockfish and `legal_moves` to the Move Detector.
- Saves `current.fen` and PGN after every commit so a game can be reloaded after an abrupt shutdown.
- Detects game end (checkmate, draw, no legal moves) and moves the FSM to GAME\_OVER.

### 7.2 State table

| State | Work | Transitions |
| --- | --- | --- |
| IDLE | Program off (YOLO off): camera preview only, ESP32 still connected, heartbeat still sent, drivers still enabled | INIT when YOLO is switched on |
| INIT | Connect ESP32, camera, engine; load YOLO model; load config and calibration; check pending.json | CALIBRATE |
| CALIBRATE / HOMING | Check camera calibration; home Z, then both joints | SETUP\_CHECK |
| SETUP\_CHECK | Camera confirms the required position; user picks side and mode, presses Start | WAIT\_HUMAN, ENGINE\_THINK, or PLAN (redo pending\_move) |
| WAIT\_HUMAN | Enable Move Detector; wait for a stable legal move (or manual confirm) | COMMIT\_HUMAN |
| COMMIT\_HUMAN | `commit_move(human)`; check game end | ENGINE\_THINK or GAME\_OVER |
| ENGINE\_THINK | Ask Stockfish, store `pending_move` (not committed) | PLAN |
| PLAN | Move Planner builds the command chain (capture, castling, promotion...) | ROBOT\_EXEC |
| ROBOT\_EXEC | Send commands, wait for ok and done; Move Detector locked | VERIFY or ERROR\_RECOVER |
| VERIFY | Robot to park position, capture image, YOLO compares piece types per square with the expected position; match → `commit_move(engine)`, update CorrectionMap | WAIT\_HUMAN, GAME\_OVER or ERROR\_RECOVER |
| PAUSED | Keep GameState, Move Detector locked, robot at park position | Previous state |
| ERROR\_RECOVER | Report error, highlight suspect squares, wait for Resync. After ESTOP, lost steps, HB\_TIMEOUT or SENSOR\_FAULT, homing is mandatory; keep pending\_move | WAIT\_HUMAN, PLAN or HOMING |
| GAME\_OVER | Show result, save PGN | INIT or SETUP\_CHECK |

The robot moves only in ENGINE\_THINK, PLAN and ROBOT\_EXEC; in those phases the camera is locked out of move detection.

### 7.3 Pause and YOLO on/off rules

- **Pause:** in WAIT\_HUMAN it takes effect immediately; in ENGINE\_THINK or PLAN wait for a proposed move, then stop before the robot moves; in ROBOT\_EXEC finish the pick-and-place and VERIFY before entering PAUSED, so no piece is left hanging. The only immediate stop is the Emergency Stop button.
- **YOLO off (any phase):** finish the running robot chain, park the arm, raise Z, save FEN/PGN, then enter IDLE. If in the middle of ROBOT\_EXEC, keep pending\_move.
- **YOLO on:** enter INIT; if an unfinished game exists, ask whether to continue or start a new one. If the model cannot be loaded, warn and ask whether to run the OpenCV fallback.

## 8. Computer vision

### 8.1 Camera and calibration

- **Camera Manager** runs in its own thread and always keeps the latest frame; other modules get frames through it. Lock exposure, white balance and focus. Detect lost frames and reopen. Store the camera by device name, not just index; the user confirms with a preview. Scan for cameras only in INIT, ERROR\_RECOVER, or when the dialog is open, never while the robot is running.
- **Intrinsics:** checkerboard from at least 20 poses, saved to calibration.json.
- **Homography:** four board corners, `cv2.warpPerspective`, 8×8 grid.
- **Board-to-robot mapping:** touch four corners (or four reference squares) with the gripper to compute the transform from board squares to robot mm.
- **Empty-square reference image** for the OpenCV fallback.
- Camera looks down obliquely at 45°–60°, 1080p, 30 fps, with even LED lighting. The arm must be at the park position outside the camera view before the verification image is taken.

### 8.2 YOLO26 (primary detector)

- **14 classes, in this exact order in classes.yaml:** 12 piece types (P, N, B, R, Q, K for each color), `hand`, `robot_gripper`. The last two support the veto mechanism.
- **Square assignment:** take the bottom-center point of the bounding box (the piece's foot) and pass it through the homography. Do not use the box center, because tall pieces would shift into a neighboring square. If a square has several detections, keep the highest confidence.
- **Training data:** about 1000–2000 images of the actual board, using the exact piece set, all 12 types, many positions, partial occlusion by hand and gripper, varying light. Auto-label by placing positions generated from FEN by python-chess and projecting back through the inverse homography. Fine-tune from COCO weights.
- **Model loading:** choose from data/models/yolo/ or browse for a .pt/.onnx file. Verify classes.yaml has all 14 classes in order; on error keep the old model and report. Switch models only in IDLE, PAUSED or SETUP\_CHECK.
- **Evaluation:** by the rate of correctly recognized moves over many test games, not mAP alone.

### 8.3 Move Detector

Uses the position as the signal; motion is only used to discard bad frames. Active only in WAIT\_HUMAN.

1. Each frame: compute the expected grid for the current position (not yet moved) and for every legal move (about 20–40 hypotheses), and score matching squares (correct piece type and color) against the observed grid from YOLO. Low-confidence squares (occluded, piece lifted) are excluded from the score rather than counted as wrong.
2. Acceptance: best score reaches a threshold (for example ≥ 62/64 over trusted squares); the squares that differ between the best hypothesis and the unmoved position (2–4 squares) must match exactly; the score beats the second-best hypothesis by a safety margin.
3. N-frame stability: the same move recognized continuously for about 1 second (for example 15 frames); a frame with a different move resets the counter to 0.
4. Veto: a human hand in the board area (class `hand`) resets the stability counter.
5. Special cases: lifting a piece and putting it back is not a move; promotion takes the new piece type from YOLO, defaulting to Queen if uncertain and asking the user to choose in the GUI; warn if nothing matches for 10 seconds; always provide a manual confirm button.

### 8.4 OpenCV fallback

After homography, crop the center of each square (skip the border) and compare with the reference image to see if the square is occupied, then check the brightness of the top of the piece to tell white from black. It cannot tell piece types: Move Detector compares only empty/white/black, promotion defaults to Queen, and VERIFY cannot check piece types.

## 9. Chess engine

Stockfish over UCI with python-chess. Always query with the FEN from GameState so nothing drifts if the engine process restarts. If the engine does not answer within the time limit, restart it and reload the FEN. Config lives in config/difficulty\_levels.json; a new mode takes effect from the next engine move. Internal keys are English; display names are Vietnamese (section 11).

| # | Key | Display (Vietnamese) | Stockfish config | Think time |
| --- | --- | --- | --- | --- |
| 1 | beginner | Tập sự | Skill Level 0, depth ≤ 2 | 0.2 s |
| 2 | amateur | Nghiệp dư | UCI\_LimitStrength on, UCI\_Elo ≈ 1400 | 0.5 s |
| 3 | player | Kỳ thủ | UCI\_Elo ≈ 1800 | 1 s |
| 4 | master | Đại sư | UCI\_Elo ≈ 2400 | 2 s |
| 5 | grandmaster | Chiến thần | Limit off, full strength | 3 s |

Values are starting points; tune with real players. In Stockfish 16 and later UCI\_Elo ranges from 1320 to 3190, so the beginner level must use Skill Level and depth.

## 10. ESP32 firmware

### 10.1 Architecture

PlatformIO, C++, FreeRTOS with several tasks, FastAccelStepper library.

| Task | Responsibility |
| --- | --- |
| serial\_cmd | Read lines, parse commands, reply ok / error, push commands into the queue |
| motion | Take commands from the queue, generate joint pulses via FastAccelStepper, synchronize both joints to finish together |
| zaxis | Raise/lower Z by PWM, timeout, state TOP / BOTTOM / MID |
| sensors | Read ADS1115 (A0, A1, A2), average several samples, compute r and angle, detect SENSOR\_FAULT |
| safety | Heartbeat watchdog, ESTOP flag, highest priority |

### 10.2 Mandatory requirements

- **Boot:** configure the three ENA pins at the very start of setup(). After reset the pins float, so drivers are enabled, which is intentional so the motors hold position. Emit no pulses before a command arrives.
- **FastAccelStepper:** `setEnablePin(pin, false)` and `setAutoEnable(false)`; enable/disable only through ENABLE / DISABLE. Check function names against the library version in use. The DIR opto is slow, so set a direction-change delay as the TB6600 requires.
- **PWM:** LEDC for the Z motor (about 10 kHz) and the servo (50 Hz) use separate channels and timers that do not collide.
- **Z axis:** ZUP runs until limit #1 triggers; ZDOWN runs until limit #2 triggers. A limit switch stops the motor inside the interrupt (PWM = 0, brake). Moderate PWM, ramped up, eased near the end of travel. On timeout stop and send `!alarm:Z_TIMEOUT`. `LIMIT_Z` when both switches are active, or when Z is at TOP and the top switch releases without a command.
- **Heartbeat:** if PING is missing beyond the threshold, decelerate both joints in a controlled way, stop and brake Z, **keep drivers enabled** (releasing the motors can collapse the arm and drop Z), wait for the PC to reconnect and re-home.
- **ESTOP:** stop joint pulses (short deceleration if moving fast; stop at once if triggered by a hardware button or LIMIT\_Z), stop and brake Z, keep drivers enabled, set the ESTOP flag. DISABLE only when the user commands a shutdown or a driver reports over-temperature or over-current.
- **Homing:** Z first (raise slowly until limit #1), then both joints using the AS5600 angle. Every MOVE is rejected with error:4 until homed.
- **External pull-downs on D34 / D35,** and 5–10 ms debounce in firmware.

## 11. Desktop GUI (PySide6) and Vietnamese UI strings

### 11.1 Behavior

- Four tabs: Play, Log, Calibration, Maintenance. The Play tab has three areas: left is the virtual chessboard (drawn from GameState, never from the camera), middle is the camera view with diagnostic overlays, right is a single control widget (camera selection, model selection, YOLO on/off switch, play mode, side, Start, Pause/Resume, New game, Confirm, Resync, move list, Emergency Stop).
- **Which button is enabled in which phase is decided by the FSM;** the GUI only enables or disables buttons based on the state received from the Event Bus. The Emergency Stop button is always visible and always clickable.
- Results from the camera, vision, engine and Robot Commander threads reach the UI through queued Signal/Slot; the camera view only takes the newest frame. A frozen GUI does not affect safety because the ESP32 has a watchdog.
- Calibration tab: camera intrinsics, four board corners, empty-square image, board-to-robot mapping, k and b of the two AS5600 sensors, homing. Maintenance tab: jog the robot when the FSM is idle, reset CorrectionMap, back up and restore calibration, check connections.
- Minimum window 1280×720, scales with the window, supports high-DPI, styling centralized in utils/theme.py (QSS).

### 11.2 Vietnamese string rules

- **All user-visible text is Vietnamese** and is defined only in `desktop_app/gui/strings_vi.py`, as a dictionary keyed by stable English IDs (for example `state.WAIT_HUMAN`, `alarm.LIMIT_Z`, `button.start`).
- Internal values (FSM state names, alarm codes, engine mode keys, protocol text, config keys, log messages) stay English. Only the GUI layer maps an ID to Vietnamese.
- Use UTF-8 everywhere. Make sure the chosen font renders Vietnamese diacritics correctly.
- **Tests:** (1) every FSM state, alarm code and engine mode key has a Vietnamese string, with no missing keys; (2) a grep-style test confirms no Vietnamese characters appear outside strings\_vi.py and the docs.
- The strings below are defaults taken from the architecture document or proposed here; the owner may reword them.

### 11.3 Strings: tabs, controls, sides, pieces

| ID | Vietnamese |
| --- | --- |
| tab.play | Chơi |
| tab.log | Nhật ký |
| tab.calibration | Hiệu chỉnh |
| tab.maintenance | Bảo trì |
| button.start | Bắt đầu |
| button.pause | Tạm dừng |
| button.resume | Tiếp tục |
| button.new\_game | Ván mới |
| button.confirm | Xác nhận |
| button.resync | Đồng bộ lại |
| button.retry | Thử lại |
| button.rehome | Homing lại |
| button.cancel | Hủy |
| button.save\_pgn | Lưu PGN |
| button.quit | Thoát |
| button.estop | Dừng khẩn cấp |
| button.refresh | Làm mới |
| button.connect / button.disconnect | Kết nối / Ngắt |
| button.browse | Duyệt… |
| label.camera | Camera |
| label.model | Mô hình |
| label.yolo\_switch | Bật/Tắt YOLO |
| label.side | Chọn màu quân |
| label.mode | Chế độ chơi |
| label.status | Trạng thái |
| label.moves | Nước đi |
| label.captured | Quân bị ăn |
| side.white / side.black | Trắng / Đen |
| piece.K / Q / R / B / N / P | Vua / Hậu / Xe / Tượng / Mã / Tốt |
| mode.beginner / amateur / player / master / grandmaster | Tập sự / Nghiệp dư / Kỳ thủ / Đại sư / Chiến thần |

### 11.4 Strings: FSM states, alarms, messages

| ID | Vietnamese |
| --- | --- |
| state.IDLE | Chương trình đang tắt |
| state.INIT | Đang kết nối ESP32, camera, engine... |
| state.CALIBRATE / state.HOMING | Đang kiểm tra hiệu chỉnh và homing |
| state.SETUP\_CHECK | Kiểm tra thế cờ ban đầu |
| state.WAIT\_HUMAN | Đến lượt bạn |
| state.ENGINE\_THINK / state.PLAN | Máy đang nghĩ |
| state.ROBOT\_EXEC | Robot đang đi quân |
| state.VERIFY | Đang kiểm tra thế cờ |
| state.PAUSED | Đã tạm dừng |
| state.ERROR\_RECOVER | Có lỗi, cần xử lý |
| state.GAME\_OVER | Ván cờ kết thúc |
| alarm.ESTOP | Đã dừng khẩn cấp |
| alarm.LIMIT\_Z | Lỗi công tắc hành trình trục Z |
| alarm.Z\_TIMEOUT | Trục Z quá thời gian, kiểm tra cơ khí |
| alarm.SENSOR\_FAULT | Lỗi cảm biến góc, kiểm tra nam châm và dây |
| alarm.HB\_TIMEOUT | Mất kết nối với ESP32 |
| alarm.STEP\_LOSS | Động cơ mất bước, cần homing lại |
| msg.camera\_lost | Mất tín hiệu camera |
| msg.illegal\_move | Nước đi không hợp lệ hoặc quân đặt lệch |
| msg.engine\_restart | Engine không phản hồi, đang khởi động lại |
| msg.model\_load\_error | Không nạp được mô hình YOLO |
| msg.ask\_fallback | Chạy lớp dự phòng OpenCV? |
| msg.piece\_uncertain | Không chắc loại quân, vui lòng xác nhận |
| msg.choose\_promotion | Chọn quân phong cấp |
| msg.resume\_or\_new | Tiếp tục ván dở hay bắt đầu ván mới? |
| result.checkmate | Chiếu hết |
| result.draw | Hòa |
| result.stalemate | Hết nước đi (hòa) |
| result.white\_wins / result.black\_wins | Trắng thắng / Đen thắng |

## 12. Phased build plan (vibe coding)

For each phase: give the AI one prompt, have it write tests first or together with the code, run the tests, then commit. Always end a prompt with: Consult docs/SPEC.md, do not change hardware constants, and ask me when you meet a \[TO CONFIRM\] item.

### Phase 0: Project skeleton

- **Work:** create the directory layout (section 3), load hardware.yaml, Event Bus, logging, pytest.
- **Done when:** `pytest` runs; a test proves core/ does not import gui/; hardware.yaml holds every constant from section 4.
- **Prompt:** Create the project skeleton following sections 3 and 4 of the SPEC. Write a typed config loader, an Event Bus built on Qt signals, and a test proving core/ never imports gui/.

### Phase 1: Kinematics

- **Work:** five\_bar.py (FK, IK, Jacobian, μ), dimension.py (square centers, workspace), trajectory.py.
- **Done when:** the section 6.2 test vectors pass, FK(IK(P)) = P on all 64 squares, smallest μ is about 26°, and trajectories report an error before sending when a point is out of workspace.
- **Prompt:** Implement the five-bar kinematics from section 6 as pure NumPy with no Qt or serial dependency. Write tests using the four vectors in the table and sweep all 64 squares.

### Phase 2: GameState, engine, FSM (simulated run)

- **Work:** game\_state.py, engine.py (Stockfish, 5 modes), fsm.py, move\_planner.py (pick-and-place chains including capture, en passant, castling, promotion).
- **Done when:** a full game plays on the console (user types moves, engine replies, Move Planner prints command chains); special moves are tested; recovery of pending\_move after a mid-move shutdown is tested.
- **Prompt:** Build GameState, engine and FSM per sections 7 and 9. Do not connect camera or robot yet; use stubs. Write a test for every transition in the state table and for the pause rules.

### Phase 3: Protocol, Robot Commander, ESP32 simulator

- **Work:** protocol.py + protocol\_commands.py, serial\_link.py, robot\_commander.py, esp32\_simulator.py (models joint state, Z, the queue, queue full, heartbeat).
- **Done when:** the whole pick-and-place chain runs through the simulator; ESTOP, LIMIT\_Z, Z\_TIMEOUT, SENSOR\_FAULT and a lost connection can be simulated; the FSM enters ERROR\_RECOVER correctly.
- **Prompt:** Implement the protocol in section 5 and the Robot Commander. Write esp32\_simulator.py speaking the exact protocol over a virtual port pair. Add end-to-end tests: FSM to Move Planner to Commander to simulator.

### Phase 4: Firmware and no-load bench test

- **Work:** firmware per section 10. Bring-up order on hardware: INFO/PING, then ENA (correct polarity), then each motor slowly, then Z limit switches by hand, then Z with timeout, then servo, then ADS1115, then homing, then heartbeat, then ESTOP.
- **Done when:** each step is verifiable with real Serial commands; cutting a limit-switch wire stops Z; unplugging USB makes the ESP32 stop safely.
- **Prompt:** Write the PlatformIO ESP32 firmware per section 10, starting with INFO, PING and ENABLE/DISABLE only. Add one feature at a time and document how to verify it with Serial commands.

### Phase 5: Mechanical calibration

- **Work:** calibrate k and b of both AS5600 sensors (move each joint to two known angles, read r, solve k and b); board-to-robot mapping; measure trajectory straightness and vibration; measure the Z timeout; initial CorrectionMap.
- **Done when:** the gripper touches the square center at the four corners and all 64 squares with an acceptable error (a small fraction of the 31 mm square); the lost-step threshold is set from the measured standard deviation.

### Phase 6: Vision

- **Work:** camera\_manager.py, calibration.py, image capture and auto-labeling workflow, YOLO26 training, yolo\_detector.py, move\_detector.py, opencv\_fallback.py.
- **Done when:** Move Detector recognizes moves correctly on recorded videos of many games (including captures, castling and promotion) without the robot; the target recognition rate is reached.
- **Prompt:** Implement Move Detector per section 8.3 with a fake observed grid as input (no real YOLO). Test: lift and put back, occluded squares, two close hypotheses, promotion.

### Phase 7: GUI

- **Work:** gui/ per section 11, wired through the Event Bus, with all text from strings\_vi.py.
- **Done when:** each FSM state shows the right set of enabled buttons; Emergency Stop is always clickable; pytest-qt checks button enabling per FSM state; the two string tests from section 11.2 pass.

### Phase 8: Integration and safety testing

- **Work:** play many real games; deliberately inject faults: unplug USB, cover the camera, ESTOP mid-move, abrupt shutdown then reload, dropped piece, lighting change.
- **Done when:** the system recovers correctly from every case in the failure table; measure the move recognition rate and the rate of the robot landing on the right square.

## 13. Open items

The AI must not invent values for these; ask, then record them in hardware.yaml once known.

- NEMA 17 model, phase current, inductance (needed to set TB6600 DIP SW4–SW6). Note: the uploaded NEMA17.PDF is a closed-loop motor with an integrated driver, not a plain NEMA 17.
- Gear ratio i from motor to driving link, and whether each AS5600 is mounted on the joint axis or the motor axis.
- Rated voltage of the Z DC motor, the drivetrain type (trapezoidal lead screw), screw pitch, and a self-locking mechanism or counterweight that holds Z when power is lost; the actual L298 module (L298N?).
- 5 V supply for the SG90 (needs a 24 to 5 V buck converter or a separate supply).
- Coordinates of the capture zone, reserve zone and park position (outside the camera view, inside the workspace, μ ≥ 15°).
- Serial baud rate, Z timeout (measure at assembly), maximum acceleration and speed, lost-step threshold (start at 3°).
- Notation: in the original architecture document the symbol i means both lead-screw pitch and gear ratio; in this SPEC i is only the gear ratio. Fix the source document for consistency.

## 14. Safety checklist for the first hardware run

1. Run with no load: disconnect the links from the motors, or keep the arm in a safe position, and keep all pieces off the board.
2. Set the TB6600 current low first and raise it gradually to the motor's phase current; check the microstep DIP (16: SW1 OFF, SW2 OFF, SW3 ON).
3. Fuse the 24 V supply; check electrolytic capacitor polarity; keep the operator next to the power switch.
4. Check ENA: ENABLE and DISABLE must act in the right direction (DISABLE leaves the shaft free to turn).
5. Press both Z limit switches by hand; confirm TOP/BOTTOM are detected and `!alarm` fires; test a broken wire.
6. Run Z with a short timeout the first time, with a hand ready to cut power; do not run Z until you are sure it will not drop when power is lost.
7. Run MOVE at low speed and ramp up, comparing the AS5600 angle with the commanded angle.
8. Test the heartbeat: unplug USB while moving, confirm a safe stop with drivers still enabled.
9. Test ESTOP at several positions and speeds before placing real chess pieces.

## 15. Vibe coding tips

- Give one small job at a time and paste only the relevant SPEC sections instead of the whole document.
- Always require the AI to write and run tests before it says done. For firmware, require it to describe how to verify with Serial commands.
- Commit after each phase meets its definition of done; when the AI fixes a bug, paste real Serial logs or real test output instead of describing it in words.
- If the AI proposes changing the protocol, a constant or a safety rule, update the SPEC first and then the code, so the PC and firmware never drift apart.
- Do not let the AI add safety logic only on the PC side; every hardware protection (timeout, limit switch, heartbeat) must live in firmware.
- When moving from the simulator to real hardware, rerun the whole protocol test suite to confirm the simulator and firmware speak the same language.
