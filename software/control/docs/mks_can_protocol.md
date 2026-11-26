# The MKS SERVO42D CAN protocol, as this arm uses it

Every byte here was checked against the **MKS SERVO42D/57D CAN User Manual V1.0.9** (for firmware V1.0.9; MKS's PDF,
not kept in this repo); § numbers are the manual's. `src/robotic_arm/motor_driver.py` implements it and
`tests/test_motor_driver.py` checks the frames against the manual's own examples.

**Which firmware the arm's boards run is not known yet** (`cad/docs/open_issues.md`). Read it with `0x40` (the CLI's
Read menu, `command_sweep`) or on the board's About menu. Where the manual's version notes date a command, the
"since" column says so. Older firmware may not have a command, or may answer it differently. The code takes either
reply where the manual shows a difference (`0x3B`, `0x40`), leaves the newer byte out unless asked (`0x90`'s hm_mode,
`0x91`'s mode), and sends the plain emergency stop unless asked for V1.0.9's immediate one.

The club's reference this replaces was compiled for the Arctos arm from the V1.0.5 manual. Its Arctos joint table,
gear ratios, DH parameters, wiring and endstop sections do not apply to this arm and were left behind.

## Frames (Part 4)

- Standard 11-bit frames, at most 8 data bytes: **code, data, check byte**. Multi-byte values are big-endian; signed
  ones are two's complement.
- The check byte: **SUM8** = (CAN id + every byte before it) & 0xFF (the default), or a **fixed 0x6B** (the CanCRC
  menu, or `0x4C`).
- CAN id 0 is broadcast; group ids are set by `0x8D`. Boards do **not** reply to either (Part 5 note 1). Ids 1-10 can be
  set on the menu, any id up to 2047 by `0x8B`. The factory id is 1: two new boards on one bus answer as one.
- A reply carries its request's code, with two exceptions: `0x00` replies with the parameter's code, and `0x01`'s
  periodic reports look exactly like the read's own replies.
- With the default response mode (`0x8C`: respond and active), a move or homing replies twice: when it starts (1)
  and when it ends (2, or 3 when stopped at a limit).

## Setting up a board for this arm

Once per board, on its menu or by command:

| setting | value | command | note |
|---|---|---|---|
| Mode | **SR_vFOC** | `82 05` | bus control, up to 3000 RPM. The factory default is CR_vFOC, a pulse mode, in which the motion commands below are not valid (Parts 9-11) |
| CanRate | 500K | `8A 02` | the factory default; `config.CAN_BITRATE` |
| CanID | its joint's row in `config.JOINTS` (1 base_yaw … 6 wrist_roll) | `8B` + uint16 | only one new board on the bus at a time while setting it |
| Ma | the motor's rated current | `83` + uint16 mA | SERVO42D: 0-3000 mA, default 1600; the manual advises against the board's maximum |
| Protect | to decide on the arm | `88 00/01` | stall protection, default off; `3E` reads it, `3D` releases it |
| MPlyer | Enable | `89 01` | the default: internal 256 microstep interpolation |
| Heartbeat | to decide on the arm | `98` + uint32 ms | default 0 (off): the board stops when no command reaches it in time |
| Homing | hm_mode 0 (origin switch) | `90` | each joint's KY-003 home switch (`cad/robot/AGENTS.md` home angles); trigger level and direction to set on the arm |

En (the EN pin's level) does not matter here. In bus mode `F3` enables the motor, and the En menu applies to pulse
mode only (§3.6).

## Commands

Request = the bytes after the CAN id and before the check byte. "since" = the version note that adds or changes the
command; blank = in the first manual or not dated. ✓ = a `MotorDriver` method; the CLI sends the rest as raw
commands.

### Reads (§5.1, §5.4.3, §6.1, §9.2.1)

| code | request | reply data (after the code) | since | |
|---|---|---|---|---|
| `30` | `30` | carry int32 (turns), value uint16 (0-0x3FFF in the turn) | | ✓ `read_encoder_carry` |
| `31` | `31` | value int48: the cumulative encoder count, 0x4000 per motor turn - the `F4` / `F5` coordinate once `92` or homing zeroed it (see "The coordinate") | | ✓ `read_encoder` |
| `32` | `32` | speed int16, RPM; positive turning CCW | | ✓ `read_speed` |
| `33` | `33` | received pulses int32 (pulse modes) | | |
| `34` | `34` | bit0 IN_1, bit1 IN_2, bit2 OUT_1, bit3 OUT_2 (after `9E`: bit0 En, bit1 Dir) | V1.0.3 | ✓ `read_io` |
| `35` | `35` | raw cumulative encoder int48 | V1.0.6 | |
| `38` | `38` | supply mV uint16, motor mA uint16 | **V1.0.9** | ✓ `read_power` |
| `39` | `39` | angle error int32: commanded − actual, 51200 = 360° of the motor | | ✓ `read_angle_error` |
| `3A` | `3A` | 1 enabled, 0 not | | ✓ `read_enabled` |
| `3B` | `3B` | V1.0.9: two statuses, single-turn zero return, then homing (`91`): 0 going, 1 success, 2 failed | V1.0.7 added the second | ✓ `read_home_status` (all bytes, as sent) |
| `3E` | `3E` | 1 stalled, 0 not | | ✓ `read_stalled` |
| `40` | `40` | Byte2 high nibble calibrated (1/0), low nibble board type (2 = S42D_CAN, 4 = S57D_CAN); then the firmware version; last the bootloader (0x10 RS485, 0x20 CAN) | | ✓ `read_version` |
| `F1` | `F1` | 0 query failed, 1 stopped, 2 speeding up, 3 slowing down, 4 full speed, 5 homing, 6 calibrating | | ✓ `read_status` |
| `00` | `00` code | the setting's value, in its write format; `FF FF` if it cannot be read back | V1.0.6 / V1.0.8 | ✓ `read_param` |
| `01` | `01` code period(uint16 ms) | `code` status (1 set); then a report every period, 0 stops it | **V1.0.9** | ✓ `set_auto_report` |

### Settings (§5.2-§5.5, §7.3)

Each replies `code status` (1 success, 0 failed) unless noted.

| code | request | meaning | since | |
|---|---|---|---|---|
| `80` | `80 00` | calibrate the encoder, **unloaded**; replies 0 calibrating, then 1 success / 2 failed | | ✓ `calibrate` |
| `82` | `82` mode | work mode 0-5: CR_OPEN, CR_CLOSE, CR_vFOC, SR_OPEN, SR_CLOSE, SR_vFOC | | |
| `83` | `83` mA(uint16) [`00`] | working current; with the trailing `00` it is not saved (reply 2) | V1.0.7 (`00`) | |
| `84` | `84` microsteps | subdivision (default 16) | | |
| `85` | `85` level | En pin: 0 low, 1 high, 2 hold (pulse mode only) | | |
| `88` | `88` 0/1 | stall (overcurrent) protection | | |
| `89` | `89` 0/1 | MPlyer interpolation | | |
| `8A` | `8A` 0-3 | CAN bitrate 125K / 250K / 500K / 1M | | |
| `8B` | `8B` id(uint16) | CAN id | | |
| `8C` | `8C` respond active | response mode (above) | V1.0.4 (active) | |
| `8D` | `8D` id(uint16) | group id | V1.0.1 | |
| `9D` | `9D` enable time(uint16) errors(uint16) | position-error protection; time × ~15 ms, 28000 errors = 360° | V1.0.6 | |
| `9E` | `9E` 0/1 | limit port remap: left → En, right → Dir | V1.0.4 | |
| `4C` | `4C` 0/1 | check byte: 0 SUM8, 1 fixed 0x6B. The manual does not say which the reply uses; the driver takes either | **V1.0.9** | ✓ `set_checksum_method` |
| `98` | `98` ms(uint32) | heartbeat protection, 0 = off | changed in V1.0.9 | ✓ `set_heartbeat` |
| `3D` | `3D` | release a stall | | ✓ `release_stall` |
| `3F` | `3F` | restore factory settings (no recalibration since V1.0.8); the board restarts | | |
| `41` | `41` | restart, settings kept | V1.0.5 | |

### Homing (Part 7)

| code | request | meaning | since | |
|---|---|---|---|---|
| `90` | `90` trig dir speed(uint16) endLimit [hm_mode] | homing setup: switch level when closed (0 low, 1 high), direction (0 CW, 1 CCW), RPM, limits on/off, hm_mode (0 origin switch, 1 mechanical limit, 2 single turn) | hm_mode: not dated (the menu's Hm_Mode came in V1.0.5) | ✓ `set_home_params` (hm_mode only when given) |
| `94` | `94` offset(int32) current(uint16 mA) | the mechanical-limit homing's back-off and current | V1.0.5 | |
| `9A` | `9A` mode set speed dir | single-turn zero return | V1.0.3 | |
| `91` | `91` [goZeroMode] | home: no byte or 0 = return to zero (search the switch), 1 = coordinate zeroing (go to the zero found before). Replies 0 failed, 1 started, 2 complete | goZeroMode: V1.0.9 | ✓ `go_home` |
| `92` | `92` | the current position becomes the zero, without moving | V1.0.1 | ✓ `set_zero` |

Homing searches in the set direction, stops on the switch's rising edge and marks that position as the coordinate
zero (§7.1). A board that starts on the switch first backs off it.

### Motion (Parts 9-11; bus modes only)

| code | request | replies | |
|---|---|---|---|
| `F3` | `F3` 0/1 | 1 done. **0 frees the shaft: a loaded joint falls** | ✓ `enable` |
| `F4` | `F4` speed(uint16) acc offset(int24) | 0 failed, 1 started, 2 complete, 3 stopped at a limit | ✓ `move_relative_axis` |
| `F5` | `F5` speed(uint16) acc position(int24) | as `F4`. A new `F5` while moving updates the target and speed (V1.0.5) | ✓ `move_to_axis` |
| `F6` | `F6` dir·speed(12 bit) acc [runtime(uint24, 10 ms)] | 1 running, 2 complete; speed 0 stops (acc 0 at once, acc > 0 ramping down) | ✓ `set_speed`, `stop` |
| `F7` | `F7` [`01`] | 1 stopped. Plain `F7` "may fail to stop the unit if the current is too low"; `F7 01` stops "regardless of current magnitude" | ✓ `emergency_stop` (`01` with `priority=True`) |
| `FD` / `FE` | pulse-count moves | (not used) | |
| `4A` / `4B` | synchronized start (V1.0.8) | `4B` is sent to id 0 and gets no reply | |

`F6`'s second byte is dir in bit 7 (0 = CCW, 1 = CW) and speed bits 11-8 in bits 3-0; the third byte is speed bits
7-0.

### Speed, acceleration, coordinates (§9.1, §11)

- **speed** is 0-3000 RPM, calibrated at 16/32/64 microsteps; other subdivisions scale it. A work mode's own maximum
  caps it: OPEN 400, CLOSE 1500, vFOC 3000.
- **acc** is 0-255: the speed changes by 1 RPM every (256 − acc) × 50 µs. acc 0 means no ramp at all, and acc 0 or
  `F7` above 1000 RPM is not recommended.
- **The coordinate** is `31`'s value, 0x4000 per motor turn, int24 in `F4` / `F5` (±8,388,607 = ±512 motor turns).
  Its zero is set by homing (`91`) or `92`; before either, the power-on position is the zero (§11.4.3).
  **Measured (base_yaw, V1.0.9, 2026-11-26):** before `92` or homing, `31` does not read that coordinate: after
  power-on it read the encoder's own angle in the turn (`30`: carry 0, value 1551) while `F5` counted from the
  power-on spot, so `F5` to 0 stopped where `31` read 1551 and `F5` to 273 where it read 1817 (inside the arrival
  window). After `92`, `30` and `31` read 0 and `F5` to 546 landed at 539. The dashboard sends `92` when a board
  comes on.
- **Direction:** the manual contradicts itself on which way the encoder counts (§5.1.1's example: +0x4000 after a
  CCW turn; §5.1.2: "after one turn clockwise, += 0x4000" but its example says CCW; §5.1.5: CW +). So each joint's
  `sign` in `config.JOINTS` is measured on the arm, not taken from the manual.

## Screen errors (§14.2)

| screen | cause | fix |
|---|---|---|
| Not Cal | not calibrated | calibrate (menu Cal or `80`), unloaded |
| Reverse Lookup Error! | bad calibration data | check the motor's wire order |
| Magnet Loss! / Strong! / Weak! | magnet missing, too close, too far | the magnet behind the motor shaft |
| Encoder Error! | encoder fault | contact MKS |
| Offset Current Error! | reference voltage fault | check the wire order |
| Phase Line Error! | motor wiring order, or too weak a supply (24 V/1 A, 12 V/2 A) | check them; a long press on Next clears it |
| Wrong Protect! | stall protection tripped | Enter, `3D`, or free the shaft (`F3 00`) |
| Coming Back to Origin.. | homing at power-on | if it persists, restore defaults (Next held at power-on) |
| Low Voltage Error! / Over Voltage error! | supply outside 12-24 V | fix the supply |

## Safety notes (§14.1 and the commands above)

- Supply 12-24 V. Do not plug or unplug the power or signal cables with the power on.
- Calibrate unloaded, best before the motor goes into the arm. Recalibrate after the board first goes on a motor or
  the motor's wiring changes.
- No immediate stop (`F7`, acc 0) above 1000 RPM. Use a ramp.
- `F3 00` and the end of `command_sweep` free the shaft: support the arm first.
- The heartbeat (`98`) stops a board that hears nothing for its period: a guard against a crashed host.

## Errata in manual V1.0.9

Found while checking every example frame and table against the bytes. The code follows the bytes that check out.

- §11.3.1: `01 F4 02 58 02 FF C0 00 09`: the bytes sum to 0x310, so the check byte is `10`. Its +0x4000 twin and
  §11.4.1's `F5` version check out.
- §11.3.3 step 4: `01 F4 00 00 02 00 00 00 F6`: the check byte is `F7`.
- §11.4.3 step 5, §11.4.4 step 6: `01 F5 00 00 02 00 00 00 00 F8` has one `00` too many (nine bytes): the frame is
  `F5 00 00 02 00 00 00 F8`.
- §5.1.6 `39`: the DLC is printed 4, but its layout (code, int32, check) is 6.
- §5.1.13 `38`: the DLC is printed 5; the layout and the manual's own capture are 6.
- §5.1.11 `40`: the reply table says DLC 5, lists a Byte6, and puts "firmVer[3]" in "Bytes 3-4". It is ambiguous, so
  `decode_version` takes every byte between Byte2 and the last data byte as the firmware version (two or three).
- §9.2.3 `F7 01`: the DLC is printed 2, with a Byte3 after it.
- §5.2.13 `4C` is captioned "Same as the CanRate option"; it is the CanCRC option (§3.14).
- §5.2.14 `8F`: both values read "Key lock".
