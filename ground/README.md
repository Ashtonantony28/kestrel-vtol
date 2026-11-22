# Kestrel ground radio bridge (Xbox controller → ESP32 → ELRS)

Replaces a $55–80 RC transmitter with parts you mostly already have: an Xbox Wireless Controller
(Bluetooth), an ESP32 dev board (~$6) and a second 2.4 GHz ELRS receiver (~$12) flashed as the
transmitter. The ESP32 reads the controller with [Bluepad32](https://github.com/ricardoquesada/bluepad32),
turns it into 16 RC channels and sends standard CRSF frames at 250 Hz to the ELRS module.

**Why this is safe enough when a plain Bluetooth link was not:** Bluetooth only spans the 1–2 m from
your hands to the ESP32. The 2.4 GHz link to the aircraft is real ELRS, with its own failsafe. If the
controller disconnects, or its samples stop arriving for 300 ms, the bridge **stops sending CRSF on
purpose**. The aircraft then sees an RC failsafe and runs `FS_LONG_ACTN` (RTL, which lands as a VTOL
at home because `Q_RTL_MODE 1`). That path is tested in SITL (`rc_link_lost_in_cruise`) and in
`tests/test_ground.cpp`.

## Verify

```bash
ground/tests/run.sh
```

This builds the protocol and stick-mapping core on a PC with `-Wall -Wextra -Wconversion -Werror`. It runs
about 2,300 checks plainly and again under AddressSanitizer and UBSan, then type-checks the sketch against
a stub of the Bluepad32 API.

What the checks cover:

- CRC-8/DVB-S2 against the published check value
- 11-bit channel packing and round trips
- the telemetry parser, including a corrupt frame and a bad length
- deadband, expo, monotonicity and symmetry
- the mode bands
- the arm-hold timing
- the controller-lost failsafe
- rumble alerts

What is **not** verified in the cloud: compiling against the real Bluepad32 board package, and
the ELRS module's baud rate. Both are bench item B9 in `build/BENCH.md`.

## Wiring

| ESP32 | ELRS module (flashed as TX) |
|---|---|
| GPIO17 (TX2) | RX pad |
| GPIO16 (RX2) | TX pad, through a 1 kΩ resistor |
| 5V / VIN | 5V |
| GND | GND |

Power everything from a USB power bank. The ESP32's onboard LED stays on while CRSF is being sent.

## Flashing

1. **ELRS module:** use the ExpressLRS Configurator to flash the receiver with the "RX as TX" target for
   its hardware family (ESP8285 or ESP32 based; check that your receiver model is supported *before* buying).
   - Use the same binding phrase as the aircraft receiver.
   - Set the packet rate to 250 Hz.
   - Set the module's serial baud to match `kCrsfBaud` in the sketch (420000 by default; some builds use 400000).
2. **ESP32:** in the Arduino IDE, add the Bluepad32 board package and select the "ESP32 Dev Module" board
   from the esp32_bluepad32 core. Open `kestrel_radio/kestrel_radio.ino` and upload it.
   - The `src/` folder sits inside the sketch folder, so the IDE picks it up automatically.
3. **Pair the controller:** hold the Xbox pair button. The first gamepad that connects is locked in, and new
   pairings are refused while it stays connected.

## Controls (Mode 2 layout)

| Input | Channel | Meaning |
|---|---|---|
| Right stick X | CH1 roll | right = roll right |
| Right stick Y | CH2 pitch | pull back = nose up (ArduPlane `norm_input > 0` = pitch up, `RC2_REVERSED 0`) |
| Left stick Y | CH3 throttle | up = more. The stick is spring-centred, so centred = hold altitude in Q modes |
| Left stick X | CH4 yaw | right = yaw right |
| Y / B / A | CH5 | QLOITER / QHOVER / CRUISE |
| D-pad up / X / D-pad down | CH5 | AUTO / RTL / QLAND |
| LB + RB held 1.5 s, throttle fully down | CH7 | toggle arm/disarm |
| Rumble | | ELRS uplink LQ < 50 % or aircraft battery < 10.6 V |

Because the throttle stick springs back to centre, fly **assisted modes only**: QLOITER, QHOVER,
CRUISE, AUTO and RTL. Never use QSTABILIZE or MANUAL with this controller.

Without an airspeed sensor, the throttle stick sets the pusher directly in CRUISE. Centred is 50 %,
which is below the 64 % cruise trim. In CRUISE, hold the stick about two-thirds up. The Q_ASSIST
lift motors catch a slow-speed sag, but don't rely on them. For hands-off fixed-wing flight, use AUTO.

## ArduPilot parameters for the receiver side

```
SERIALn_PROTOCOL 23      # RCIN on the UART wired to the aircraft ELRS receiver
RC_OPTIONS 8192          # bit 13: 420 kbaud for ELRS
RSSI_TYPE 3              # link quality from CRSF
FLTMODE_CH 5
FLTMODE1 19              # QLOITER
FLTMODE2 18              # QHOVER
FLTMODE3 7               # CRUISE
FLTMODE4 10              # AUTO
FLTMODE5 11              # RTL
FLTMODE6 20              # QLAND
RC7_OPTION 153           # ArmDisarm (with RC7 high = arm)
ARMING_RUDDER 0          # arm only through CH7
```

Bench check (B9): in Mission Planner's radio calibration screen, confirm each stick moves the right
bar in the direction shown above *before* fitting propellers. If pitch is backwards, set `RC2_REVERSED 1`.
Do not change the firmware for this.
