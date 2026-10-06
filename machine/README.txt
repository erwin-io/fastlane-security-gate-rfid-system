FASTLANE ONE-WAY ENTRANCE RFID GATE - MODULAR BUILD 1.7.2
ESP32-S3 + 1x ULN2003A (4-channel) + 2x TB6600 + 2x NEMA17 + RDM6300 +
WS2812B 8x8 + DS3231 + MicroSD + W5500 + 2x VL53L0X + SOLENOID RELAY + BUZZER

============================================================
VERSION 1.7.2 HOTFIX SUMMARY
============================================================

This build finalizes the two-driver motor architecture:

- TB6600 #1 drives LEFT / Motor #1 only.
- TB6600 #2 drives RIGHT / Motor #2 only.
- One existing 4-channel ULN2003A module handles all four logic signals.
- Both NEMA17 motors are wired identically at A+/A-/B+/B-.
- Motor #2 opposite physical rotation is generated in software using its own DIR.
- Motor #1 / LEFT has physical CLOSED and OPEN limit switches.
- Motor #2 / RIGHT remains open-loop until its own sensors/encoder are added.
- Each motor now has an independent logical position counter.
- If Motor #1 reaches an endpoint before Motor #2, Motor #1 can stop while Motor #2
  continues its remaining calculated steps.
- If the calculated travel ends before the LEFT endpoint switch activates, firmware
  performs a bounded slow Motor #1-only endpoint seek. Failure to find the switch
  enters ERROR instead of falsely reporting OPEN/LOCKED.
- CLOSED+OPEN limit active at the same time is treated as a limit conflict.
- Limit switches are optional during bench testing. They auto-arm only after an
  endpoint switch is actually observed active, so an unwired/unpressed switch does
  not block a valid RFID card.
- Strict CLOSED-limit-before-RFID mode remains available but defaults OFF.

Existing RFID, database, W5500, RTC, Web UI, synchronization, solenoid, buzzer,
WS2812B, latch-release assist and VL53L0X behavior remain.

============================================================
PROJECT STRUCTURE
============================================================

/boot.py
/config.py
/main.py
/webui.html
/sdcard.py
/README.txt
/WIRING_AND_SETUP.txt
/filestructure.txt
/components/__init__.py
/components/tb6600.py
/components/w5500.py
/components/ds3231.py
/components/sdcard.py
/components/rdm6300.py
/components/ws2812b.py
/components/rfid.py
/components/solenoid.py
/components/buzzer.py
/components/webserver.py
/components/vl53l0x.py

============================================================
DUAL TB6600 CONTROL ARCHITECTURE
============================================================

DRIVER #1 / LEFT
ESP32 GPIO1  -> ULN2003A IN1 -> OUT1 -> TB6600 #1 PUL-
ESP32 GPIO2  -> ULN2003A IN2 -> OUT2 -> TB6600 #1 DIR-
TB6600 #1 PUL+ -> 5V
TB6600 #1 DIR+ -> 5V

DRIVER #2 / RIGHT
ESP32 GPIO40 -> ULN2003A IN3 -> OUT3 -> TB6600 #2 PUL-
ESP32 GPIO41 -> ULN2003A IN4 -> OUT4 -> TB6600 #2 DIR-
TB6600 #2 PUL+ -> 5V
TB6600 #2 DIR+ -> 5V

ULN2003A module VDD -> 5V
ULN2003A module GND -> common logic GND

TB6600 #1 ENA+ / ENA- -> NOT CONNECTED
TB6600 #2 ENA+ / ENA- -> NOT CONNECTED

ULN2003A behavior:
ESP32 HIGH -> ULN output sinks LOW -> TB6600 '-' input active
ESP32 LOW  -> ULN output released

============================================================
MOTOR WIRING - BOTH MOTORS IDENTICAL
============================================================

MOTOR #1 / LEFT via TB6600 #1
A+ -> BLACK -> Pin 4
A- -> GREEN -> Pin 2
B+ -> RED   -> Pin 3
B- -> BLUE  -> Pin 1

MOTOR #2 / RIGHT via TB6600 #2
A+ -> BLACK -> Pin 4
A- -> GREEN -> Pin 2
B+ -> RED   -> Pin 3
B- -> BLUE  -> Pin 1

IMPORTANT:
Do NOT use the older Motor #2 wiring that swapped BLACK and GREEN.
That phase reversal was only required when both motors shared one TB6600.

============================================================
MOTOR DIRECTION
============================================================

Motor #1 and Motor #2 receive independent DIR signals.
Motor #2 direction is inverted in software:

OPEN:
LEFT  Motor #1 -> OPEN physical direction
RIGHT Motor #2 -> opposite physical shaft rotation, same gate OPEN timing

CLOSE:
LEFT  Motor #1 -> CLOSE physical direction
RIGHT Motor #2 -> opposite physical shaft rotation, same gate CLOSE timing

Default software coordinates:
CLOSED = 0 degrees
OPEN   = 80 degrees

Physical reference used by the current mechanism:
CLOSED: LEFT ~90 degrees, RIGHT ~90 degrees
OPEN:   LEFT ~170 degrees, RIGHT ~10 degrees

============================================================
TB6600 DIP SETTINGS
============================================================

Use the switch table printed on the exact driver if it differs.
Current tested project setting for BOTH drivers:

SW1 -> OFF
SW2 -> ON
SW3 -> OFF
Microstep -> 1/8
Pulses/revolution -> 1600

SW4 -> ON
SW5 -> OFF
SW6 -> OFF
Current setting -> approximately 2.0 A on the user's TB6600 units

Normal FASTLANE software speed profile:
START half-pulse delay = 1750 us
RUN half-pulse delay   = 630 us
ACCEL steps            = 80

============================================================
MASTER LIMIT SWITCHES - MOTOR #1 / LEFT
============================================================

GPIO39 -> CLOSED/LOCK limit switch -> GND
GPIO42 -> OPEN/UNLOCK limit switch -> GND

Firmware uses ESP32 internal pull-ups:
NOT PRESSED = HIGH
PRESSED     = LOW

Debounce = 5 ms

The switches are on the LEFT/master mechanism only.
They do not directly measure the RIGHT leaf.

Normal endpoint behavior:
1. LEFT endpoint switch activates.
2. Motor #1 STEP pulses stop immediately after switch detection.
3. Motor #1 logical position is anchored to the physical endpoint.
4. Motor #2 continues its own remaining calculated pulse count.

If the normal calculated travel finishes before the expected LEFT limit activates:
- Motor #2 stays at its calculated target.
- Motor #1 performs a slow endpoint seek only.
- Maximum additional seek = 3 degrees.
- Seek half-pulse delay = 4000 us.
- If the switch still does not activate, gate state becomes ERROR.

If CLOSED and OPEN limits are both active simultaneously, the firmware treats it
as a wiring/mechanical conflict and blocks/aborts normal gate operation.

Limit-switch startup rule:
- GATE_REQUIRE_CLOSED_LIMIT_FOR_OPEN = False by default.
- GATE_LIMIT_AUTO_ARM = True.
- If neither endpoint is active at boot, normal RFID/open-loop travel still works.
- As soon as either LEFT endpoint switch is observed active, endpoint enforcement
  automatically arms and the switches are used for stop/anchor/seek behavior.
- Set GATE_REQUIRE_CLOSED_LIMIT_FOR_OPEN = True only after the CLOSED switch has
  been physically installed and verified.

For long limit-switch wires in a motor/relay enclosure, use twisted signal/GND
wiring and keep it away from motor phase and solenoid wiring. For additional noise
immunity, an external 4.7k-10k pull-up to 3.3V and about 100nF from GPIO to GND can
be used if needed.

============================================================
RFID GRANTED / NORMAL OPEN FLOW
============================================================

Valid RFID
-> green arrow / grant feedback
-> active-low solenoid relay unlocks immediately
-> wait motor_start_delay_ms (default 1000 ms)
-> slow CLOSE-direction latch-release preload on both barriers
   default 3 degrees at 6500 us half-pulse
-> keep preload / solenoid released for 350 ms
-> re-anchor logical CLOSED reference
-> start normal full-speed OPEN movement
-> LEFT physical OPEN switch can stop Motor #1 independently
-> RIGHT Motor #2 completes its own calculated OPEN travel
-> if LEFT switch was not reached, perform bounded LEFT-only endpoint seek
-> OPEN state starts only after endpoint handling completes
-> gate remains open for gate_unlock_sec (default 12 s)
-> VL53L0X safety must report clear before closing
-> normal CLOSE movement
-> LEFT CLOSED switch can stop Motor #1 independently
-> RIGHT Motor #2 completes its own calculated CLOSE travel
-> if LEFT CLOSED switch was not reached, perform bounded LEFT-only endpoint seek
-> solenoid returns/holds LOCKED/OFF

============================================================
OBSTRUCTION REOPEN
============================================================

If VL53L0X detects an obstruction while CLOSING:
- current motor motion stops
- solenoid unlocks
- configured reopen delay runs
- both drivers reopen

v1.7.2 retains separate Motor #1 and Motor #2 logical positions. Therefore if
one motor had already reached an endpoint before the obstruction, the next reopen
calculates a separate remaining pulse count for each driver.

The latch-release CLOSE preload is NOT repeated during obstruction reopening,
because the barrier is already away from the mechanical latch.

============================================================
SOLENOID - FIXED ACTIVE-LOW REQUIREMENT
============================================================

SOLENOID_RELAY_ACTIVE_LOW = True

GPIO15 HIGH = relay OFF = solenoid unpowered = LOCKED
GPIO15 LOW  = relay ON  = solenoid powered   = UNLOCKED

Do not reverse this setting for this machine.

============================================================
VL53L0X SAFETY / PRESENCE
============================================================

Shared I2C0 with DS3231:
GPIO8  -> SDA
GPIO9  -> SCL
GPIO47 -> VL53L0X #1 XSHUT
GPIO48 -> VL53L0X #2 XSHUT

At startup:
VL53L0X #1 -> address 0x30
VL53L0X #2 -> address 0x31
DS3231      -> address 0x68

Default background = 600 mm each.

Behavior:
- optional presence requirement before RFID tap
- closing waits for clear passage
- obstruction while closing causes stop/reopen
- UNCERTAIN/POSSIBLE sensor states are treated as unsafe to close

============================================================
IMPORTANT POSITION LIMITATION
============================================================

LEFT/Motor #1 now has real physical endpoint confirmation.
RIGHT/Motor #2 still has no encoder or independent OPEN/CLOSED switch.

The RIGHT position therefore remains based on commanded TB6600 pulses. If the
RIGHT motor mechanically stalls/skips steps, firmware cannot directly measure that
error yet. Adding RIGHT OPEN/CLOSED switches or an encoder is the next upgrade for
true two-sided endpoint confirmation.

============================================================
POWER / RELIABILITY NOTES
============================================================

Each NEMA17 now has its own TB6600 current regulator. This is the intended dual-
driver architecture; do not reconnect both motors to one TB6600 output.

Both TB6600 motor-power inputs may share the central motor PSU when the PSU is
properly sized. Keep motor/solenoid high-current returns separated from sensitive
ESP32 signal returns as much as practical and join grounds in a controlled/star
arrangement where required by the logic interface.

Changing a PSU from 5A to 20A alone does not increase stepper phase current; the
TB6600 current setting regulates each motor. Higher motor-driver supply voltage can
improve high-speed stepper torque if it is within the exact TB6600/motor/system
ratings, but verify hardware ratings before changing supply voltage.

============================================================
END
============================================================
