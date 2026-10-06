# ============================================================
# FASTLANE ONE-WAY ENTRANCE RFID TURNSTILE / SPEED GATE
# STATIC FIRMWARE CONFIGURATION
# ============================================================
#
# This file contains hardware pins, filesystem paths, database
# tuning values, RTC defaults, LED matrix definitions, and the
# DEFAULT runtime configuration.
#
# IMPORTANT:
# - Hardware/pin changes are made here.
# - DEFAULT_CONFIG values are only defaults.
# - The web configuration portal stores runtime overrides in
#   /config.json on the ESP32 internal flash.
# - Existing /config.json values override DEFAULT_CONFIG on boot.
# - Delete /config.json if you intentionally want to return to
#   the defaults declared in this file.
# ============================================================

# ============================================================
# PROJECT
# ============================================================

PROJECT_NAME = "FASTLANE RFID ACCESS CONTROL"
DEVICE_ROLE = "ONE-WAY ENTRANCE"
FIRMWARE_VERSION = "2.5.2"

# ============================================================
# GPIO - RDM6300 125KHZ RFID
# ============================================================

RFID_UART_ID = 1
RFID_RX_PIN = 16
RFID_TX_PIN = None     # RDM6300 is RX-only; GPIO47 is used by VL53L0X XSHUT #1
RFID_BAUD = 9600

# ============================================================
# GPIO - WS2812B 8x8 MATRIX
# ============================================================

LED_PIN = 18
NUM_LEDS = 64
MATRIX_WIDTH = 8
MATRIX_HEIGHT = 8

# ============================================================
# GPIO - DS3231 RTC
# ============================================================

RTC_SDA_PIN = 8
RTC_SCL_PIN = 9
DS3231_ADDRESS = 0x68

# v2.0.0: shared I2C0 (DS3231 + 2x VL53L0X) runs at 400 kHz fast mode so each
# ToF poll costs ~4x less CPU/bus time. Both devices support 400 kHz.
# Automatic fallback to I2C_FALLBACK_FREQUENCY happens when:
#   - VL53L0X hardware init fails at 400 kHz (long wires / weak pull-ups), or
#   - repeated runtime I2C errors are seen while the gate is stationary.
# Set I2C_FREQUENCY = 100000 to force the old conservative speed.
I2C_FREQUENCY = 400000
I2C_FALLBACK_FREQUENCY = 100000
I2C_RUNTIME_ERROR_DOWNGRADE_COUNT = 12   # errors within 5 s -> drop to fallback

# v2.0.5 FREEZE FIX: use the bit-banged SoftI2C driver. The ESP-IDF hardware
# I2C driver can block forever when an EMI spike (solenoid / relay switching)
# leaves SDA stuck mid-transfer - that froze the gate at OPEN. SoftI2C is pure
# software with a hard per-transfer timeout, so it always returns.
I2C_USE_SOFTWARE = True
I2C_SOFT_TIMEOUT_US = 50000

# v2.0.5: no ToF I2C traffic for this long after every solenoid switch (the
# 12 V coil's switching spike reset the VL53L0X sensors in the logs).
SOLENOID_I2C_QUIET_MS = 250

# v2.0.5 optional hardware watchdog: reboots the ESP32 (solenoid fails safe
# LOCKED in boot.py, gate re-homes) if the main loop ever stops for this long.
# Keep False while developing with Thonny: stopping the program in Thonny
# would trigger a reset after the timeout. Set True for production.
HARDWARE_WATCHDOG_ENABLED = False  # v2.5.2: OFF again - it survives a soft reboot/Thonny stop and aborted the board
HARDWARE_WATCHDOG_TIMEOUT_MS = 8000

# ============================================================
# GPIO - DUAL VL53L0X SAFETY / PRESENCE
# ============================================================
#
# The VL53L0X sensors SHARE I2C0 with the DS3231:
#   GPIO8 -> SDA
#   GPIO9 -> SCL
#
# XSHUT is required because both sensors power up at address 0x29.
# Sensor #1 is moved to 0x30 and Sensor #2 to 0x31 during boot.
#
# GPIO47 was previously only a dummy/unconnected RFID UART TX.
# RDM6300 is receive-only now, so GPIO47 is safely reassigned to XSHUT #1.
# GPIO48 is used for XSHUT #2.
# ============================================================

TOF_I2C_ID = 0
TOF_SDA_PIN = RTC_SDA_PIN
TOF_SCL_PIN = RTC_SCL_PIN
TOF_XSHUT1_PIN = 47
TOF_XSHUT2_PIN = 48
TOF_DEFAULT_ADDRESS = 0x29
TOF_SENSOR1_ADDRESS = 0x30
TOF_SENSOR2_ADDRESS = 0x31
TOF_SENSOR1_ANGLE = 90
TOF_SENSOR2_ANGLE = 90

# Boot-only VL53L0X internal wait limit (SPAD/ref-calibration in the "full"
# profile). Runtime never waits on the sensor at all.
TOF_IO_TIMEOUT_MS = 60

# Calibration file written ON THE ESP32 by tools/presence_calibrate.py.
# Option 1 stores offsets; option 2 stores the empty-lane background.
# If background_calibrated=true in this file, its background is used directly.
TOF_CALIBRATION_FILE = "/presence_range_calibration.json"

# One-time runtime background capture is saved here and reused on every boot.
# Delete this file (or POST /api/tof/background/capture) to capture again.
TOF_BACKGROUND_FILE = "/tof_background.json"

# ============================================================
# GPIO - MICROSD / SOFTSPI
# ============================================================

SD_CS_PIN = 10
SD_MOSI_PIN = 11
SD_SCK_PIN = 12
SD_MISO_PIN = 13
SD_BAUDRATE = 1000000
SD_INIT_BAUDRATE = 100000

# ============================================================
# GPIO - TWO TB6600 STEPPER DRIVERS / ONE 4-CHANNEL ULN2003A
# ============================================================
#
# FASTLANE v1.7.2 uses ONE TB6600 PER NEMA17.
# One existing 4-channel ULN2003A module is enough for all four logic signals:
#
#   DRIVER #1 / LEFT BARRIER
#   ESP32 GPIO1  -> ULN IN1 -> OUT1 -> TB6600 #1 PUL-
#   ESP32 GPIO2  -> ULN IN2 -> OUT2 -> TB6600 #1 DIR-
#
#   DRIVER #2 / RIGHT BARRIER
#   ESP32 GPIO40 -> ULN IN3 -> OUT3 -> TB6600 #2 PUL-
#   ESP32 GPIO41 -> ULN IN4 -> OUT4 -> TB6600 #2 DIR-
#
#   BOTH DRIVERS:
#   PUL+ -> 5V
#   DIR+ -> 5V
#
#   ULN2003A VDD/module supply -> 5V (for the user's module)
#   ULN2003A GND -> common GND
#
# IMPORTANT MOTOR WIRING:
# Both motors are now wired IDENTICALLY because each has its own DIR signal:
#   A+ -> BLACK / Pin 4
#   A- -> GREEN / Pin 2
#   B+ -> RED   / Pin 3
#   B- -> BLUE  / Pin 1
# Do NOT reverse Motor #2's BLACK/GREEN pair anymore.
#
# Motor #2 is made physically opposite in SOFTWARE by reversing TB6600 #2 DIR.
# ============================================================

TB6600_1_STEP_PIN = 1
TB6600_1_DIR_PIN = 2
TB6600_2_STEP_PIN = 40
TB6600_2_DIR_PIN = 41

# Compatibility aliases used by older status/debug code.
TB6600_STEP_PIN = TB6600_1_STEP_PIN
TB6600_DIR_PIN = TB6600_1_DIR_PIN

# True means Motor #2 receives the opposite physical direction from Motor #1
# while receiving the same step timing/count. Keep True for the dual swing gate.
TB6600_MOTOR2_OPPOSITE = True

# FASTLANE v1.7.5 FAST/SMOOTH motor profile.
#
# IMPORTANT:
# The pulse-rate values below are intentionally kept close to the proven
# loaded-gate profile. The big v1.7.5 speed/smoothness improvement comes from
# removing long gaps BETWEEN pulses, not from forcing the smaller 40 mm NEMA17
# to run at an excessively high step frequency.
TB6600_FULL_STEPS_PER_REV = 200
TB6600_MICROSTEP = 8
TB6600_STEPS_PER_REV = TB6600_FULL_STEPS_PER_REV * TB6600_MICROSTEP
TB6600_STEPS_PER_DEGREE = TB6600_STEPS_PER_REV / 360.0
TB6600_START_DELAY_US = 1700
TB6600_RUN_DELAY_US = 900
TB6600_ACCEL_STEPS = 80
# v2.1.7 normal OPEN start: S-curve (smootherstep in speed) over 120 steps.
# Same time to full speed as the old 80-step linear ramp, but without the
# torque spike near top speed that made the weaker arm slip back.
TB6600_OPEN_ACCEL_STEPS = 120
TB6600_OPEN_S_CURVE = True

# v1.7.5 pulse-burst scheduler.
#
# Older firmware emitted only ONE complete STEP pulse each time main.py called
# stepper.update(). ToF/RFID/LED/Web/RTC work between those calls inserted
# variable millisecond gaps, producing the visible "lag animation" movement.
#
# The driver now emits a short, tightly-timed burst per service call and returns
# to main.py frequently enough to retain VL53L0X/limit-switch safety.
#
# At the 900 us run half-pulse, an 8000 us budget fits about 4 complete pulses.
# At the slower 1700 us start, it fits about 2. This keeps acceleration smooth
# while preventing long blocking periods.
TB6600_MAX_PULSES_PER_UPDATE = 6
TB6600_MAX_BURST_US = 8000

# v1.7.13 HARDWARE RMT DUAL-PULSE ENGINE.
#
# The old scheduler generated STEP edges with Python Pin.value() + sleep_us().
# That works for a single driver, but two drivers plus ToF/limit/UI servicing can
# introduce uneven gaps between short software pulse bursts. ESP32-S3 RMT moves
# the actual STEP waveform into hardware so both TB6600 pulse trains continue
# while MicroPython services safety logic.
#
# If RMT cannot initialize, components/tb6600.py automatically falls back to the
# previous bounded bit-bang scheduler and prints a clear warning at boot.
TB6600_USE_HARDWARE_RMT = True
TB6600_RMT_CHANNEL_1 = 0
TB6600_RMT_CHANNEL_2 = 1
TB6600_RMT_RESOLUTION_HZ = 1000000       # 1 RMT tick = 1 microsecond
# v2.1.3: deeper hardware pulse buffer. 12 pulses = 21.6 ms at full speed,
# shorter than a measured MicroPython GC (21-24 ms) or one 600-char serial
# log block (48 ms), so the motor got pulse gaps up to 43 ms (stutter/stall).
# Endpoint stops still cancel the channel immediately.
TB6600_RMT_CHUNK_PULSES = 28             # max pulses queued per driver/chunk
TB6600_RMT_CHUNK_MAX_US = 50000           # <= ~50 ms queued motion before refill

# v1.7.15 RMT liveness guard. If a submitted hardware chunk remains reported busy
# for this long, firmware abandons RMT for the rest of the boot and continues the
# same move with the bounded bit-bang fallback. This prevents a stale RMT channel
# from freezing retry motion or the cooperative gate state machine.
TB6600_RMT_STALL_TIMEOUT_MS = 250

# v1.7.6 OPEN soft-landing profile.
#
# Keep the proven v1.7.5 fast section untouched, then progressively reduce
# speed as the gate approaches OPEN. The deceleration curve is smooth so
# the barrier does not hit the OPEN end at high speed and bounce backward.
TB6600_OPEN_SOFT_LAND_ENABLED = True
TB6600_OPEN_DECEL_START_PERCENT = 50
TB6600_OPEN_FINAL_DELAY_US = 3200

# Small final OPEN seating/preload movement.
#
# After normal OPEN travel (and any LEFT endpoint seek), apply a very small
# slow OPEN-direction push to seat the mechanism firmly at OPEN.
# Each motor is independently held if its own OPEN limit is already active.
# With both OPEN limits confirmed (the normal v1.7.16 case), this becomes only
# the configured settle/hold period and no motor is driven into its hard stop.
GATE_OPEN_SEAT_ENABLED = False  # per-side timed push now runs before OPEN
GATE_OPEN_PUSH_MS = 150         # v2.1.4: was 500 ms (~16 deg) -> switches bounced, arms desynced
GATE_OPEN_PUSH_DELAY_US = 3500
# v2.1.1: same seating push on first CLOSE-switch arrival. Live log: Motor #1
# CLOSE switch went ACTIVE, then RELEASED right after the motor stopped, so the
# next tap was refused as CLOSE_LIMIT_NOT_CONFIRMED and started a recovery.
# The push drives the arm slowly past the switch trip point. 0 = disabled.
GATE_CLOSE_PUSH_MS = 60         # v2.1.5: 300 ms (~10 deg) smashed; 0 (v2.1.4) left Motor #2 parked ON the
                                # trip point -> released at idle -> RFID_IDLE_CLOSE_RECOVERY. 60 ms ~2 deg slow overtravel.
GATE_CLOSE_PUSH_DELAY_US = 3500
# v2.1.1 auto-close after BOTH OPEN limits are confirmed:
#   - no presence for GATE_UNUSED_OPEN_CLOSE_MS (2 s)  -> close
#   - hard cap GATE_OPEN_MAX_HOLD_MS (5 s, v2.1.9) from OPEN -> close even if the
#     lane is NOT clear (person standing in the lane). 0 = cap disabled.
GATE_UNUSED_OPEN_CLOSE_MS = 2000
# v2.1.10: exit sensor S2 also blocks RFID taps while the gate is idle LOCKED
# (person standing in the closed lane). False = old rule (S2 only when OPEN).
TOF_EXIT_SENSOR_WHEN_LOCKED = True
GATE_OPEN_MAX_HOLD_MS = 8000      # v2.1.12: lane NOT clear -> close at 8 s max (was 5 s)
# v2.1.12: lane CLEAR -> always close after this, also when S2 never saw the
# exit (overrides tof.clear_close_delay_ms / unconfirmed_exit_close_delay_ms
# from config.json). 0 = use the config.json values.
GATE_CLEAR_CLOSE_MS = 2000
MANUAL_OPEN_BUTTON_PIN = 19
MANUAL_OPEN_BUTTON_HOLD_MS = 1500
MANUAL_OPEN_BUTTON_DEBOUNCE_MS = 50
# v2.3.1 staff open rules (decided when the button is RELEASED):
#   hold 2 s, release                  -> OPEN
#   hold 1.0-1.4 s, release            -> nothing (too short)
#   hold 1.5 s, keep holding to 4 s    -> OPEN on release
#   hold to 5 s                        -> configuration mode, never an open
# A short chirp sounds when the hold reaches 1.5 s ("release now to open").
MANUAL_OPEN_ARMED_CHIRP_MS = 60      # 0 = no chirp

# ============================================================
# v2.3.0 MACHINE CONFIGURATION MODE (GPIO19 long press)
# ============================================================
# The Wi-Fi AP and the Machine Configuration Web are OFF at boot. Holding the
# GPIO19 button for CONFIG_MODE_HOLD_MS toggles configuration mode:
#   UNLOCK: AP + web server start (web also answers on W5500 Ethernet),
#           VL53L0X paused, RFID disabled, motors/TB6600 never commanded,
#           solenoid locked, LED = permanent red X, unlock melody (~3 s).
#   LOCK  : web server + AP stop, lock melody, gate re-seeks BOTH CLOSE
#           switches with the boot-homing retry (waits for NO_PRESENCE), RFID
#           re-arms after GATE LOCKED + 1 s clear.
# A hold shorter than CONFIG_MODE_HOLD_MS never changes the mode.
# Unlock is refused (3 short beeps) while the gate moves or is open for a
# passage. W5500 sync, SD access logs and the RTC keep running in both modes.
CONFIG_MODE_HOLD_MS = 5000
# (freq_hz, on_ms, off_ms) - pitch is used by a PASSIVE buzzer only; an
# ACTIVE buzzer plays the rhythm. Unlock = rising, ~3.0 s.
CONFIG_MODE_UNLOCK_MELODY = (
    (1047, 150, 50), (1319, 150, 50), (1568, 150, 50), (2093, 300, 100),
    (1568, 150, 50), (2093, 150, 50), (2637, 500, 150),
    (2093, 150, 50), (2637, 150, 50), (3136, 550, 0),
)
# Lock = falling, ~2.0 s.
CONFIG_MODE_LOCK_MELODY = (
    (3136, 300, 100), (2637, 300, 100), (2093, 300, 100),
    (1568, 300, 100), (1047, 400, 0),
)
CONFIG_MODE_REFUSED_BEEPS = (3, 80, 70)
# Background capture requested from the web while in configuration mode runs
# the VL53L0X only until the capture is saved or this timeout.
CONFIG_MODE_TOF_CAPTURE_TIMEOUT_MS = 20000
# v2.1.1 staff override: the button opens from ANY gate state (closing,
# seeking, retrying, open, error, boot homing) with NO lane-clear or
# limit-switch validation. A gate that is not latched closed reverses after
# this short pause (the full latch-release sequence is used only when the
# gate is LOCKED on both CLOSE switches).
MANUAL_OVERRIDE_REOPEN_DELAY_MS = 150
RFID_PENDING_TAP_MS = 1500
# v2.1.11 LED / tap rules (gate LOCKED, idle):
#   - LED holds RED while presence is detected AND for the 1 s clear re-arm
#     (tof.rfid_rearm_clear_ms); BLUE standby = taps allowed.
#   - Taps during that red second are refused (red X + warning), not queued.
#     True = old v2.0.x behaviour (queue the tap and grant after re-arm).
RFID_ACCEPT_TAP_DURING_REARM = False
#   - Invalid / unknown card: deny beep + RED for RFID_DENY_LOCKOUT_MS, every
#     tap ignored during that time, then the next tap is allowed.
RFID_DENY_LOCKOUT_MS = 1000
GATE_OPEN_SEAT_DEGREES = 1.5
GATE_OPEN_SEAT_DELAY_US = 3500
GATE_OPEN_SEAT_SETTLE_MS = 600

# ESP32-side logic BEFORE ULN2003A inversion.
# GPIO HIGH -> ULN output sinks LOW -> corresponding TB6600 '-' input active.
# GPIO LOW  -> ULN output OFF/released.
TB6600_STEP_IDLE_LEVEL = 0
TB6600_STEP_ACTIVE_LEVEL = 1

# Tested ULN2003A/TB6600 direction levels for this machine.
# These are ESP32 GPIO2 levels BEFORE the ULN2003A inversion:
#   OPEN  -> GPIO2 LOW  -> ULN OUT2 released
#   CLOSE -> GPIO2 HIGH -> ULN OUT2 sinks LOW -> TB6600 DIR- active
#
# v1.7.18: OPEN/CLOSE direction levels were intentionally swapped because
# the physical gate was rotating toward CLOSE when commanded OPEN.
#
# Keep OPEN and CLOSE as DIFFERENT values. The motor component now receives
# an explicit OPEN/CLOSE command from main.py instead of relying only on the
# sign of the software angle difference.
TB6600_OPEN_DIR_LEVEL = 0
TB6600_CLOSE_DIR_LEVEL = 1

# Backward-compatible names used by TB6600DualMotor constructor.
TB6600_FORWARD_DIR_LEVEL = TB6600_OPEN_DIR_LEVEL
TB6600_REVERSE_DIR_LEVEL = TB6600_CLOSE_DIR_LEVEL
TB6600_DIR_SETUP_MS = 30

# ============================================================
# LATCH-RELEASE WIDE SHAKE PROFILE
# ============================================================
#
# v1.7.4 keeps the wide alternating shake but reduces it to two cycles.
#
# Sequence after the solenoid has already been energized:
#
#   CLOSE 5 deg
#   OPEN  5 deg
#   CLOSE 5 deg
#   OPEN  5 deg
#   CLOSE 5 deg
#   hold 350 ms
#   re-anchor logical position to CLOSED
#   start normal OPEN travel
#
# With SHAKE_CYCLES=2 this is one initial CLOSE preload followed by two
# complete OPEN/CLOSE shake cycles. The final leg is always CLOSE so the
# mechanism finishes pressed toward its CLOSED reference before normal OPEN.
#
# The solenoid remains continuously energized during the entire shake and hold.
# CLOSE limit enforcement is intentionally ignored only during this shake.
#
# IMPORTANT:
# - Normal OPEN/CLOSE speed remains controlled by TB6600_START/RUN_DELAY_US.
# - The shake uses its own slower delay for higher low-speed torque.
# - The temporary shake position is discarded before the normal OPEN move.
# ============================================================

LATCH_RELEASE_PROFILE_VERSION = 4

# Wide mechanical shake amplitude per leg.
LATCH_RELEASE_JOG_DEGREES_DEFAULT = 5.0

# Slow/high-torque shake speed. Lower value = faster.
LATCH_RELEASE_JOG_DELAY_US_DEFAULT = 2500

# Number of full OPEN/CLOSE oscillations after the initial CLOSE preload.
LATCH_RELEASE_SHAKE_CYCLES_DEFAULT = 1

# Small pause after each leg so DIR can settle mechanically and the latch can react.
LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT = 80

# Final CLOSE preload hold before the normal full OPEN begins.
LATCH_RELEASE_SETTLE_MS_DEFAULT = 350

# v2.1.6 latch-release mode.
#   "SLOW_OPEN" : ONE smooth slow leg in the OPEN direction only, then the
#                 normal OPEN continues from there (no CLOSE preload, no
#                 reversals, no hold). Arms never push back into the CLOSE
#                 stop / CLOSE switches before opening.
#   "SHAKE"     : legacy v1.7.4 CLOSE -> OPEN -> CLOSE shake + 350 ms hold.
LATCH_RELEASE_MODE = "SLOW_OPEN"
LATCH_SLOW_OPEN_DEGREES = 6.0      # slow lift-off distance before normal OPEN
LATCH_SLOW_OPEN_DELAY_US = 2500    # same slow/high-torque speed as the old shake

# ============================================================
# GPIO - DUAL-SIDE MECHANICAL CLOSE / OPEN LIMIT SWITCHES
# ============================================================
#
# v1.7.16 gives EACH NEMA17/TB6600 pair its own authoritative OPEN/CLOSE
# endpoint switches. Both sides start together and receive the same pulse
# profile, but either motor is stopped independently as soon as ITS OWN limit
# is confirmed. The other motor keeps moving until its own limit is reached.
#
# Wiring (active LOW with ESP32 internal pull-up):
#
#   MOTOR #1 / LEFT SIDE
#     GPIO45 -> CLOSE limit switch -> GND
#     GPIO38 -> OPEN  limit switch -> GND
#
#   MOTOR #2 / RIGHT SIDE
#     GPIO39 -> CLOSE limit switch -> GND
#     GPIO42 -> OPEN  limit switch -> GND
#
# NOT PRESSED = HIGH
# PRESSED     = LOW
#
# IMPORTANT:
# - A gate endpoint is considered fully OPEN/CLOSE only when BOTH matching
#   switches are confirmed.
# - Cross-side disagreement (for example M1=CLOSE while M2=OPEN) is treated as
#   mechanical desynchronization to recover, not an electrical limit conflict.
# - A conflict exists only if OPEN and CLOSE are active simultaneously on the
#   SAME motor side.
# ============================================================

GATE_MOTOR1_CLOSE_LIMIT_PIN = 45
GATE_MOTOR1_OPEN_LIMIT_PIN = 38
GATE_MOTOR2_CLOSE_LIMIT_PIN = 39
GATE_MOTOR2_OPEN_LIMIT_PIN = 42

# Backward-compatible aliases for older dashboard/status code.
GATE_CLOSE_LIMIT_PIN = GATE_MOTOR1_CLOSE_LIMIT_PIN
GATE_OPEN_LIMIT_PIN = GATE_MOTOR1_OPEN_LIMIT_PIN
GATE_CLOSE_LIMIT2_PIN = GATE_MOTOR2_CLOSE_LIMIT_PIN
GATE_OPEN_LIMIT2_PIN = GATE_MOTOR2_OPEN_LIMIT_PIN

GATE_LIMIT_ACTIVE_LOW = True

# Fast debounce used while a motor is actually moving.
GATE_LIMIT_DEBOUNCE_MS = 10

# Stationary endpoint glitch filter.
#
# When the gate is physically stationary at a known endpoint, a short electrical
# glitch must not make RFID think the gate has been forced open. During movement
# the normal 10 ms debounce still applies so endpoint stopping remains fast.
GATE_IDLE_LIMIT_GLITCH_FILTER_MS = 500
# v2.5.1: OPEN+CLOSE on one side must hold this long before it is a fault
LIMIT_CONFLICT_CONFIRM_MS = 1000
# v2.5.1: gate ERROR -> automatic re-home to CLOSE after this delay
GATE_ERROR_REHOME_MS = 3000
# v2.5.2: idle LOCKED gate with a CLOSE limit released this long -> re-home
IDLE_CLOSE_RESEEK_MS = 1500

GATE_LIMIT_SWITCHES_ENABLED = True

# Fixed authoritative endpoint policy.
# The machine homes to CLOSE at boot and normal access is not enabled until
# the CLOSE switch has been found.
GATE_LIMIT_SWITCHES_REQUIRED = True

# v1.7.16 DUAL-SIDE BIDIRECTIONAL LIMIT-FIND / RETRY POLICY.
#
# These are STATIC firmware constants only. They are intentionally NOT included
# in DEFAULT_CONFIG, are NOT exposed to /api/config, and cannot be changed from
# the Web configuration page.
#
# Every active attempt to FIND an OPEN or CLOSE limit is allowed to drive each
# still-missing motor for at most 2 seconds. A side whose own limit is already
# active is held. If either required side is still missing, active motors stop,
# rest for 2 seconds, then retry the SAME direction. This repeats until BOTH
# physical endpoint switches for the requested direction are confirmed.
#
# The CLOSE recovery path is also armed when RFID detects a previously closed
# gate whose Motor #1 GPIO45 OR Motor #2 GPIO39 CLOSE reference is missing. Repeated RFID frames do
# not reset the already-running recovery timer.
GATE_LIMIT_FIND_MAX_MS = 10000
GATE_LIMIT_RETRY_INTERVAL_MS = 0

# BOOT homing: each motor whose CLOSE switch is not already active moves slowly
# toward CLOSE for at most 2 seconds per attempt. A side stops immediately on its
# own CLOSE switch. Boot does not declare SYSTEM READY until BOTH CLOSE switches
# are confirmed.
GATE_BOOT_HOME_DELAY_US = 4000
GATE_BOOT_HOME_MAX_DEGREES = 720.0
GATE_BOOT_HOME_MAX_MS = GATE_LIMIT_FIND_MAX_MS

# OPEN confirmation rule. Each motor is stopped independently by its own OPEN
# switch (M1 GPIO38, M2 GPIO42). If one or both switches are still missing after
# normal travel, only the missing side(s) continue OPEN slowly for up to 2 seconds
# per attempt. The gate does NOT enter OPEN hold until BOTH OPEN switches confirm.
# v2.0.10: OPEN seek / retry attempt = 3 s (2 s fast + 1 s ease-out).
GATE_OPEN_LIMIT_MAX_WAIT_MS = 3000   # = GATE_OPEN_RECOVERY_FAST_MS + _SLOW_MS below
GATE_OPEN_LIMIT_SEEK_MAX_DEGREES = 3600.0
GATE_OPEN_LIMIT_SEEK_DELAY_US = TB6600_OPEN_FINAL_DELAY_US   # fallback only

# v2.0.6: OPEN endpoint seek + retry run at the NORMAL OPEN speed profile on
# the RMT pulse engine (same as CLOSE recovery), not the old 3200 us crawl.
# Set either to False to restore the legacy slow fixed-speed bit-bang seek.
GATE_OPEN_RECOVERY_USE_NORMAL_SPEED = False
GATE_OPEN_RECOVERY_USE_RMT = True

# v2.0.10: OPEN seek/retry speed = same feel as the normal OPEN: FAST first,
# then EASE-OUT at the end. Time based, per attempt:
#   0 .. 2000 ms     : full normal OPEN speed (after a 10-step launch ramp)
#   2000 .. 3000 ms  : ease-out braking down to the soft crawl
# Each side still stops instantly on its own OPEN switch.
GATE_OPEN_RECOVERY_EASE_OUT = False
GATE_OPEN_RECOVERY_FAST_MS = 2000
GATE_OPEN_RECOVERY_SLOW_MS = 1000
GATE_OPEN_RECOVERY_EASE_FINAL_DELAY_US = TB6600_OPEN_FINAL_DELAY_US   # 3200 us
GATE_OPEN_RECOVERY_EASE_LAUNCH_STEPS = 10

# CLOSE confirmation after a normal gate cycle. Each motor is stopped independently
# by its own CLOSE switch (M1 GPIO45, M2 GPIO39). If one or both switches remain
# missing, only the missing side(s) continue CLOSE slowly for up to 2 seconds.
# On failure those active side(s) stop and background retry begins every 2 seconds.
GATE_CLOSE_LIMIT_SEEK_MAX_DEGREES = 3600.0
GATE_CLOSE_LIMIT_SEEK_DELAY_US = TB6600_RUN_DELAY_US  # fallback/reference only
GATE_CLOSE_RECOVERY_USE_NORMAL_SPEED = True
GATE_CLOSE_RECOVERY_USE_RMT = True
GATE_CLOSE_LIMIT_MAX_WAIT_MS = GATE_LIMIT_FIND_MAX_MS

# v2.1.2 closing speed (v2.1.1 full-speed close bounced off the CLOSE switch):
#   normal CLOSE      : full normal speed for the first
#                       TB6600_CLOSE_DECEL_START_PERCENT of the travel, then a
#                       smooth slowdown to TB6600_CLOSE_FINAL_DELAY_US, held
#                       at that slow speed into the CLOSE switches.
#   seek / retries    : SLOW fixed GATE_CLOSE_SLOW_RETRY_DELAY_US (only the
#                       side(s) still missing their CLOSE switch move).
# Windows unchanged: each attempt is 10 s; a first window that expires without
# both CLOSE switches (blocked) switches to the slow retries.
# A slow retry waits for a clear lane up to GATE_CLOSE_RETRY_CLEAR_WAIT_MS,
# then proceeds slowly anyway.
TB6600_CLOSE_SOFT_LAND_ENABLED = True
TB6600_CLOSE_DECEL_START_PERCENT = 50
TB6600_CLOSE_FINAL_DELAY_US = 3200           # ~35 deg/s at the CLOSE switch
GATE_CLOSE_SLOW_RETRY_DELAY_US = 3200        # ~35 deg/s (normal run 900 us ~125 deg/s)
GATE_CLOSE_RETRY_CLEAR_WAIT_MS = 3000

# ============================================================
# GPIO - 12V SOLENOID RELAY
# ============================================================

SOLENOID_RELAY_PIN = 15

# FAIL-SAFE RELAY POLARITY
# ------------------------
# IMPORTANT: this machine ALWAYS uses an ACTIVE-LOW relay module:
#   GPIO HIGH = relay de-energized = solenoid unpowered = LOCKED
#   GPIO LOW  = relay energized   = solenoid powered   = UNLOCKED
#
# This is intentionally a STATIC hardware setting. It is NOT editable from
# /config.json or the web UI, so a stale runtime configuration cannot change
# the relay polarity. Keep this True for this FASTLANE machine.
SOLENOID_RELAY_ACTIVE_LOW = True
SOLENOID_RELAY_LOCKED_LEVEL = 1 if SOLENOID_RELAY_ACTIVE_LOW else 0
SOLENOID_RELAY_UNLOCKED_LEVEL = 0 if SOLENOID_RELAY_ACTIVE_LOW else 1

# FIXED SOLENOID TIMING POLICY - v1.7.4
# -------------------------------------
# The solenoid MUST stay released during:
#   pre-motor delay -> latch shake -> complete NEMA OPEN travel -> endpoint seek.
# Only after the gate is fully OPEN does the 1-second lock delay start.
# This is static machine behavior and is intentionally not user-configurable.
SOLENOID_LOCK_AFTER_OPEN_DELAY_MS = 1000

# Backward-compatible alias used by older code/config references.
GATE_RELAY_PIN = SOLENOID_RELAY_PIN

# ============================================================
# GPIO - BUZZER CONTROL
# ============================================================
#
# GPIO17 is only a control signal. A 5V/12V buzzer must be driven
# through a transistor/MOSFET or a suitable buzzer module input.
# ============================================================

BUZZER_PIN = 17
BUZZER_ACTIVE_HIGH = True

# v2.1.9 buzzer volume (PWM on GPIO17 through the existing transistor/module).
#   BUZZER_TYPE = "ACTIVE"  : buzzer makes its own tone from DC (beeps on plain
#                             3.3-5 V). 100 % = plain ON (exact old behaviour);
#                             lower % chops the supply at BUZZER_ACTIVE_PWM_HZ
#                             -> quieter (can sound slightly rougher).
#   BUZZER_TYPE = "PASSIVE" : plain disc that needs a tone. Plays BUZZER_TONE_HZ;
#                             volume = duty (100 % = 50 % duty = loudest).
# Volume: 0-100 (0 = silent). Examples: 100 loud, 60 medium, 25 quiet.
BUZZER_TYPE = "ACTIVE"
BUZZER_VOLUME_PERCENT = 100
BUZZER_TONE_HZ = 2700          # PASSIVE only (2-4 kHz is loudest for most discs)
BUZZER_ACTIVE_PWM_HZ = 20000   # ACTIVE only, chopping frequency when volume < 100

# ============================================================
# SIMPLE BOOLEAN PRESENCE ACCESS FLOW - v1.9.0
# ============================================================
#
# The VL53L0X layer reports only:
#   PRESENCE
#   NO_PRESENCE
#
# Direction/crossing classification is removed.
#
# Machine behavior:
# - New RFID is accepted only at NO_PRESENCE + BOTH CLOSE limits.
# - PRESENCE blocks new RFID.
# - PRESENCE without an active grant starts the warning buzzer.
# - PRESENCE during a granted passage is normal/expected.
# - When an already-seen granted person changes from PRESENCE to
#   NO_PRESENCE, CLOSE is armed after 1.5 seconds.
# - Actual CLOSE still requires fresh NO_PRESENCE from both sensors.
# - The granted transaction remains active until BOTH CLOSE limits
#   physically confirm the completed gate cycle.
# ============================================================

PRESENCE_CLEAR_CLOSE_DELAY_MS = 1000  # fallback/default

# A fresh NO_PRESENCE result must remain stable for this long before a NEW
# RFID transaction is accepted. This prevents a just-closed gate from accepting
# a card on an old/transitioning clear sample while a person is still in lane.
RFID_CLEAR_STABLE_MS = 300

# Active-buzzer warning cadence.
TURNSTILE_WARN_BEEPS = 3
TURNSTILE_WARN_ON_MS = 80
TURNSTILE_WARN_OFF_MS = 70
TURNSTILE_WARN_COOLDOWN_MS = 1200

# ============================================================
# v2.0.0 EXPLICIT TASK SCHEDULER
# ============================================================
#
# Task              | Gate MOVING (open/close/seek/shake) | Gate STOPPED
# ------------------+-------------------------------------+------------------------
# TB6600 + limits   | ACTIVE - highest priority           | ACTIVE (idle)
# Solenoid/LED/buzz | ACTIVE (lightweight)                | ACTIVE
# VL53L0X           | PAUSED - zero I2C traffic           | ACTIVE continuous
# RFID RDM6300      | PAUSED - UART not parsed            | ACTIVE unless locked out:
#                   |                                     |  PRESENCE -> disabled;
#                   |                                     |  re-armed only when gate
#                   |                                     |  LOCKED + NO_PRESENCE
#                   |                                     |  continuously 1000 ms
# W5500 link/web    | ACTIVE - link monitor + web I/O,    | ACTIVE - full service
#                   | new web requests held until stop    |
# Sync / SD logs    | DEFERRED                            | gate idle only
# RTC               | deferred (NVS write)                | ACTIVE
#
# A request that is in flight on W5500 when motion starts keeps the motors
# serviced through its cooperative callback, and any downloaded sync page is
# held in RAM and written to SD only after motion ends.

# Presence application state machine (close timers, warnings) service period.
PRESENCE_FLOW_INTERVAL_MS = 5

# RFID re-arm dwell: gate LOCKED + fresh NO_PRESENCE continuously this long.
RFID_REARM_CLEAR_MS = 1000

# Stale UART bytes purged (bounded) every time the RFID task is re-enabled.
RFID_RESUME_DISCARD_BYTES = 256

# Optional: keep VL53L0X polling during CLOSE motion only so PRESENCE can
# stop and reopen the barrier (anti-pinch). Default False = strict "ToF off
# while any motor moves" policy. In continuous mode this guard costs only a
# 1-byte I2C status read per ~33 ms sample and does not affect RMT stepping.
TOF_CLOSE_MOTION_GUARD_DEFAULT = False

# Legacy names kept for compatibility with older tooling.
TOF_IDLE_SERVICE_INTERVAL_MS = PRESENCE_FLOW_INTERVAL_MS
TOF_MOVING_SERVICE_INTERVAL_MS = 10

# ============================================================
# GPIO - W5500 / DEDICATED HARDWARE SPI
# ============================================================

W5500_SCK_PIN = 4
W5500_MOSI_PIN = 5
W5500_MISO_PIN = 6
W5500_CS_PIN = 7
W5500_INT_PIN = 14
W5500_RST_PIN = 21
W5500_SPI_ID = 1
W5500_SPI_BAUDRATE = 8000000   # v2.4.0 field fix: 20 MHz corrupted traffic on the gate wiring

# v2.0.0: W5500 is never disabled. The ESP-IDF driver services the chip in
# its own task; Python only monitors the link (cheap) every interval below.
ETH_LINK_CHECK_MS = 1000

# ============================================================
# FILESYSTEM PATHS
# ============================================================

SD_MOUNT = "/sd"
LOG_DIRECTORY = "/sd/logs"

# v2.2.0: cards are identified by their 10-digit DECIMAL card number
# (RDM6300 "030081DFD8" -> "0008511448", components/card_id.py). The decimal
# database lives in new folders; the old hex-UID folders below are read once
# at boot to migrate, and are otherwise never touched (rollback-safe).
RFID_DB_A = "/sd/cards_A"
RFID_DB_B = "/sd/cards_B"
RFID_ACTIVE_FILE = "/sd/cards_active.txt"
# v2.1.4 SD guard: mirror of the active card list in the ESP32's internal
# flash. On 04 Oct 2026 the SD FAT was found wiped while running (all 4 cards
# then DENIED, log writes OSError(2)). If the SD database is empty at boot,
# or lost while running, the cards are restored from this mirror.
# v2.2.0: one decimal card ID per line.
RFID_FLASH_BACKUP_FILE = "/cards_backup.txt"
RFID_FLASH_BACKUP_MAX = 2000         # larger databases are not mirrored

# Pre-v2.2.0 hex-UID data - migration sources only (read, never written).
LEGACY_HEX_DB_A = "/sd/rfid_db_A"
LEGACY_HEX_DB_B = "/sd/rfid_db_B"
LEGACY_HEX_ACTIVE_FILE = "/sd/rfid_active.txt"
LEGACY_HEX_OLD_DB = "/sd/rfid_db"
LEGACY_HEX_TXT_DATABASE = "/sd/rfids.txt"
LEGACY_HEX_FLASH_BACKUP_FILE = "/rfid_backup.txt"
SYNC_STATE_FILE = "/sd/sync_state.json"

# v2.4.0 server integration profile. A saved /config.json whose
# server.profile_version is older gets the new server/sync defaults once
# (keeping the user's enable switches, interval, timeout, key and gate ID).
SERVER_PROFILE_VERSION = 250
# Boot sync: retried until the W5500 link is up, for at most this long.
BOOT_SYNC_RETRY_MS = 2000
BOOT_SYNC_GIVE_UP_MS = 120000
# Tap notifications (fire-and-forget POST /api/turnstile/tap).
TAP_NOTIFY_QUEUE_MAX = 8
TAP_NOTIFY_MAX_AGE_MS = 10000
TAP_NOTIFY_TIMEOUT_MS = 3000
# While the motors run the notifier is stepped at most this often.
TAP_NOTIFY_MOTION_STEP_MS = 10
# v2.4.1: a failed sync run is retried after this many seconds (or the
# normal interval, whichever is shorter).
SYNC_RETRY_AFTER_FAIL_SEC = 10

# Runtime settings saved by the local web configuration portal.
CONFIG_FILE = "/config.json"

# Static AP configuration page stored as a normal flash file.
# It is streamed to the browser in small chunks instead of being imported
# as a giant Python string, which greatly reduces permanent heap use.
WEBUI_FILE = "/webui.html"
WEBUI_CHUNK_SIZE = 1024

# Cooperative local web server tuning. These are static performance values;
# all existing AP/server/sync/tap/gate settings remain in DEFAULT_CONFIG.
WEB_RECV_CHUNK_SIZE = 1024
WEB_SEND_CHUNK_SIZE = 1024
WEB_CLIENT_TIMEOUT_MS = 2000

# v2.0.0: bind the Web UI/API to ALL interfaces (Wi-Fi AP + W5500 Ethernet),
# so the same page is reachable at http://<W5500 IP>/ as well.
WEB_BIND_ALL_INTERFACES = True

# While motors move, web socket I/O is serviced at this interval and newly
# completed requests are held (not routed) until the gate stops.
WEB_MOTION_SERVICE_INTERVAL_MS = 10
WEB_MAX_DISPATCH_DEFER_MS = 4000

# Fast cooperative application loop. Motor pulses are generated by a
# non-blocking state machine, so a short loop sleep improves responsiveness.
MAIN_LOOP_SLEEP_MS = 1
# Short yield while STEP batches need frequent refilling. Idle keeps its normal
# millisecond sleep; pulse widths and acceleration are unchanged.
MOTOR_LOOP_SLEEP_US = 100

# v2.1.4 gapless RMT pulse train (pre-computed next chunk chained before the
# current one ends; measured 0.3 ms join instead of 4-23 ms refill holes).
TB6600_RMT_PREQUEUE = True
TB6600_RMT_CHAIN_LEAD_US = 10000  # v2.1.7: was 3000; loop passes >3 ms missed the chain -> stop/start gaps up to 8 ms at full speed
# v2.1.4: a side whose switch confirmed during the current move stays
# confirmed for that move even if the switch bounces open afterwards.
GATE_ENDPOINT_CYCLE_LATCH = True

# v2.1.3 motor priority ("stand down" while the NEMA motors run):
#   - automatic garbage collection is paused during motion (a GC of the 8 MB
#     PSRAM heap blocks ~21-24 ms) and done while stationary instead;
#   - every console print during motion is queued and printed after the
#     motors stop (a 600-char print blocks ~48 ms on the UART console).
# ToF/RFID behaviour is unchanged (they were already paused in motion).
MOTION_GC_PAUSE = True
MOTION_GC_MIN_FREE = 1000000          # emergency collect if free heap drops below
MOTION_PRINT_DEFER = True
MOTION_PRINT_QUEUE_MAX = 300          # lines kept; oldest dropped beyond this
MOTION_PRINT_FLUSH_LINES = 12         # lines printed per stationary pass

# v2.0.1 lag monitor: a main-loop pass longer than this is reported on the
# serial log (throttled) and in /api/status -> tasks.
LOOP_SLOW_WARN_MS = 100
LOOP_SLOW_PRINT_INTERVAL_MS = 5000

# v1.9.8 FREEZE-HARDENING
# Boot homing is a cooperative state machine in main.py. It retries forever if
# necessary, but it NEVER traps startup inside a blocking while-loop.
BOOT_HOME_NONBLOCKING = True
BOOT_HOME_STATUS_INTERVAL_MS = 1000

# SD SPI write busy time is independently bounded inside components/sdcard.py.
SD_WRITE_BUSY_TIMEOUT_MS = 750

# Background synchronization starts only when the physical gate is idle.
SYNC_IDLE_GUARD_MS = 250

# ============================================================
# RFID DATABASE TUNING
# ============================================================

# v2.2.0: a card ID is the 32-bit decimal card number. Each binary record
# is that value in 4 big-endian bytes; the bucket file is value mod 256
# (000.bin .. 255.bin).
RFID_RECORD_SIZE = 4
RFID_READ_CHUNK = 1024
RFID_PAGE_DEFAULT = 50
RFID_PAGE_MAX = 200

# Cooperative RFID/logging budgets. The UART reader itself also enforces a
# bounded byte/frame budget, so a card held on the antenna cannot monopolize
# the main loop. Access-log writes are queued and flushed only while the gate is
# idle; they are never allowed to delay the beginning of a motor cycle.
ACCESS_LOG_QUEUE_MAX = 16
RECENT_CARD_CACHE_LIMIT = 32

# ============================================================
# RTC
# ============================================================

FORCE_SET_RTC = False
INITIAL_DATETIME = (2026, 9, 17, 4, 30, 0)
RTC_RESYNC_MS = 60000

# ============================================================
# LED MATRIX ORIENTATION
# ============================================================

SERPENTINE = True
FLIP_X = False
FLIP_Y = True

# ============================================================
# LED COLORS
# ============================================================

OFF = (0, 0, 0)
GREEN_BASE = (0, 80, 0)
RED_BASE = (80, 0, 0)
BLUE_BASE = (0, 0, 50)

# ============================================================
# LED IMAGES
# ============================================================

ARROW_IMAGE = 0x3C3C3C3CFF7E3C18
X_IMAGE = 0xC3E77E3C3C7EE7C3
STANDBY_IMAGES = [
    0x4224994224994224,
    0x9942259942249942,
    0x2499422499422418,
]

STATE_STANDBY = 0
STATE_RESULT = 1
STATE_WAIT_STANDBY = 2

# ============================================================
# DEFAULT RUNTIME CONFIGURATION
# ============================================================
#
# These values are copied at boot and then /config.json is merged
# on top of them. Values changed from the browser are therefore
# persistent and override these defaults until /config.json is
# removed or changed.
# ============================================================

DEFAULT_CONFIG = {
    "ap": {
        "ssid": "FASTLANE-GATE-01",
        "password": "Fastlane123",
        "ip": "192.168.4.1",
        "subnet": "255.255.255.0",
        "channel": 6,
        "max_clients": 4,
    },

    "ethernet": {
        "ip": "192.168.50.2",
        "subnet": "255.255.255.0",
        "gateway": "192.168.50.1",
        "dns": "192.168.50.1",
    },

    # v2.5.0: the web service DECIDES every tap (no card list on the machine).
    #   POST /api/turnstile/tap  APIKey: <key>
    #        {"rfid_uid","gate_id","direction","tap_id"}
    #   -> {"success":true,"data":{"access_result":"granted"|"denied",...}}
    # The gate opens only on "granted". No reply within decision_timeout_ms
    # = denied (fail-closed). A warm keep-alive connection is kept open so a
    # tap costs one request/response. The API key lives in /config.json only.
    "server": {
        "enabled": True,
        "tap_url": "http://192.168.254.103:3000/api/turnstile/tap",
        "health_url": "http://192.168.254.103:3000/health",
        "api_key_header": "APIKey",
        "api_key": "",
        # The gate this turnstile is in the web service (Gates page / _id).
        "gate_id": "6ac24981e07ee7f53c891602",
        "direction": "entry",
        "decision_timeout_ms": 2500,
        "keep_warm": True,
        "timeout_ms": 5000,
        "profile_version": SERVER_PROFILE_VERSION,
    },

    "tof": {
        # v2.0.0 CONTINUOUS FIXED-BACKGROUND PRESENCE.
        #
        # Exactly two physical states: PRESENCE / NO_PRESENCE.
        # Both sensors range continuously IN PARALLEL while the gate is
        # stationary and are not read at all while a motor moves.
        "enabled": True,

        # Closing is allowed only with fresh NO_PRESENCE from BOTH sensors.
        "require_clear_to_close": True,

        # If PRESENCE appears while closing, stop and reopen. Only effective
        # when close_motion_guard=True (ToF is otherwise paused while moving).
        "reopen_on_obstruction": True,
        "close_motion_guard": TOF_CLOSE_MOTION_GUARD_DEFAULT,

        # Raw VL53L0X validity limits. Returns above max_valid_mm (including
        # the 8190/8191 "no target" code) count as CLEAR, not as invalid.
        "min_valid_mm": 50,
        "max_valid_mm": 3000,

        # Calibration: dual_sensor_multi_point_constant_offset
        # (5/10/15/20 cm, schema v2). corrected = raw + offset.
        "sensor1_offset_mm": -43.375,
        "sensor2_offset_mm": -42.0,

        # Fixed background (real/corrected distance to the far side of the
        # lane, mm). 0 = use /presence_range_calibration.json background if
        # calibrated, else /tof_background.json, else capture once and save.
        # Measure with a tape and enter it here to configure it directly.
        # Site value: S1 reads the opposite pillar at ~600-700 mm.
        "sensor1_background_mm": 600,
        "sensor2_background_mm": 600,

        # v2.0.4: a valid-looking return closer than this to the sensor face
        # is crosstalk (mount hole / cover / pillar edge), counted as CLEAR.
        # Per-sensor override: "sensor1_near_blank_mm" / "sensor2_near_blank_mm".
        "near_blank_mm": 150,
        # v2.1.12: echoes >= this signal inside the blank zone are a real
        # person right at the sensor (not cover/crosstalk). 0 = blank all.
        "near_blank_strong_mcps": 2.5,

        # v2.0.7 sensor roles (top view): S1 straight across the ENTRY side
        # (sees the person first). S2 angled ~70 deg through the barrier
        # toward the EXIT (sees the person last). S2 counts only while the
        # gate is stationary OPEN - when closed its beam hits the closed arm,
        # so S1 alone decides "lane clear" for RFID.
        "sensor2_role": "exit",

        # v2.0.8: with the gate OPEN the angled S2 beam also returns a fixed
        # object at ~160-206 mm (opened arm / pillar edge). Anything closer
        # than this to S2 is treated as that fixed object, never a person.
        "sensor2_near_blank_mm": 250,

        # Normal close (clear_close_delay_ms) only after S2 saw the person
        # and cleared again. Entered (S1) but exit never seen by S2 -> the
        # longer delay below, in case they stand between the beams.
        "exit_confirm_required": True,
        "unconfirmed_exit_close_delay_ms": 3000,

        # PRESENCE when median <= background - presence_delta_mm.
        # NO_PRESENCE when median >= background - release_delta_mm.
        "presence_delta_mm": 150,
        "release_delta_mm": 90,

        # Confirmation on the median-of-3 signal (1 = 2-of-3 raw samples).
        "presence_confirm_samples": 2,   # v2.1.1: 2 consecutive (~80 ms)
        "clear_confirm_samples": 2,

        # Driver profile: "legacy" keeps the init sequence the offsets and
        # background were calibrated with. "full" = full ST/Pololu init
        # (SPAD + reference calibration); re-run calibration after switching.
        "driver_profile": "legacy",

        # 0 = keep the device/calibrated timing budget (~33 ms, ~30 Hz per
        # sensor). 20000 = fastest supported (~50 Hz per sensor, slightly
        # more noise); re-check the background after changing it.
        "timing_budget_us": 0,

        # 0 = keep device default.
        "signal_rate_limit_mcps": 0,

        # Use /presence_range_calibration.json background when present.
        "use_calibration_file": True,

        # One-time background capture rules (only when no background known).
        "background_capture_samples": 30,
        "background_capture_max_spread_mm": 80,

        # Stall watchdog: restart continuous ranging if a sensor produces no
        # sample for this long while running. (Key name kept for the Web UI.)
        "measurement_timeout_ms": 250,

        # Sensor data older than this is STALE (not used for new RFID).
        "sensor_stale_ms": 350,

        # Delay from real NO_PRESENCE to CLOSE, in milliseconds.
        "clear_close_delay_ms": 1000,

        # RFID re-arm dwell after PRESENCE / a close cycle (gate LOCKED +
        # fresh NO_PRESENCE continuously for this long).
        "rfid_rearm_clear_ms": RFID_REARM_CLEAR_MS,
        # v2.0.9: while the gate is idle LOCKED (no grant, not moving) a
        # PRESENCE must last this long before it locks RFID / beeps. S1 gave
        # 1-sample false returns (296-449 mm) about every 5 s on an EMPTY
        # lane; each one reset the 1 s RFID re-arm and swallowed the tap.
        # Open / closing / passage safety is NOT debounced (instant).
        "idle_presence_confirm_ms": 200,
        # v2.1.1: weak-echo rejection. A VALID ToF return weaker than this
        # signal rate is treated as CLEAR. Live empty-lane capture: S1 ghost
        # 0.58-0.78 MCPS (status 11, 315-403 mm raw), S2 background 0.46-0.59.
        # A person in a 55-60 cm lane returns much more. Raise if a ghost still
        # appears; lower (or 0 = off) if a dark-clothed person is missed.
        # Per sensor: "sensor1_min_signal_mcps" / "sensor2_min_signal_mcps".
        "min_presence_signal_mcps": 1.0,

        "close_clear_grace_ms": 2000,
        "close_allow_stale_no_presence": True,

        # Any blocked/different RFID while gate is open + freshly clear requests
        # the same automatic close sequence.
        "invalid_rfid_close_enabled": True,
    },

    "gate": {
        # v1.7.19: BOTH sides have physical CLOSE and OPEN endpoint switches.
        # Both TB6600/NEMA channels begin OPEN/CLOSE with the same speed profile.
        # If one side reaches its own OPEN limit first, that motor stops immediately
        # and WAITs while the late motor continues toward its own OPEN limit.
        # The OPEN hold/close-preparation timer does NOT start until BOTH OPEN
        # limits are confirmed. CLOSE uses the same independent endpoint policy.
        "motor_enabled": True,
        "closed_angle": 0.0,
        "open_angle": 80.0,
        "direction_inverted": False,

        # Solenoid / motor opening sequence - FIXED v1.7.4 policy:
        # 1) On GRANTED, the active-low solenoid relay unlocks immediately.
        # 2) Wait motor_start_delay_ms before the latch-release shake begins.
        # 3) Keep the solenoid continuously UNLOCKED through the complete shake.
        # 4) Keep it UNLOCKED through the complete normal dual-NEMA OPEN movement
        #    and any independent OPEN endpoint seek needed by either side.
        # 5) ONLY after OPEN is fully complete, wait the fixed
        #    SOLENOID_LOCK_AFTER_OPEN_DELAY_MS (1000 ms), then LOCK the solenoid.
        #
        # solenoid_unlock_ms is retained only as a backward-compatible config/UI
        # field. main.py forces it to the fixed value and does not use it as a
        # pulse duration while the gate is opening.
        "motor_start_delay_ms": 1000,
        "solenoid_unlock_ms": SOLENOID_LOCK_AFTER_OPEN_DELAY_MS,

        # Latch-release assist - v1.7.4 TWO-CYCLE WIDE SHAKE profile.
        #
        # After the solenoid has been energized for motor_start_delay_ms:
        #   1) initial CLOSE preload
        #   2) alternate OPEN/CLOSE for latch_release_shake_cycles
        #   3) finish on CLOSE
        #   4) hold the final CLOSE preload
        #   5) re-anchor software position to CLOSED
        #   6) begin the normal OPEN movement
        #
        # With the defaults below the actual command pattern is:
        #   C5 -> O5 -> C5 -> O5 -> C5
        #
        # This keeps the wide 5-degree mechanical shake but reduces the repeat
        # count from 3 cycles to 2 so access opens sooner.
        #
        # The assist is used only for the normal LOCKED -> OPEN cycle. It is NOT
        # repeated for VL53L0X obstruction reopening.
        "latch_release_profile_version": LATCH_RELEASE_PROFILE_VERSION,
        "latch_release_jog_enabled": True,
        "latch_release_jog_degrees": LATCH_RELEASE_JOG_DEGREES_DEFAULT,
        "latch_release_jog_delay_us": LATCH_RELEASE_JOG_DELAY_US_DEFAULT,
        "latch_release_shake_cycles": LATCH_RELEASE_SHAKE_CYCLES_DEFAULT,
        "latch_release_shake_pause_ms": LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT,
        "latch_release_settle_ms": LATCH_RELEASE_SETTLE_MS_DEFAULT,

        # Audible feedback.
        "buzzer_enabled": True,
        "buzzer_grant_ms": 120,
        "buzzer_deny_beeps": 2,
        "buzzer_deny_on_ms": 90,
        "buzzer_deny_off_ms": 90,
    },

    "tap": {
        # Ignore the same card for this amount of time after a tap.
        "same_card_cooldown_sec": 3.0,

        # Legacy post-close delay. With VL53L0X enabled, v1.8.4 re-arms access from
        # NO_PRESENCE + BOTH CLOSE limits instead of waiting this extra timer.
        # The value is retained for safety-disabled/backward-compatible operation.
        "next_card_delay_sec": 5.0,

        # Minimum reader-frame processing interval.
        "scan_interval_ms": 100,

        # Delay before the same invalid card may be evaluated again.
        "invalid_card_retry_sec": 1.0,

        # Time the gate remains open before the motor closes it again.
        "gate_unlock_sec": 12.0,

        # v2.0.2: a card tapped while RFID is locked out (PRESENCE / 1 s
        # re-arm) shows RED X + warning beep. It never opens the gate.
        # False = strict "UART not read at all while locked out".
        "lockout_tap_feedback": True,

        # Ignore new RFID taps while the gate is still in its active cycle.
        "ignore_scans_while_gate_busy": True,

        # Result / standby LED timing.
        "grant_display_ms": 2000,
        "deny_display_ms": 2000,
        "standby_delay_ms": 1000,
        "standby_frame_ms": 300,
        "led_brightness_percent": 100,
    },
}
