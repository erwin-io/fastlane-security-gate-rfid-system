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
FIRMWARE_VERSION = "1.4.0"

# ============================================================
# GPIO - RDM6300 125KHZ RFID
# ============================================================

RFID_UART_ID = 1
RFID_RX_PIN = 16
RFID_TX_PIN = 47       # UART assignment only; physically unconnected
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
I2C_FREQUENCY = 100000

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
# GPIO - TB6600 STEPPER DRIVER
# ============================================================
#
# Common-anode wiring used by the tested TB6600 code:
#   TB6600 PUL+ -> 5V
#   TB6600 PUL- -> ESP32 GPIO1
#   TB6600 DIR+ -> 5V
#   TB6600 DIR- -> ESP32 GPIO2
#
# One TB6600 drives both NEMA17 motors. The second motor must be
# wired with one coil pair reversed if it needs to rotate in the
# opposite physical direction from motor 1.
# ============================================================

TB6600_STEP_PIN = 1
TB6600_DIR_PIN = 2

# Tested motor settings from the standalone TB6600 program.
TB6600_FULL_STEPS_PER_REV = 200
TB6600_MICROSTEP = 8
TB6600_STEPS_PER_REV = TB6600_FULL_STEPS_PER_REV * TB6600_MICROSTEP
TB6600_STEPS_PER_DEGREE = TB6600_STEPS_PER_REV / 360.0
TB6600_START_DELAY_US = 2500
TB6600_RUN_DELAY_US = 900
TB6600_ACCEL_STEPS = 80

# Common-anode logic: PUL-/DIR- are driven by the ESP32.
TB6600_STEP_IDLE_LEVEL = 1
TB6600_STEP_ACTIVE_LEVEL = 0
TB6600_FORWARD_DIR_LEVEL = 0
TB6600_REVERSE_DIR_LEVEL = 1
TB6600_DIR_SETUP_MS = 20

# ============================================================
# GPIO - 12V SOLENOID RELAY
# ============================================================

SOLENOID_RELAY_PIN = 15

# FAIL-SAFE RELAY POLARITY
# ------------------------
# The installed relay module is treated as ACTIVE-HIGH:
#   GPIO LOW  = relay de-energized = solenoid unpowered = LOCKED
#   GPIO HIGH = relay energized   = solenoid powered   = UNLOCKED
#
# This is intentionally a STATIC hardware setting. It is NOT editable from
# /config.json or the web UI, so a stale runtime configuration cannot leave
# the gate unlocked after reboot.
SOLENOID_RELAY_ACTIVE_LOW = True
SOLENOID_RELAY_LOCKED_LEVEL = 1 if SOLENOID_RELAY_ACTIVE_LOW else 0
SOLENOID_RELAY_UNLOCKED_LEVEL = 0 if SOLENOID_RELAY_ACTIVE_LOW else 1

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
W5500_SPI_BAUDRATE = 20000000

# ============================================================
# FILESYSTEM PATHS
# ============================================================

SD_MOUNT = "/sd"
LOG_DIRECTORY = "/sd/logs"

RFID_DB_A = "/sd/rfid_db_A"
RFID_DB_B = "/sd/rfid_db_B"
RFID_ACTIVE_FILE = "/sd/rfid_active.txt"
RFID_OLD_DB = "/sd/rfid_db"
LEGACY_RFID_DATABASE = "/sd/rfids.txt"
SYNC_STATE_FILE = "/sd/sync_state.json"

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

# Fast cooperative application loop. Motor pulses are generated by a
# non-blocking state machine, so a short loop sleep improves responsiveness.
MAIN_LOOP_SLEEP_MS = 1

# Background synchronization starts only when the physical gate is idle.
SYNC_IDLE_GUARD_MS = 250

# ============================================================
# RFID DATABASE TUNING
# ============================================================

# RDM6300 Raw ID is 10 hex characters. The first byte is used as
# the bucket name and the remaining four bytes are stored in the
# bucket file, therefore each binary record is four bytes.
RFID_RECORD_SIZE = 4
RFID_READ_CHUNK = 1024
RFID_PAGE_DEFAULT = 50
RFID_PAGE_MAX = 200
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
FLIP_Y = False

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

    "server": {
        "enabled": True,
        "url": "http://192.168.50.1:3000/api/rfids",
        "health_url": "http://192.168.50.1:3000/health",
        "method": "GET",
        "timeout_ms": 5000,
        "max_response_bytes": 131072,
    },

    "sync": {
        "enabled": True,
        "sync_on_boot": False,
        "interval_sec": 60,
        "mode": "delta",

        # JSON response mapping
        # object_array example:
        # {"data":[{"rfid":"030081DFD8","active":true}]}
        # string_array example:
        # {"data":["030081DFD8","03006C34C3"]}
        "record_type": "object_array",
        "records_path": "data",
        "rfid_field": "rfid",
        "active_field": "active",
        "active_value": "true",

        # Delta response mapping
        "action_field": "action",
        "upsert_value": "upsert",
        "delete_value": "delete",

        # Request pagination: none | page | cursor
        "pagination": "page",
        "parameter_location": "query",
        "page_param": "page",
        "page_size_param": "limit",
        "page_start": 1,
        "page_size": 500,
        "cursor_param": "cursor",
        "next_cursor_path": "next_cursor",
        "max_pages_per_run": 5000,
    },

    "gate": {
        # The controller assumes the mechanism is physically at the
        # closed angle when the ESP32 boots. There is no home sensor yet.
        "motor_enabled": True,
        "closed_angle": 0.0,
        "open_angle": 90.0,
        "direction_inverted": False,

        # Solenoid behavior:
        # On GRANTED the relay is energized immediately to release both
        # mechanical locks. It stays released for this fixed pulse time,
        # then returns to LOCK even while the barrier is still moving.
        #
        # The mechanism is expected to slide over / re-catch the lock
        # automatically when the barrier returns to the closed position.
        "solenoid_unlock_ms": 1000,

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

        # Minimum delay before a different card is processed.
        "next_card_delay_sec": 1.0,

        # Minimum reader-frame processing interval.
        "scan_interval_ms": 100,

        # Delay before the same invalid card may be evaluated again.
        "invalid_card_retry_sec": 1.0,

        # Time the gate remains open before the motor closes it again.
        "gate_unlock_sec": 12.0,

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
