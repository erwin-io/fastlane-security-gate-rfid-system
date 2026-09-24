from machine import Pin, SoftSPI
import machine
import network
import socket
import time
import os
import gc

from config import *
from components import sdcard
from components.tb6600 import TB6600DualMotor
from components.w5500 import W5500Ethernet
from components.ds3231 import DS3231Clock, format_datetime
from components.rdm6300 import RDM6300
from components.ws2812b import StatusMatrix
from components.rfid import RFIDDatabase
from components.solenoid import SolenoidRelay
from components.buzzer import Buzzer
from components.webserver import CooperativeWebServer

try:
    import ujson as json
except ImportError:
    import json


# ============================================================
# FASTLANE ONE-WAY ENTRANCE RFID TURNSTILE / SPEED GATE
# MODULAR COMPONENT ARCHITECTURE
# ============================================================
#
# Hardware-specific code lives under /components.
# main.py is the application/orchestration layer.
# ============================================================

config = None

# MicroSD state
sd_spi = None
sd_cs = None
sd = None
sd_ready = False

# Wi-Fi AP / local cooperative web server
ap = None
web_server = None
web_server_ready = False
pending_reboot_at = None

# Access-control runtime state
last_processed_tap_time = 0
recent_cards = {}
last_rfid_uid = ""
last_rfid_status = ""
last_rfid_timestamp = ""
last_rfid_lookup_ms = 0

gate_busy = False
gate_busy_until = 0
gate_state = "LOCKED"

# Sync runtime state
sync_running = False
sync_mode = ""
sync_page = 0
sync_cursor = ""
sync_target_dir = ""
sync_target_name = ""
sync_records_processed = 0
sync_pages_processed = 0
sync_started_at = ""
sync_last_finished_at = ""
sync_last_result = "Never synced"
sync_last_error = ""
sync_last_http_status = 0
sync_next_due_ms = 0
sync_full_count = 0
sync_next_cursor_candidate = ""
sync_persisted_state = {"cursor": "", "last_success": ""}

# Components are created after runtime config is loaded.
clock = None
rfid_reader = None
rfid_db = None
matrix = None
stepper = None
solenoid = None
buzzer = None
ethernet = None


# ============================================================
# GENERIC HELPERS / RUNTIME CONFIG
# ============================================================

def memory_report(label):
    try:
        gc.collect()
        print(
            "MEMORY {}: free={} bytes, allocated={} bytes".format(
                label, gc.mem_free(), gc.mem_alloc()
            )
        )
    except Exception:
        pass


def clamp_int(value, minimum, maximum, default):
    try:
        value = int(value)
    except Exception:
        return default
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def clamp_float(value, minimum, maximum, default):
    try:
        value = float(value)
    except Exception:
        return default
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def deep_copy(value):
    if isinstance(value, dict):
        return {k: deep_copy(v) for k, v in value.items()}
    if isinstance(value, list):
        return [deep_copy(v) for v in value]
    return value


def deep_merge(base, incoming):
    if not isinstance(incoming, dict):
        return base
    for key, value in incoming.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def sanitize_config(cfg):
    ap_cfg = cfg["ap"]
    ap_cfg["ssid"] = str(ap_cfg.get("ssid", "FASTLANE-GATE-01"))[:31]
    password = str(ap_cfg.get("password", "Fastlane123"))
    if len(password) < 8 or len(password) > 63:
        password = "Fastlane123"
    ap_cfg["password"] = password
    ap_cfg["ip"] = str(ap_cfg.get("ip", "192.168.4.1"))
    ap_cfg["subnet"] = str(ap_cfg.get("subnet", "255.255.255.0"))
    ap_cfg["channel"] = clamp_int(ap_cfg.get("channel"), 1, 13, 6)
    ap_cfg["max_clients"] = clamp_int(ap_cfg.get("max_clients"), 1, 10, 4)

    eth = cfg["ethernet"]
    eth["ip"] = str(eth.get("ip", "192.168.50.2"))
    eth["subnet"] = str(eth.get("subnet", "255.255.255.0"))
    eth["gateway"] = str(eth.get("gateway", "192.168.50.1"))
    eth["dns"] = str(eth.get("dns", "192.168.50.1"))

    server = cfg["server"]
    server["enabled"] = bool(server.get("enabled", True))
    server["url"] = str(server.get("url", ""))
    server["health_url"] = str(server.get("health_url", ""))
    method = str(server.get("method", "GET")).upper()
    if method not in ("GET", "POST"):
        method = "GET"
    server["method"] = method
    server["timeout_ms"] = clamp_int(server.get("timeout_ms"), 500, 30000, 5000)
    server["max_response_bytes"] = clamp_int(
        server.get("max_response_bytes"), 4096, 1048576, 131072
    )

    sync_cfg = cfg["sync"]
    sync_cfg["enabled"] = bool(sync_cfg.get("enabled", True))
    sync_cfg["sync_on_boot"] = bool(sync_cfg.get("sync_on_boot", False))
    sync_cfg["interval_sec"] = clamp_int(sync_cfg.get("interval_sec"), 10, 86400, 60)

    mode = str(sync_cfg.get("mode", "delta")).lower()
    if mode not in ("full", "delta"):
        mode = "delta"
    sync_cfg["mode"] = mode

    record_type = str(sync_cfg.get("record_type", "object_array")).lower()
    if record_type not in ("object_array", "string_array"):
        record_type = "object_array"
    sync_cfg["record_type"] = record_type

    for key in (
        "records_path", "rfid_field", "active_field", "active_value",
        "action_field", "upsert_value", "delete_value",
        "page_param", "page_size_param", "cursor_param", "next_cursor_path",
    ):
        sync_cfg[key] = str(sync_cfg.get(key, ""))

    pagination = str(sync_cfg.get("pagination", "page")).lower()
    if pagination not in ("none", "page", "cursor"):
        pagination = "page"
    sync_cfg["pagination"] = pagination

    location = str(sync_cfg.get("parameter_location", "query")).lower()
    if location not in ("query", "json_body"):
        location = "query"
    sync_cfg["parameter_location"] = location
    sync_cfg["page_start"] = clamp_int(sync_cfg.get("page_start"), 0, 1000000, 1)
    sync_cfg["page_size"] = clamp_int(sync_cfg.get("page_size"), 1, 2000, 500)
    sync_cfg["max_pages_per_run"] = clamp_int(
        sync_cfg.get("max_pages_per_run"), 1, 1000000, 5000
    )

    gate_cfg = cfg["gate"]
    gate_cfg["motor_enabled"] = bool(gate_cfg.get("motor_enabled", True))
    gate_cfg["closed_angle"] = clamp_float(gate_cfg.get("closed_angle"), 0.0, 360.0, 0.0)
    gate_cfg["open_angle"] = clamp_float(gate_cfg.get("open_angle"), 0.0, 360.0, 90.0)
    gate_cfg["direction_inverted"] = bool(gate_cfg.get("direction_inverted", False))
    gate_cfg["solenoid_unlock_ms"] = clamp_int(
        gate_cfg.get("solenoid_unlock_ms"), 100, 5000, 1000
    )
    for old_key in ("solenoid_release_delay_ms", "solenoid_lock_delay_ms"):
        if old_key in gate_cfg:
            del gate_cfg[old_key]

    gate_cfg["buzzer_enabled"] = bool(gate_cfg.get("buzzer_enabled", True))
    gate_cfg["buzzer_grant_ms"] = clamp_int(gate_cfg.get("buzzer_grant_ms"), 0, 5000, 120)
    gate_cfg["buzzer_deny_beeps"] = clamp_int(gate_cfg.get("buzzer_deny_beeps"), 0, 10, 2)
    gate_cfg["buzzer_deny_on_ms"] = clamp_int(gate_cfg.get("buzzer_deny_on_ms"), 10, 2000, 90)
    gate_cfg["buzzer_deny_off_ms"] = clamp_int(gate_cfg.get("buzzer_deny_off_ms"), 10, 2000, 90)

    tap = cfg["tap"]
    tap["same_card_cooldown_sec"] = clamp_float(tap.get("same_card_cooldown_sec"), 0.0, 3600.0, 3.0)
    tap["next_card_delay_sec"] = clamp_float(tap.get("next_card_delay_sec"), 0.0, 60.0, 1.0)
    tap["scan_interval_ms"] = clamp_int(tap.get("scan_interval_ms"), 0, 5000, 100)
    tap["invalid_card_retry_sec"] = clamp_float(tap.get("invalid_card_retry_sec"), 0.0, 60.0, 1.0)
    tap["gate_unlock_sec"] = clamp_float(tap.get("gate_unlock_sec"), 0.1, 60.0, 12.0)
    tap["ignore_scans_while_gate_busy"] = bool(tap.get("ignore_scans_while_gate_busy", True))
    if "relay_active_low" in tap:
        del tap["relay_active_low"]
    tap["grant_display_ms"] = clamp_int(tap.get("grant_display_ms"), 100, 10000, 2000)
    tap["deny_display_ms"] = clamp_int(tap.get("deny_display_ms"), 100, 10000, 2000)
    tap["standby_delay_ms"] = clamp_int(tap.get("standby_delay_ms"), 0, 10000, 1000)
    tap["standby_frame_ms"] = clamp_int(tap.get("standby_frame_ms"), 50, 5000, 300)
    tap["led_brightness_percent"] = clamp_int(tap.get("led_brightness_percent"), 1, 100, 100)
    return cfg


def load_config():
    cfg = deep_copy(DEFAULT_CONFIG)
    try:
        with open(CONFIG_FILE, "r") as f:
            saved = json.loads(f.read())
        deep_merge(cfg, saved)
        print("CONFIG LOADED:", CONFIG_FILE)
    except Exception as e:
        print("CONFIG USING DEFAULTS:", repr(e))
    return sanitize_config(cfg)


def save_config():
    try:
        tmp = CONFIG_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write(json.dumps(config))
        try:
            os.remove(CONFIG_FILE)
        except Exception:
            pass
        os.rename(tmp, CONFIG_FILE)
        if hasattr(os, "sync"):
            os.sync()
        print("CONFIG SAVED:", CONFIG_FILE)
        return True
    except Exception as e:
        print("CONFIG SAVE ERROR:", repr(e))
        return False


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def make_directory(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def ticks_after_ms(ms):
    return time.ticks_add(time.ticks_ms(), int(ms))


# ============================================================
# COMPONENT CONSTRUCTION
# ============================================================

def create_components():
    global clock, rfid_reader, rfid_db, matrix, stepper, solenoid, buzzer, ethernet

    clock = DS3231Clock(
        RTC_SDA_PIN,
        RTC_SCL_PIN,
        address=DS3231_ADDRESS,
        frequency=I2C_FREQUENCY,
        force_set=FORCE_SET_RTC,
        initial_datetime=INITIAL_DATETIME,
        resync_ms=RTC_RESYNC_MS,
    )

    rfid_reader = RDM6300(
        RFID_UART_ID,
        RFID_RX_PIN,
        RFID_TX_PIN,
        RFID_BAUD,
    )

    rfid_db = RFIDDatabase(
        RFID_DB_A,
        RFID_DB_B,
        RFID_ACTIVE_FILE,
        RFID_OLD_DB,
        LEGACY_RFID_DATABASE,
        record_size=RFID_RECORD_SIZE,
        read_chunk=RFID_READ_CHUNK,
        page_default=RFID_PAGE_DEFAULT,
        page_max=RFID_PAGE_MAX,
    )

    matrix = StatusMatrix(
        LED_PIN,
        NUM_LEDS,
        MATRIX_WIDTH,
        MATRIX_HEIGHT,
        SERPENTINE,
        FLIP_X,
        FLIP_Y,
        OFF,
        GREEN_BASE,
        RED_BASE,
        BLUE_BASE,
        ARROW_IMAGE,
        X_IMAGE,
        STANDBY_IMAGES,
    )
    matrix.configure(config["tap"])

    stepper = TB6600DualMotor(
        TB6600_STEP_PIN,
        TB6600_DIR_PIN,
        full_steps_per_rev=TB6600_FULL_STEPS_PER_REV,
        microstep=TB6600_MICROSTEP,
        start_delay_us=TB6600_START_DELAY_US,
        run_delay_us=TB6600_RUN_DELAY_US,
        accel_steps=TB6600_ACCEL_STEPS,
        step_idle_level=TB6600_STEP_IDLE_LEVEL,
        step_active_level=TB6600_STEP_ACTIVE_LEVEL,
        forward_dir_level=TB6600_FORWARD_DIR_LEVEL,
        reverse_dir_level=TB6600_REVERSE_DIR_LEVEL,
        dir_setup_ms=TB6600_DIR_SETUP_MS,
    )

    solenoid = SolenoidRelay(
        SOLENOID_RELAY_PIN,
        active_low=SOLENOID_RELAY_ACTIVE_LOW,
    )

    buzzer = Buzzer(BUZZER_PIN, active_high=BUZZER_ACTIVE_HIGH)

    ethernet = W5500Ethernet(
        W5500_SPI_ID,
        W5500_SCK_PIN,
        W5500_MOSI_PIN,
        W5500_MISO_PIN,
        W5500_CS_PIN,
        W5500_INT_PIN,
        W5500_RST_PIN,
        baudrate=W5500_SPI_BAUDRATE,
    )


def initialize_runtime_components():
    rfid_reader.initialize()
    matrix.initialize()
    stepper.initialize(config["gate"]["closed_angle"])
    solenoid.initialize()
    buzzer.initialize()


# ============================================================
# MICROSD
# ============================================================

def initialize_sd():
    global sd_spi, sd_cs, sd, sd_ready

    print()
    print("========================================")
    print("CHECKING MICRO SD")
    print("========================================")
    sd_ready = False

    try:
        if hasattr(os, "umount"):
            try:
                os.umount(SD_MOUNT)
            except Exception:
                pass

        sd_cs = Pin(SD_CS_PIN, Pin.OUT, value=1)
        time.sleep_ms(250)

        sd_spi = SoftSPI(
            baudrate=SD_INIT_BAUDRATE,
            polarity=0,
            phase=0,
            sck=Pin(SD_SCK_PIN),
            mosi=Pin(SD_MOSI_PIN),
            miso=Pin(SD_MISO_PIN),
        )

        print("SD SPI READY")
        print("CS   : GPIO", SD_CS_PIN)
        print("MOSI : GPIO", SD_MOSI_PIN)
        print("SCK  : GPIO", SD_SCK_PIN)
        print("MISO : GPIO", SD_MISO_PIN)

        sd_cs.value(1)
        for _ in range(20):
            sd_spi.write(b"\xFF")
        time.sleep_ms(50)

        sd = sdcard.SDCard(sd_spi, sd_cs, baudrate=SD_BAUDRATE)
        print("SD CARD COMMUNICATION OK")

        if hasattr(os, "VfsFat"):
            filesystem = os.VfsFat(sd)
            os.mount(filesystem, SD_MOUNT)
        else:
            os.mount(sd, SD_MOUNT)

        test_path = SD_MOUNT + "/_test.txt"
        with open(test_path, "w") as f:
            f.write("FASTLANE_SD_OK")
        with open(test_path, "r") as f:
            result = f.read()
        if result != "FASTLANE_SD_OK":
            raise OSError("SD verification mismatch")
        try:
            os.remove(test_path)
        except Exception:
            pass

        if hasattr(os, "sync"):
            os.sync()

        sd_ready = True
        make_directory(LOG_DIRECTORY)
        print("SD READ/WRITE TEST: OK")
        print("MICRO SD READY")
        return True
    except Exception as e:
        print("SD INITIALIZATION FAILED:", repr(e))
        sd_ready = False
        return False


# ============================================================
# ACCESS LOG
# ============================================================

def log_access(uid, status, dt):
    if not sd_ready:
        print("LOG SKIPPED: SD NOT READY")
        return False
    try:
        filename = "{}/{:04d}-{:02d}-{:02d}.csv".format(
            LOG_DIRECTORY, dt[0], dt[1], dt[2]
        )
        new_file = not file_exists(filename)
        with open(filename, "a") as f:
            if new_file:
                f.write("timestamp,rfid,status\n")
            f.write("{},{},{}\n".format(format_datetime(dt), uid, status))
        if hasattr(os, "sync"):
            os.sync()
        return True
    except Exception as e:
        print("LOG ERROR:", repr(e))
        return False


# ============================================================
# GATE APPLICATION LOGIC
# ============================================================

def _mark_gate_open():
    global gate_busy_until, gate_state
    gate_state = "OPEN"
    gate_busy_until = ticks_after_ms(config["tap"]["gate_unlock_sec"] * 1000)
    print("GATE STATUS: OPEN")


def _mark_gate_locked():
    global gate_busy, gate_busy_until, gate_state
    gate_busy = False
    gate_busy_until = 0
    gate_state = "LOCKED"
    print("GATE STATUS: LOCKED")


def _gate_error(message):
    global gate_busy, gate_busy_until, gate_state
    print("GATE ERROR:", message)
    stepper.stop()
    solenoid.force_locked()
    gate_busy = False
    gate_busy_until = 0
    gate_state = "ERROR"


def unlock_gate():
    global gate_busy, gate_busy_until, gate_state

    if gate_busy:
        print("GATE OPEN REQUEST IGNORED: GATE BUSY")
        return False

    gate_busy = True
    gate_busy_until = 0
    gate_state = "OPENING"

    print()
    print("GATE OPEN REQUEST")
    print("MOTOR ENABLED :", config["gate"]["motor_enabled"])
    print("CLOSED ANGLE  :", config["gate"]["closed_angle"])
    print("OPEN ANGLE    :", config["gate"]["open_angle"])
    print("DIR INVERTED  :", config["gate"]["direction_inverted"])

    # Release immediately. The motor is started immediately after this call.
    # Solenoid timing remains independent of motor position.
    solenoid.start_unlock_pulse(config["gate"]["solenoid_unlock_ms"])

    if not config["gate"]["motor_enabled"]:
        print("WARNING: TB6600 MOTOR IS DISABLED IN CONFIG / WEB UI")
        _mark_gate_open()
        return True

    if float(config["gate"]["open_angle"]) == float(config["gate"]["closed_angle"]):
        print("WARNING: OPEN ANGLE EQUALS CLOSED ANGLE - MOTOR HAS NO DISTANCE TO MOVE")

    started = stepper.start_move_to_angle(
        config["gate"]["open_angle"],
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
    )
    if not started:
        _gate_error(stepper.last_error or "TB6600 OPEN START FAILED")
        return False

    # If already at the configured target there is no asynchronous movement.
    if not stepper.moving:
        _mark_gate_open()
    else:
        print("GATE STATUS: OPENING")

    return True


def lock_gate():
    global gate_busy, gate_busy_until, gate_state

    gate_busy_until = 0

    if not config["gate"]["motor_enabled"]:
        _mark_gate_locked()
        return True

    # A reboot/forced close may be requested while an opening move is active.
    # Stop the current pulse state cleanly, preserve the actual step counter,
    # then command a return from the current software position.
    if stepper.moving:
        stepper.stop()

    gate_busy = True
    gate_state = "CLOSING"

    started = stepper.start_move_to_angle(
        config["gate"]["closed_angle"],
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
    )
    if not started:
        _gate_error(stepper.last_error or "TB6600 CLOSE START FAILED")
        return False

    if not stepper.moving:
        _mark_gate_locked()
    else:
        print("GATE STATUS: CLOSING")

    return True


def stop_gate_outputs():
    global gate_busy, gate_busy_until, gate_state
    stepper.stop()
    buzzer.stop()
    solenoid.force_locked()
    gate_busy = False
    gate_busy_until = 0
    gate_state = "STOPPED"


def update_gate():
    # Highest-priority physical timing. These calls are intentionally short.
    solenoid.update()
    buzzer.update()

    if not gate_busy:
        return

    if gate_state == "OPENING":
        stepper.update(during_step=solenoid.update)
        if not stepper.moving:
            if stepper.last_result == "ERROR":
                _gate_error(stepper.last_error or "TB6600 OPEN MOVE ERROR")
            else:
                _mark_gate_open()
        return

    if gate_state == "OPEN":
        if gate_busy_until and time.ticks_diff(time.ticks_ms(), gate_busy_until) >= 0:
            lock_gate()
        return

    if gate_state == "CLOSING":
        stepper.update(during_step=solenoid.update)
        if not stepper.moving:
            if stepper.last_result == "ERROR":
                _gate_error(stepper.last_error or "TB6600 CLOSE MOVE ERROR")
            else:
                _mark_gate_locked()


def service_critical_tasks():
    """Keep the physical gate responsive during W5500 HTTP wait slices."""
    if rfid_reader is not None and config is not None:
        rfid_reader.update(check_access, config["tap"]["scan_interval_ms"])
    update_gate()
    if matrix is not None:
        matrix.update()
    if clock is not None:
        clock.update()


# ============================================================
# RFID ACCESS DECISION
# ============================================================

def cleanup_recent_cards(now):
    if len(recent_cards) <= RECENT_CARD_CACHE_LIMIT:
        return
    oldest_uid = None
    oldest_age = -1
    for uid, entry in recent_cards.items():
        try:
            stamp = entry[0]
        except Exception:
            stamp = entry
        age = time.ticks_diff(now, stamp)
        if age > oldest_age:
            oldest_age = age
            oldest_uid = uid
    if oldest_uid is not None:
        try:
            del recent_cards[oldest_uid]
        except Exception:
            pass


def check_tap_allowed(uid):
    now = time.ticks_ms()

    if config["tap"]["ignore_scans_while_gate_busy"] and gate_busy:
        return False, "GATE_BUSY"

    next_delay_ms = int(config["tap"]["next_card_delay_sec"] * 1000)
    if last_processed_tap_time and time.ticks_diff(now, last_processed_tap_time) < next_delay_ms:
        return False, "NEXT_CARD_DELAY"

    previous = recent_cards.get(uid)
    if previous is not None:
        try:
            previous_time, previous_status = previous
        except Exception:
            previous_time, previous_status = previous, "GRANTED"

        if previous_status == "DENIED":
            cooldown_ms = int(config["tap"]["invalid_card_retry_sec"] * 1000)
            reason = "INVALID_CARD_RETRY"
        else:
            cooldown_ms = int(config["tap"]["same_card_cooldown_sec"] * 1000)
            reason = "SAME_CARD_COOLDOWN"

        if time.ticks_diff(now, previous_time) < cooldown_ms:
            return False, reason

    return True, "OK"


def check_access(uid):
    global last_processed_tap_time
    global last_rfid_uid, last_rfid_status, last_rfid_timestamp, last_rfid_lookup_ms

    allowed, reason = check_tap_allowed(uid)
    if not allowed:
        print("RFID IGNORED:", reason, uid)
        return

    now = time.ticks_ms()
    lookup_start = time.ticks_ms()
    authorized = rfid_db.contains(uid)
    lookup_ms = time.ticks_diff(time.ticks_ms(), lookup_start)

    dt = clock.current_datetime()
    status = "GRANTED" if authorized else "DENIED"

    last_processed_tap_time = now
    recent_cards[uid] = (now, status)
    cleanup_recent_cards(now)

    last_rfid_uid = uid
    last_rfid_status = status
    last_rfid_timestamp = format_datetime(dt)
    last_rfid_lookup_ms = lookup_ms

    print()
    print("========================================")
    print("ACCESS RESULT")
    print("========================================")
    print("RFID      :", uid)
    print("STATUS    :", status)
    print("TIMESTAMP :", format_datetime(dt))
    print("SD LOOKUP :", lookup_ms, "ms")

    log_access(uid, status, dt)

    if authorized:
        matrix.show_result(True)
        gate_opened = unlock_gate()
        if config["gate"]["buzzer_enabled"]:
            buzzer.start_beep(config["gate"]["buzzer_grant_ms"])
        if not gate_opened:
            print("GATE CYCLE ERROR: COULD NOT OPEN")
    else:
        matrix.show_result(False)
        if config["gate"]["buzzer_enabled"]:
            buzzer.start_pattern(
                config["gate"]["buzzer_deny_beeps"],
                config["gate"]["buzzer_deny_on_ms"],
                config["gate"]["buzzer_deny_off_ms"],
            )

    print("========================================")


# ============================================================
# WIFI AP
# ============================================================

def wlan_ap_constant():
    try:
        return network.WLAN.IF_AP
    except Exception:
        return network.AP_IF


def wlan_sta_constant():
    try:
        return network.WLAN.IF_STA
    except Exception:
        return network.STA_IF


def initialize_access_point():
    global ap

    print()
    print("========================================")
    print("STARTING CONFIGURATION ACCESS POINT")
    print("========================================")

    try:
        try:
            sta = network.WLAN(wlan_sta_constant())
            sta.active(False)
        except Exception:
            pass

        ap = network.WLAN(wlan_ap_constant())
        try:
            ap.active(False)
        except Exception:
            pass
        time.sleep_ms(100)

        ap_cfg = config["ap"]
        configured = False
        try:
            kwargs = {
                "ssid": ap_cfg["ssid"],
                "key": ap_cfg["password"],
                "channel": ap_cfg["channel"],
                "max_clients": ap_cfg["max_clients"],
            }
            try:
                kwargs["security"] = network.WLAN.SEC_WPA2
            except Exception:
                pass
            ap.config(**kwargs)
            configured = True
        except Exception:
            pass

        if not configured:
            try:
                authmode = getattr(network, "AUTH_WPA_WPA2_PSK", 3)
                ap.config(
                    essid=ap_cfg["ssid"],
                    password=ap_cfg["password"],
                    authmode=authmode,
                    channel=ap_cfg["channel"],
                    max_clients=ap_cfg["max_clients"],
                )
                configured = True
            except Exception:
                pass

        if not configured:
            try:
                ap.config(essid=ap_cfg["ssid"], password=ap_cfg["password"])
            except Exception:
                ap.config(ssid=ap_cfg["ssid"], key=ap_cfg["password"])

        ap.active(True)
        time.sleep_ms(300)
        try:
            ap.ifconfig((
                ap_cfg["ip"],
                ap_cfg["subnet"],
                ap_cfg["ip"],
                ap_cfg["ip"],
            ))
        except Exception as e:
            print("AP STATIC IP WARNING:", repr(e))

        print("AP SSID:", ap_cfg["ssid"])
        print("AP IP  :", ap.ifconfig()[0])
        print("WEB UI : http://{}".format(ap.ifconfig()[0]))
        return True
    except Exception as e:
        print("AP START ERROR:", repr(e))
        return False


# ============================================================
# SYNC / API MAPPING
# ============================================================

def get_path_value(obj, path, default=None):
    if path is None or str(path).strip() == "":
        return obj
    current = obj
    for part in str(path).split("."):
        if isinstance(current, dict):
            if part not in current:
                return default
            current = current[part]
        elif isinstance(current, list):
            try:
                current = current[int(part)]
            except Exception:
                return default
        else:
            return default
    return current


def scalar_equal(value, expected):
    if isinstance(value, bool):
        actual = "true" if value else "false"
    elif value is None:
        actual = "null"
    else:
        actual = str(value).strip().lower()
    return actual == str(expected).strip().lower()


def extract_sync_operations(payload, mode):
    sync_cfg = config["sync"]
    records = get_path_value(payload, sync_cfg["records_path"], [])
    if not isinstance(records, list):
        raise ValueError("records_path does not point to an array")

    operations = []
    record_type = sync_cfg["record_type"]
    for item in records:
        if record_type == "string_array":
            uid = rfid_db.normalize_uid(item)
            if uid:
                operations.append(("upsert", uid))
            continue

        if not isinstance(item, dict):
            continue

        uid = rfid_db.normalize_uid(get_path_value(item, sync_cfg["rfid_field"], None))
        if uid is None:
            continue

        operation = "upsert"
        if mode == "delta" and sync_cfg["action_field"]:
            action = get_path_value(item, sync_cfg["action_field"], "")
            if scalar_equal(action, sync_cfg["delete_value"]):
                operation = "delete"
            elif scalar_equal(action, sync_cfg["upsert_value"]):
                operation = "upsert"

        if sync_cfg["active_field"]:
            active_value = get_path_value(item, sync_cfg["active_field"], None)
            if not scalar_equal(active_value, sync_cfg["active_value"]):
                if mode == "delta":
                    operation = "delete"
                else:
                    continue

        operations.append((operation, uid))

    return records, operations


def load_sync_persisted_state():
    global sync_persisted_state
    state = {"cursor": "", "last_success": ""}
    if sd_ready:
        try:
            with open(SYNC_STATE_FILE, "r") as f:
                loaded = json.loads(f.read())
            if isinstance(loaded, dict):
                state["cursor"] = str(loaded.get("cursor", ""))
                state["last_success"] = str(loaded.get("last_success", ""))
        except Exception:
            pass
    sync_persisted_state = state
    return sync_persisted_state


def save_sync_persisted_state(cursor, last_success):
    global sync_persisted_state
    sync_persisted_state = {
        "cursor": str(cursor or ""),
        "last_success": str(last_success or ""),
    }
    if not sd_ready:
        return
    try:
        with open(SYNC_STATE_FILE, "w") as f:
            f.write(json.dumps(sync_persisted_state))
        if hasattr(os, "sync"):
            os.sync()
    except Exception as e:
        print("SYNC STATE SAVE ERROR:", repr(e))


def start_sync(mode=None):
    global sync_running, sync_mode, sync_page, sync_cursor
    global sync_target_dir, sync_target_name, sync_records_processed
    global sync_pages_processed, sync_started_at, sync_last_error
    global sync_full_count, sync_next_cursor_candidate

    if sync_running:
        return False, "Sync already running"
    if not sd_ready:
        return False, "SD card not ready"
    if not ethernet.ready:
        return False, "W5500 Ethernet not ready"
    if not config["server"]["enabled"]:
        return False, "Server integration disabled"
    if not config["server"]["url"]:
        return False, "Server URL is empty"

    if mode is None:
        mode = config["sync"]["mode"]
    mode = str(mode).lower()
    if mode not in ("full", "delta"):
        return False, "Invalid sync mode"

    sync_running = True
    sync_mode = mode
    sync_page = config["sync"]["page_start"]
    sync_records_processed = 0
    sync_pages_processed = 0
    sync_started_at = format_datetime(clock.current_datetime())
    sync_last_error = ""
    sync_full_count = 0
    sync_next_cursor_candidate = ""

    sync_cursor = sync_persisted_state.get("cursor", "") if mode == "delta" else ""

    if mode == "full":
        sync_target_name, sync_target_dir = rfid_db.inactive_db()
        rfid_db.remove_directory_files(sync_target_dir)
        rfid_db.write_db_count(sync_target_dir, 0)
    else:
        sync_target_name = rfid_db.active_name
        sync_target_dir = rfid_db.active_dir

    print()
    print("SYNC STARTED:", mode.upper())
    print("Target DB:", sync_target_name)
    return True, "Sync started"


def sync_request_parameters():
    sync_cfg = config["sync"]
    params = {}
    pagination = sync_cfg["pagination"]

    if pagination == "page":
        if sync_cfg["page_param"]:
            params[sync_cfg["page_param"]] = sync_page
        if sync_cfg["page_size_param"]:
            params[sync_cfg["page_size_param"]] = sync_cfg["page_size"]
    elif pagination == "cursor":
        if sync_cfg["page_size_param"]:
            params[sync_cfg["page_size_param"]] = sync_cfg["page_size"]
        if sync_cursor and sync_cfg["cursor_param"]:
            params[sync_cfg["cursor_param"]] = sync_cursor

    return params


def finish_sync(success, message):
    global sync_running, sync_last_finished_at, sync_last_result, sync_last_error
    global sync_next_due_ms

    sync_running = False
    sync_last_finished_at = format_datetime(clock.current_datetime())

    if success and sync_mode == "full":
        if not rfid_db.activate(sync_target_name, sync_target_dir):
            success = False
            message = "Full sync downloaded but active DB marker could not be saved"

    if success:
        if sync_next_cursor_candidate:
            save_sync_persisted_state(sync_next_cursor_candidate, sync_last_finished_at)
        else:
            save_sync_persisted_state(
                sync_persisted_state.get("cursor", ""),
                sync_last_finished_at,
            )
        sync_last_result = "SUCCESS: " + message
        sync_last_error = ""
        print("SYNC COMPLETE:", message)
    else:
        sync_last_result = "FAILED: " + message
        sync_last_error = message
        print("SYNC FAILED:", message)

    sync_next_due_ms = ticks_after_ms(config["sync"]["interval_sec"] * 1000)


def process_full_operations(operations):
    global sync_full_count
    batches = {}
    for operation, uid in operations:
        if operation != "upsert":
            continue
        bucket = uid[0:2]
        if bucket not in batches:
            batches[bucket] = bytearray()
        batches[bucket].extend(rfid_db.uid_record(uid))
        sync_full_count += 1

    for bucket, data in batches.items():
        path = sync_target_dir + "/" + bucket + ".bin"
        with open(path, "ab") as f:
            f.write(data)

    rfid_db.write_db_count(sync_target_dir, sync_full_count)


def process_delta_operations(operations):
    for operation, uid in operations:
        if operation == "delete":
            rfid_db.remove(uid, quiet=True, db_dir=rfid_db.active_dir)
        else:
            rfid_db.add(uid, quiet=True, db_dir=rfid_db.active_dir)


def sync_step():
    global sync_page, sync_cursor, sync_pages_processed, sync_records_processed
    global sync_last_http_status, sync_next_cursor_candidate

    if not sync_running:
        return

    sync_cfg = config["sync"]
    if sync_pages_processed >= sync_cfg["max_pages_per_run"]:
        finish_sync(False, "max_pages_per_run reached")
        return

    try:
        params = sync_request_parameters()
        url = config["server"]["url"]
        body = None
        method = config["server"]["method"]

        if method == "GET" or sync_cfg["parameter_location"] == "query":
            url = ethernet.append_query(url, params)
        else:
            body = params

        status, headers, response_body = ethernet.http_request(
            url,
            method=method,
            json_body=body,
            timeout_ms=config["server"]["timeout_ms"],
            max_body=config["server"]["max_response_bytes"],
            service_callback=service_critical_tasks,
        )
        sync_last_http_status = status

        if status < 200 or status >= 300:
            finish_sync(False, "HTTP {}".format(status))
            return

        payload = json.loads(response_body.decode("utf-8"))
        raw_records, operations = extract_sync_operations(payload, sync_mode)

        if sync_mode == "full":
            process_full_operations(operations)
        else:
            process_delta_operations(operations)

        sync_pages_processed += 1
        sync_records_processed += len(operations)

        pagination = sync_cfg["pagination"]
        done = False
        if pagination == "none":
            done = True
        elif pagination == "page":
            if len(raw_records) < sync_cfg["page_size"]:
                done = True
            else:
                sync_page += 1
        elif pagination == "cursor":
            next_cursor = get_path_value(payload, sync_cfg["next_cursor_path"], "")
            if next_cursor is None:
                next_cursor = ""
            next_cursor = str(next_cursor)
            sync_next_cursor_candidate = next_cursor
            if not next_cursor or next_cursor == sync_cursor:
                done = True
            else:
                sync_cursor = next_cursor

        if hasattr(os, "sync"):
            os.sync()

        if done:
            if sync_mode == "full":
                rfid_db.write_db_count(sync_target_dir, sync_full_count)
                finish_sync(True, "{} records, {} pages".format(sync_full_count, sync_pages_processed))
            else:
                finish_sync(True, "{} changes, {} pages".format(sync_records_processed, sync_pages_processed))
    except Exception as e:
        finish_sync(False, repr(e))


def update_sync_scheduler():
    global sync_next_due_ms
    if not config["sync"]["enabled"] or sync_running:
        return

    # Never begin a background network job while the physical gate is active.
    if gate_busy or (stepper is not None and stepper.moving):
        return

    now = time.ticks_ms()
    if last_processed_tap_time and time.ticks_diff(now, last_processed_tap_time) < SYNC_IDLE_GUARD_MS:
        return
    if sync_next_due_ms == 0:
        sync_next_due_ms = ticks_after_ms(config["sync"]["interval_sec"] * 1000)
        return

    if time.ticks_diff(now, sync_next_due_ms) >= 0:
        ok, message = start_sync(config["sync"]["mode"])
        if not ok:
            sync_next_due_ms = ticks_after_ms(config["sync"]["interval_sec"] * 1000)


# ============================================================
# LOCAL WEB SERVER
# ============================================================

def url_decode(text):
    text = str(text).replace("+", " ")
    out = ""
    i = 0
    while i < len(text):
        if text[i] == "%" and i + 2 < len(text):
            try:
                out += chr(int(text[i + 1:i + 3], 16))
                i += 3
                continue
            except Exception:
                pass
        out += text[i]
        i += 1
    return out


def parse_query(query):
    result = {}
    if not query:
        return result
    for part in query.split("&"):
        if "=" in part:
            key, value = part.split("=", 1)
        else:
            key, value = part, ""
        result[url_decode(key)] = url_decode(value)
    return result


def split_path_query(target):
    if "?" in target:
        return target.split("?", 1)
    return target, ""


def json_response(data, status=200):
    return status, "application/json", json.dumps(data)


def get_status_payload():
    ap_ip = ""
    ap_clients = None
    try:
        ap_ip = ap.ifconfig()[0] if ap else ""
    except Exception:
        pass
    try:
        ap_clients = len(ap.config("stations")) if ap else 0
    except Exception:
        ap_clients = None

    persisted = sync_persisted_state
    next_sync_sec = None
    if config["sync"]["enabled"] and sync_next_due_ms:
        diff = time.ticks_diff(sync_next_due_ms, time.ticks_ms())
        next_sync_sec = max(0, diff // 1000)

    eth_ifconfig = ethernet.ifconfig() if ethernet else None
    eth_link = ethernet.is_connected() if ethernet else False
    motor_status = stepper.status()
    solenoid_status = solenoid.status()

    return {
        "time": format_datetime(clock.current_datetime()),
        "rtc_ready": clock.ready,
        "sd_ready": sd_ready,
        "rfid_count": rfid_db.count,
        "active_db": rfid_db.active_name,
        "gate_busy": gate_busy,
        "gate": {
            "state": gate_state,
            "busy": gate_busy,
            "motor_enabled": config["gate"]["motor_enabled"],
            "motor_moving": motor_status["moving"],
            "motor_initialized": motor_status["initialized"],
            "motor_angle": motor_status["angle"],
            "motor_target_angle": motor_status["target_angle"],
            "motor_progress_percent": motor_status["progress_percent"],
            "motor_last_result": motor_status["last_result"],
            "motor_last_error": motor_status["last_error"],
            "open_angle": config["gate"]["open_angle"],
            "closed_angle": config["gate"]["closed_angle"],
            "solenoid_released": solenoid_status["released"],
            "solenoid_unlock_ms": config["gate"]["solenoid_unlock_ms"],
            "solenoid_remaining_ms": solenoid_status["remaining_ms"],
        },
        "last_rfid": {
            "uid": last_rfid_uid,
            "status": last_rfid_status,
            "timestamp": last_rfid_timestamp,
            "lookup_ms": last_rfid_lookup_ms,
        },
        "ap": {
            "ssid": config["ap"]["ssid"],
            "ip": ap_ip,
            "clients": ap_clients,
            "web_ready": web_server_ready,
        },
        "ethernet": {
            "ready": ethernet.ready if ethernet else False,
            "link": eth_link,
            "ifconfig": eth_ifconfig,
            "error": ethernet.last_error if ethernet else "",
        },
        "sync": {
            "running": sync_running,
            "mode": sync_mode,
            "pages": sync_pages_processed,
            "records": sync_records_processed,
            "started_at": sync_started_at,
            "last_finished_at": sync_last_finished_at,
            "last_result": sync_last_result,
            "last_error": sync_last_error,
            "last_http_status": sync_last_http_status,
            "cursor": persisted.get("cursor", ""),
            "next_sync_sec": next_sync_sec,
        },
    }


def validate_and_update_config(incoming):
    global config, sync_next_due_ms

    if not isinstance(incoming, dict):
        raise ValueError("JSON object required")

    new_cfg = deep_copy(config)
    deep_merge(new_cfg, incoming)
    new_cfg = sanitize_config(new_cfg)

    requested_ap = incoming.get("ap", {}) if isinstance(incoming.get("ap", {}), dict) else {}
    if "password" in requested_ap:
        requested_password = str(requested_ap["password"])
        if len(requested_password) < 8 or len(requested_password) > 63:
            raise ValueError("AP password must be 8 to 63 characters")

    config = new_cfg
    if not save_config():
        raise OSError("Could not save /config.json")

    matrix.configure(config["tap"])
    if not gate_busy:
        solenoid.force_locked()
    sync_next_due_ms = ticks_after_ms(config["sync"]["interval_sec"] * 1000)
    return True


def route_http(method, target, body_bytes):
    global pending_reboot_at

    path, query_text = split_path_query(target)
    query = parse_query(query_text)

    if path == "/favicon.ico":
        return 204, "text/plain", ""

    if path == "/api/status" and method == "GET":
        return json_response(get_status_payload())

    if path == "/api/config" and method == "GET":
        return json_response(config)

    if path == "/api/config" and method == "POST":
        try:
            incoming = json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
            validate_and_update_config(incoming)
            return json_response({
                "ok": True,
                "message": "Configuration saved. AP/Ethernet changes require restart.",
                "reboot_required": True,
            })
        except Exception as e:
            return json_response({"ok": False, "error": str(e)}, 400)

    if path == "/api/rfids" and method == "GET":
        cursor = query.get("cursor", "")
        limit = clamp_int(query.get("limit", RFID_PAGE_DEFAULT), 1, RFID_PAGE_MAX, RFID_PAGE_DEFAULT)
        return json_response(rfid_db.list_page(cursor, limit))

    if path == "/api/rfids/search" and method == "GET":
        uid = rfid_db.normalize_uid(query.get("uid", ""))
        if uid is None:
            return json_response({"ok": False, "error": "Invalid 10-character RFID"}, 400)
        start = time.ticks_ms()
        found = rfid_db.contains(uid)
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        return json_response({
            "ok": True,
            "rfid": uid,
            "found": found,
            "authorized": found,
            "bucket": rfid_db.bucket_path_for(uid),
            "lookup_ms": elapsed,
        })

    if path == "/api/rfids" and method == "POST":
        try:
            payload = json.loads(body_bytes.decode("utf-8"))
            uid = rfid_db.normalize_uid(payload.get("rfid", ""))
            if uid is None:
                raise ValueError("Invalid 10-character RFID")
            if rfid_db.contains(uid):
                return json_response({"ok": False, "error": "RFID already exists", "rfid": uid}, 409)
            if not rfid_db.add(uid, quiet=True):
                raise OSError("Could not add RFID")
            return json_response({"ok": True, "rfid": uid, "count": rfid_db.count})
        except Exception as e:
            return json_response({"ok": False, "error": str(e)}, 400)

    if path == "/api/rfids" and method == "DELETE":
        uid = rfid_db.normalize_uid(query.get("uid", ""))
        if uid is None:
            return json_response({"ok": False, "error": "Invalid 10-character RFID"}, 400)
        if not rfid_db.remove(uid, quiet=True):
            return json_response({"ok": False, "error": "RFID not found", "rfid": uid}, 404)
        return json_response({"ok": True, "rfid": uid, "count": rfid_db.count})

    if path == "/api/server/test" and method == "POST":
        try:
            url = config["server"]["health_url"].strip() or config["server"]["url"].strip()
            status, headers, response_body = ethernet.http_request(
                url,
                method="GET",
                timeout_ms=config["server"]["timeout_ms"],
                max_body=4096,
                service_callback=service_critical_tasks,
            )
            preview = response_body[:512].decode("utf-8", "ignore")
            return json_response({
                "ok": 200 <= status < 300,
                "status": status,
                "url": url,
                "preview": preview,
            })
        except Exception as e:
            return json_response({"ok": False, "error": repr(e)}, 502)

    if path == "/api/sync/run" and method == "POST":
        try:
            payload = json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
            mode = payload.get("mode", config["sync"]["mode"])
            ok, message = start_sync(mode)
            return json_response({"ok": ok, "message": message}, 200 if ok else 409)
        except Exception as e:
            return json_response({"ok": False, "error": repr(e)}, 400)

    if path == "/api/sync/status" and method == "GET":
        return json_response(get_status_payload()["sync"])

    if path == "/api/reboot" and method == "POST":
        pending_reboot_at = ticks_after_ms(700)
        return json_response({"ok": True, "message": "ESP32 restarting..."})

    return json_response({"ok": False, "error": "Not found"}, 404)


def initialize_web_server():
    global web_server, web_server_ready
    web_server_ready = False
    try:
        ip = ap.ifconfig()[0]
        web_server = CooperativeWebServer(
            route_callback=route_http,
            root_file=WEBUI_FILE,
            file_chunk_size=WEBUI_CHUNK_SIZE,
            recv_chunk_size=WEB_RECV_CHUNK_SIZE,
            send_chunk_size=WEB_SEND_CHUNK_SIZE,
            client_timeout_ms=WEB_CLIENT_TIMEOUT_MS,
        )
        web_server_ready = web_server.initialize(ip, 80)
        return web_server_ready
    except Exception as e:
        print("WEB SERVER ERROR:", repr(e))
        web_server_ready = False
        return False


def service_web_server():
    if web_server_ready and web_server is not None:
        web_server.update()


# ============================================================
# STARTUP
# ============================================================

print()
print("========================================")
print("FASTLANE RFID ACCESS CONTROL")
print("ONE-WAY ENTRANCE GATE")
print("MODULAR COMPONENT BUILD", FIRMWARE_VERSION)
print("========================================")
print("ESP32-S3")
print("RDM6300 125KHz")
print("WS2812B 8x8")
print("DS3231 RTC")
print("MICRO SD SPI")
print("W5500 ETHERNET")
print("TB6600 + 2x NEMA17")
print("SOLENOID RELAY")
print("========================================")

memory_report("BOOT START")
config = load_config()
create_components()
memory_report("CONFIG + COMPONENT OBJECTS")

initialize_runtime_components()
memory_report("RUNTIME COMPONENTS")

clock.initialize()
memory_report("RTC")

initialize_sd()
memory_report("SD")

rfid_db.initialize(sd_ready)
load_sync_persisted_state()
memory_report("RFID DATABASE")

initialize_access_point()
memory_report("WIFI AP")

initialize_web_server()
memory_report("WEB SERVER")

ethernet.initialize(config["ethernet"])
memory_report("W5500")

matrix.start_standby()
gc.collect()

sync_next_due_ms = ticks_after_ms(config["sync"]["interval_sec"] * 1000)
if config["sync"]["enabled"] and config["sync"]["sync_on_boot"]:
    ok, msg = start_sync(config["sync"]["mode"])
    print("BOOT SYNC:", msg)

print()
print("========================================")
print("SYSTEM READY")
print("========================================")
print("TIME:", format_datetime(clock.current_datetime()))
print("RTC READY:", clock.ready)
print("SD READY:", sd_ready)
print("RFID IDS IN RAM: 0")
print("RFID DB COUNT:", rfid_db.count)
print("ACTIVE RFID DB:", rfid_db.active_name)
print("MOTOR ENABLED:", config["gate"]["motor_enabled"])
print("MOTOR CLOSED ANGLE:", config["gate"]["closed_angle"])
print("MOTOR OPEN ANGLE:", config["gate"]["open_angle"])
print("MOTOR DIRECTION INVERTED:", config["gate"]["direction_inverted"])
try:
    print("CONFIG AP:", ap.ifconfig()[0])
except Exception:
    print("CONFIG AP: NOT READY")
try:
    print("W5500 IP:", ethernet.ifconfig()[0])
except Exception:
    print("W5500 IP: NOT READY")
print()
print("GPIO MAP")
print("----------------------------------------")
print("TB6600 STEP/PUL- -> GPIO", TB6600_STEP_PIN)
print("TB6600 DIR-      -> GPIO", TB6600_DIR_PIN)
print("W5500 SCK        -> GPIO", W5500_SCK_PIN)
print("W5500 MOSI       -> GPIO", W5500_MOSI_PIN)
print("W5500 MISO       -> GPIO", W5500_MISO_PIN)
print("W5500 CS         -> GPIO", W5500_CS_PIN)
print("DS3231 SDA       -> GPIO", RTC_SDA_PIN)
print("DS3231 SCL       -> GPIO", RTC_SCL_PIN)
print("SD CS            -> GPIO", SD_CS_PIN)
print("SD MOSI          -> GPIO", SD_MOSI_PIN)
print("SD SCK           -> GPIO", SD_SCK_PIN)
print("SD MISO          -> GPIO", SD_MISO_PIN)
print("W5500 INT        -> GPIO", W5500_INT_PIN)
print("SOLENOID RELAY   -> GPIO", SOLENOID_RELAY_PIN)
print("RDM6300 TX/RX    -> GPIO", RFID_RX_PIN)
print("BUZZER           -> GPIO", BUZZER_PIN)
print("WS2812B DIN      -> GPIO", LED_PIN)
print("W5500 RST        -> GPIO", W5500_RST_PIN)
print("RFID DUMMY TX    -> GPIO", RFID_TX_PIN, "(NOT CONNECTED)")
print("========================================")
print("Waiting for RFID...")


# ============================================================
# MAIN LOOP
# ============================================================

while True:
    try:
        # PRIORITY 1: access-control and physical gate hardware.
        rfid_reader.update(check_access, config["tap"]["scan_interval_ms"])
        update_gate()

        # PRIORITY 2: local visual/clock services.
        matrix.update()
        clock.update()

        # PRIORITY 3: local configuration UI. Cooperative server performs only
        # a small receive/send/file chunk each loop iteration.
        service_web_server()

        # PRIORITY 4: background W5500 synchronization. Automatic sync never
        # starts while the gate is active. W5500 wait slices call
        # service_critical_tasks() so an RFID tap can still operate the gate.
        if sync_running:
            if not gate_busy and not stepper.moving:
                sync_step()
        else:
            update_sync_scheduler()

        if pending_reboot_at is not None:
            if time.ticks_diff(time.ticks_ms(), pending_reboot_at) >= 0:
                if gate_busy:
                    if gate_state != "CLOSING":
                        lock_gate()
                else:
                    matrix.clear()
                    solenoid.force_locked()
                    machine.reset()

        time.sleep_ms(MAIN_LOOP_SLEEP_MS)

    except KeyboardInterrupt:
        matrix.clear()
        stop_gate_outputs()
        print()
        print("PROGRAM STOPPED")
        break

    except Exception as e:
        print()
        print("SYSTEM ERROR:", repr(e))
        time.sleep_ms(20)
