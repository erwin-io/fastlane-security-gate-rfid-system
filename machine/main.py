from machine import Pin, SoftSPI, I2C
try:
    from machine import SoftI2C
except ImportError:
    SoftI2C = None
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
from components.card_id import normalize_card_id, card_id_from_frame_hex
from components.solenoid import SolenoidRelay
from components.buzzer import Buzzer
from components.webserver import CooperativeWebServer
from components.tap_client import TapClient
from components.vl53l0x import DualVL53L0XSafety
from components.manual_button import (ManualOpenButton, SHORT as BUTTON_SHORT, LONG as BUTTON_LONG,
                                      ARMED as BUTTON_ARMED, TOO_SHORT as BUTTON_TOO_SHORT)

# ============================================================
# v2.1.3 MOTOR PRIORITY: console output is held while the motors run
# ============================================================
# A console print blocks the single MicroPython thread until the UART has
# taken the text (measured: 600 chars = 48 ms). While the NEMA motors run,
# that starved the RMT pulse refills (gaps up to 43 ms -> stutter / stall).
# Every print made during motion is queued as text and printed after the
# motors stop. Components get the same print so their messages queue too.
_REAL_PRINT = print
_motion_print_queue = []
_motion_print_dropped = 0


def _print_should_defer():
    try:
        return bool(MOTION_PRINT_DEFER and motion_active())
    except Exception:
        return False


def print(*args, **kw):
    global _motion_print_dropped
    if _print_should_defer():
        try:
            text = kw.get("sep", " ").join([str(a) for a in args])
        except Exception:
            text = "<unprintable>"
        if len(_motion_print_queue) >= MOTION_PRINT_QUEUE_MAX:
            _motion_print_queue.pop(0)
            _motion_print_dropped += 1
        _motion_print_queue.append(text)
        return
    _REAL_PRINT(*args, **kw)


def flush_motion_prints(max_lines=None):
    """Print queued motion-time lines while stationary (bounded per call)."""
    global _motion_print_dropped
    if not _motion_print_queue or _print_should_defer():
        return 0
    n = MOTION_PRINT_FLUSH_LINES if max_lines is None else int(max_lines)
    done = 0
    if _motion_print_dropped:
        _REAL_PRINT("[MOTION LOG] {} older line(s) dropped".format(_motion_print_dropped))
        _motion_print_dropped = 0
    while _motion_print_queue and done < n:
        _REAL_PRINT(_motion_print_queue.pop(0))
        done += 1
    return done


def _install_motion_print(modules):
    for mod in modules:
        try:
            mod.print = print
        except Exception:
            pass


import components.tb6600 as _m_tb6600
import components.solenoid as _m_solenoid
import components.buzzer as _m_buzzer
import components.vl53l0x as _m_vl53
import components.rdm6300 as _m_rdm
import components.webserver as _m_web
import components.ws2812b as _m_led
import components.w5500 as _m_eth
import components.manual_button as _m_btn
import components.tap_client as _m_tap
_install_motion_print((_m_tb6600, _m_solenoid, _m_buzzer, _m_vl53, _m_rdm,
                       _m_web, _m_led, _m_eth, _m_btn, _m_tap))

_motion_gc_check_at = 0


def _motion_gc_guard():
    """While GC is paused in motion, collect only if the heap gets low."""
    global _motion_gc_check_at
    if not MOTION_GC_PAUSE:
        return
    now = time.ticks_ms()
    if _motion_gc_check_at and time.ticks_diff(now, _motion_gc_check_at) < 500:
        return
    _motion_gc_check_at = now
    try:
        if gc.mem_free() < MOTION_GC_MIN_FREE:
            gc.collect()
    except Exception:
        pass


def _pre_motion_gc():
    """Collect while still stationary, just before a motion starts."""
    if MOTION_GC_PAUSE:
        try:
            gc.collect()
        except Exception:
            pass

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
manual_button = None
pending_rfid_card_id = ""
rfid_deny_until = 0          # v2.1.11 global lockout after a DENIED card
pending_rfid_until = 0

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

# v2.3.0 machine configuration mode (GPIO19 5 s hold). AP + web are OFF
# unless this is True; ToF/RFID/motors are disabled while it is True.
config_mode_active = False
config_mode_since = 0
config_mode_toggle_count = 0
config_tof_capture_until = 0
config_tof_capture_started = 0

# Access-control runtime state
last_processed_tap_time = 0
recent_cards = {}
last_rfid_card_id = ""
last_rfid_status = ""
last_rfid_timestamp = ""
last_rfid_lookup_ms = 0

gate_busy = False
gate_busy_until = 0
gate_open_since = 0          # v2.1.1: ticks when BOTH OPEN limits confirmed
gate_close_slow_mode = False  # v2.1.1: True after a blocked/expired close window
presence_clear_close_delay_ms = 0
gate_close_retry_wait_since = 0
gate_state = "LOCKED"
# When opening/reopening, the solenoid is released first and the motor starts
# only after this non-blocking deadline expires.
gate_motor_start_at = 0

# v1.7.4 fixed solenoid policy:
# keep the latch released until NEMA OPEN is fully complete, then wait the
# static SOLENOID_LOCK_AFTER_OPEN_DELAY_MS before returning the relay to LOCK.
gate_solenoid_lock_at = 0

# Normal LOCKED -> OPEN cycles perform a deliberate alternating latch-release
# shake after the solenoid is released and before the normal OPEN movement.
gate_latch_settle_until = 0
gate_latch_shake_leg = 0
gate_latch_shake_total_legs = 0
gate_latch_shake_pause_until = 0
gate_latch_single_open = False   # v2.1.6 SLOW_OPEN latch release
open_slip_back_count = [0, 0]   # v2.1.7: CLOSE switch re-pressed while OPENING

# v1.7.6 final OPEN seating/preload state.
gate_open_seat_settle_until = 0

# v1.7.16 authoritative endpoint switches - one OPEN/CLOSE pair per motor.
# Active-low with ESP32 internal pull-ups.
#
# Motor #1 / LEFT : CLOSE GPIO45, OPEN GPIO38
# Motor #2 / RIGHT: CLOSE GPIO39, OPEN GPIO42
gate_close_limit_pin = None
gate_open_limit_pin = None
gate_close_limit2_pin = None
gate_open_limit2_pin = None

gate_close_limit_active = False
gate_open_limit_active = False
gate_close_limit2_active = False
gate_open_limit2_active = False

_gate_close_limit_candidate = False
_gate_open_limit_candidate = False
_gate_close_limit2_candidate = False
_gate_open_limit2_candidate = False

_gate_close_limit_changed_at = 0
_gate_open_limit_changed_at = 0
_gate_close_limit2_changed_at = 0
_gate_open_limit2_changed_at = 0
gate_limits_armed = False

# OPEN/CLOSE endpoint wait state.
gate_open_limit_deadline = 0
gate_open_limit_confirmed = False
gate_open_limit_timed_out = False
gate_close_limit_deadline = 0

# v1.7.16 dual-side OPEN recovery scheduler. A failed 2-second OPEN-limit find attempt
# stops the motors, waits 2 seconds, then retries OPEN until BOTH OPEN limits are active.
gate_open_retry_at = 0
gate_open_retry_count = 0
gate_open_retry_reason = ""

# CLOSE recovery scheduler.
# A failed 2-second CLOSE-limit find attempt stops the motors, waits the fixed
# 2-second interval, then tries CLOSE again in the normal cooperative main loop.
gate_close_retry_at = 0
gate_close_retry_count = 0
gate_close_retry_reason = ""
boot_home_complete = False

# v1.9.8 cooperative boot-home runtime.
# No startup while-loop is allowed to own the VM indefinitely.
boot_home_attempt_deadline = 0
boot_home_retry_at = 0
boot_home_attempt_count = 0
boot_home_last_status_at = 0

# Anti-tailgating runtime.
# The configured next_card_delay_sec begins only AFTER a successful normal CLOSE
# cycle physically touches the CLOSE limit switch.
anti_tailgate_until = 0
last_granted_card_id = ""
last_granted_at = 0

# v1.9.0 simple boolean presence-integrated access session.
# A GRANTED transaction remains active from the accepted RFID tap until BOTH
# physical CLOSE limits confirm the completed cycle.
#
# NO_PRESENCE before the person enters is PRE-ENTRY and must not close early.
# Once PRESENCE has been seen for that grant, the transition back to
# NO_PRESENCE arms the 1.0-second close delay.
presence_last_state = None
presence_state_changed_at = 0
granted_passage_active = False
granted_passage_card_id = ""
granted_person_seen = False
presence_clear_close_at = 0

# v1.9.5 explicit close request raised by blocked/different RFID while
# the passage is freshly NO_PRESENCE.
auto_close_requested = False
auto_close_reason = ""

# A new RFID requires fresh NO_PRESENCE to remain stable for a short dwell.
# This timer is reset by PRESENCE, stale sensor data, a new grant, and every
# completed transaction so an old clear result can never authorize the next user.
rfid_clear_ready_since = 0

# Warning throttles prevent a continuously-present card/person from restarting
# the warning cadence on every cooperative loop.
turnstile_warn_last_at = 0
blocked_rfid_warn_card_id = ""
blocked_rfid_warn_at = 0

# v2.0.0 VL53L0X runtime service state. The driver self-schedules its I2C
# polls; only the presence application flow is throttled here.
tof_service_next_at = 0
presence_flow_next_at = 0
tof_runtime_error_text = ""
tof_runtime_error_print_at = 0
tof_runtime_error_count = 0

# v2.0.0 explicit task scheduler state (see config.py task table).
task_motion_active = False
idle_presence_since = 0
idle_glitch_count = 0
idle_glitch_log_at = 0
idle_presence_confirmed = False
task_rfid_enabled = False
task_rfid_disable_reason = "BOOT"
rfid_presence_lockout = True
rfid_lockout_reason = "BOOT"
rfid_rearm_clear_since = 0
rfid_resume_purged_bytes = 0

# W5500 link monitor / web motion throttle / shared I2C speed.
eth_link_up = None
eth_link_next_check_at = 0
eth_link_changes = 0
web_motion_next_service_at = 0
i2c_frequency_active = I2C_FREQUENCY
i2c_health_next_check_at = 0

# Sync page downloaded while the gate started moving: processed (SD writes)
# only after motion ends.

# v2.0.7 passage direction tracking (S1 = entry beam, S2 = exit beam).
passage_s1_seen = False
passage_s2_seen = False
passage_exit_confirmed = False
passage_s1_prev = False
passage_s2_prev = False
passage_tailgate_count = 0
passage_exit_confirmed_at = 0

# Gate states in which the exit beam (S2) is counted.
EXIT_SENSOR_GATE_STATES = ("OPEN", "WAIT_CLEAR", "OPEN_RETRY_WAIT")

# v2.0.5 solenoid EMI quiet window + optional hardware watchdog.
i2c_quiet_until = 0
solenoid_last_released = None
wdt = None

# v2.0.1 main-loop lag monitor.
loop_prev_top_us = 0
loop_max_gap_ms = 0
loop_slow_count = 0
loop_slow_print_at = 0
loop_window_max_ms = 0
loop_window_started = 0
loop_window_max_ms_last = [0]

# SD access logs are queued so an SD write/fsync cannot delay the critical
# RFID -> solenoid -> motor path. One record is flushed only while gate is idle.
pending_access_logs = []

# Sync runtime state
# v2.5.0 server-decided access (components/tap_client.py)
tap_client = TapClient()          # Ethernet attached at boot
tap_decision = None               # {"card", "started", "tap_id"} while waiting
tap_seq = 0
try:
    import machine as _mach
    BOOT_TAG = "".join("%02x" % b for b in bytes(_mach.unique_id())[-2:]) + "%04x" % (time.ticks_ms() & 0xFFFF)
except Exception:
    BOOT_TAG = "%04x" % (time.ticks_ms() & 0xFFFF)

# Components are created after runtime config is loaded.
shared_i2c = None
clock = None
tof = None
rfid_reader = None
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
    for key in ("tap_url", "health_url"):
        server[key] = str(server.get(key, "")).strip()
    server["api_key"] = str(server.get("api_key", "")).strip()[:256]
    header = str(server.get("api_key_header", "APIKey")).strip()
    if not header or ":" in header or " " in header:
        header = "APIKey"
    server["api_key_header"] = header[:64]
    server["gate_id"] = str(server.get("gate_id", "")).strip()[:64]
    direction = str(server.get("direction", "entry")).strip().lower()
    if direction not in ("entry", "exit"):
        direction = "entry"
    server["direction"] = direction
    server["keep_warm"] = bool(server.get("keep_warm", True))
    server["decision_timeout_ms"] = clamp_int(server.get("decision_timeout_ms"), 500, 10000, 2500)
    server["profile_version"] = clamp_int(
        server.get("profile_version"), 0, 100000, SERVER_PROFILE_VERSION
    )
    server["timeout_ms"] = clamp_int(server.get("timeout_ms"), 500, 30000, 5000)
    # v2.5.0: no card list / sync on the machine any more.
    for key in ("url", "ack_url", "method", "max_response_bytes", "tap_notify_enabled"):
        server.pop(key, None)
    cfg.pop("sync", None)

    gate_cfg = cfg["gate"]
    gate_cfg["motor_enabled"] = bool(gate_cfg.get("motor_enabled", True))
    gate_cfg["closed_angle"] = clamp_float(gate_cfg.get("closed_angle"), 0.0, 360.0, 0.0)
    gate_cfg["open_angle"] = clamp_float(gate_cfg.get("open_angle"), 0.0, 360.0, 80.0)
    gate_cfg["direction_inverted"] = bool(gate_cfg.get("direction_inverted", False))
    # Mechanical unlock must happen before the NEMA motor is allowed to move.
    # Minimum is intentionally 1000 ms as required for this gate mechanism.
    gate_cfg["motor_start_delay_ms"] = clamp_int(
        gate_cfg.get("motor_start_delay_ms"), 1000, 10000, 1000
    )
    # v1.7.4: solenoid timing is a fixed machine rule, not a tunable pulse.
    # Preserve the old key for Web UI/config compatibility, but always force it
    # to the static post-OPEN lock delay. The opening state machine itself keeps
    # the relay released until NEMA OPEN has fully completed.
    gate_cfg["solenoid_unlock_ms"] = int(SOLENOID_LOCK_AFTER_OPEN_DELAY_MS)

    # Wide alternating latch-release shake used only before a normal OPEN.
    # Normal FASTLANE OPEN/CLOSE speed is NOT changed by these values.
    gate_cfg["latch_release_profile_version"] = clamp_int(
        gate_cfg.get("latch_release_profile_version"),
        0,
        100,
        LATCH_RELEASE_PROFILE_VERSION,
    )
    gate_cfg["latch_release_jog_enabled"] = bool(
        gate_cfg.get("latch_release_jog_enabled", True)
    )
    gate_cfg["latch_release_jog_degrees"] = clamp_float(
        gate_cfg.get("latch_release_jog_degrees"),
        0.5,
        12.0,
        LATCH_RELEASE_JOG_DEGREES_DEFAULT,
    )
    gate_cfg["latch_release_jog_delay_us"] = clamp_int(
        gate_cfg.get("latch_release_jog_delay_us"),
        1500,
        15000,
        LATCH_RELEASE_JOG_DELAY_US_DEFAULT,
    )
    gate_cfg["latch_release_shake_cycles"] = clamp_int(
        gate_cfg.get("latch_release_shake_cycles"),
        1,
        6,
        LATCH_RELEASE_SHAKE_CYCLES_DEFAULT,
    )
    gate_cfg["latch_release_shake_pause_ms"] = clamp_int(
        gate_cfg.get("latch_release_shake_pause_ms"),
        0,
        1000,
        LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT,
    )
    gate_cfg["latch_release_settle_ms"] = clamp_int(
        gate_cfg.get("latch_release_settle_ms"),
        0,
        3000,
        LATCH_RELEASE_SETTLE_MS_DEFAULT,
    )

    for old_key in ("solenoid_release_delay_ms", "solenoid_lock_delay_ms"):
        if old_key in gate_cfg:
            del gate_cfg[old_key]

    gate_cfg["buzzer_enabled"] = bool(gate_cfg.get("buzzer_enabled", True))
    gate_cfg["buzzer_grant_ms"] = clamp_int(gate_cfg.get("buzzer_grant_ms"), 0, 5000, 120)
    gate_cfg["buzzer_deny_beeps"] = clamp_int(gate_cfg.get("buzzer_deny_beeps"), 0, 10, 2)
    gate_cfg["buzzer_deny_on_ms"] = clamp_int(gate_cfg.get("buzzer_deny_on_ms"), 10, 2000, 90)
    gate_cfg["buzzer_deny_off_ms"] = clamp_int(gate_cfg.get("buzzer_deny_off_ms"), 10, 2000, 90)

    tof_cfg = cfg["tof"]
    tof_cfg["enabled"] = bool(
        tof_cfg.get("enabled", True)
    )

    tof_cfg["require_clear_to_close"] = bool(
        tof_cfg.get(
            "require_clear_to_close",
            True,
        )
    )

    tof_cfg["reopen_on_obstruction"] = bool(
        tof_cfg.get(
            "reopen_on_obstruction",
            True,
        )
    )

    tof_cfg["min_valid_mm"] = clamp_int(
        tof_cfg.get("min_valid_mm"),
        20,
        1000,
        50,
    )

    tof_cfg["max_valid_mm"] = clamp_int(
        tof_cfg.get("max_valid_mm"),
        200,
        4000,
        3000,
    )

    tof_cfg["sensor1_offset_mm"] = clamp_float(
        tof_cfg.get("sensor1_offset_mm"),
        -1000.0,
        1000.0,
        -43.375,
    )

    tof_cfg["sensor2_offset_mm"] = clamp_float(
        tof_cfg.get("sensor2_offset_mm"),
        -1000.0,
        1000.0,
        -42.0,
    )

    tof_cfg["measurement_timeout_ms"] = clamp_int(
        tof_cfg.get("measurement_timeout_ms"),
        80,
        2000,
        250,
    )

    tof_cfg["sensor_stale_ms"] = clamp_int(
        tof_cfg.get("sensor_stale_ms"),
        100,
        5000,
        350,
    )

    tof_cfg["clear_close_delay_ms"] = clamp_int(
        tof_cfg.get("clear_close_delay_ms"),
        0,
        10000,
        PRESENCE_CLEAR_CLOSE_DELAY_MS,
    )

    tof_cfg["close_clear_grace_ms"] = clamp_int(
        tof_cfg.get("close_clear_grace_ms"),
        500,
        10000,
        2000,
    )

    tof_cfg["close_allow_stale_no_presence"] = bool(
        tof_cfg.get("close_allow_stale_no_presence", True)
    )

    tof_cfg["invalid_rfid_close_enabled"] = bool(
        tof_cfg.get("invalid_rfid_close_enabled", True)
    )

    # v2.0.0 continuous / fixed-background presence settings.
    tof_cfg["close_motion_guard"] = bool(
        tof_cfg.get("close_motion_guard", TOF_CLOSE_MOTION_GUARD_DEFAULT)
    )
    for bg_key in ("sensor1_background_mm", "sensor2_background_mm"):
        tof_cfg[bg_key] = clamp_float(tof_cfg.get(bg_key), 0.0, 4000.0, 0.0)
    tof_cfg["presence_delta_mm"] = clamp_int(tof_cfg.get("presence_delta_mm"), 20, 2000, 150)
    tof_cfg["release_delta_mm"] = clamp_int(tof_cfg.get("release_delta_mm"), 5, 1990, 90)
    if tof_cfg["release_delta_mm"] >= tof_cfg["presence_delta_mm"]:
        tof_cfg["release_delta_mm"] = max(5, tof_cfg["presence_delta_mm"] - 20)
    tof_cfg["presence_confirm_samples"] = clamp_int(tof_cfg.get("presence_confirm_samples"), 1, 10, 1)
    tof_cfg["clear_confirm_samples"] = clamp_int(tof_cfg.get("clear_confirm_samples"), 1, 20, 2)
    profile = str(tof_cfg.get("driver_profile", "legacy")).lower()
    if profile not in ("legacy", "full"):
        profile = "legacy"
    tof_cfg["driver_profile"] = profile
    budget = clamp_int(tof_cfg.get("timing_budget_us"), 0, 500000, 0)
    if 0 < budget < 20000:
        budget = 20000
    tof_cfg["timing_budget_us"] = budget
    tof_cfg["signal_rate_limit_mcps"] = clamp_float(tof_cfg.get("signal_rate_limit_mcps"), 0.0, 10.0, 0.0)
    tof_cfg["use_calibration_file"] = bool(tof_cfg.get("use_calibration_file", True))
    tof_cfg["background_capture_samples"] = clamp_int(tof_cfg.get("background_capture_samples"), 10, 200, 30)
    tof_cfg["background_capture_max_spread_mm"] = clamp_int(
        tof_cfg.get("background_capture_max_spread_mm"), 10, 500, 80
    )
    tof_cfg["near_blank_mm"] = clamp_int(tof_cfg.get("near_blank_mm"), 0, 1000, 150)
    role2 = str(tof_cfg.get("sensor2_role", "exit")).lower()
    tof_cfg["sensor2_role"] = role2 if role2 in ("entry", "exit") else "exit"
    tof_cfg["exit_confirm_required"] = bool(tof_cfg.get("exit_confirm_required", True))
    tof_cfg["unconfirmed_exit_close_delay_ms"] = clamp_int(
        tof_cfg.get("unconfirmed_exit_close_delay_ms"), 0, 30000, 3000)
    for nb_key in ("sensor1_near_blank_mm", "sensor2_near_blank_mm"):
        if nb_key in tof_cfg:
            tof_cfg[nb_key] = clamp_int(tof_cfg.get(nb_key), 0, 1000, 150)
    tof_cfg["rfid_rearm_clear_ms"] = clamp_int(
        tof_cfg.get("rfid_rearm_clear_ms"), 0, 10000, RFID_REARM_CLEAR_MS
    )
    tof_cfg["idle_presence_confirm_ms"] = clamp_int(
        tof_cfg.get("idle_presence_confirm_ms"), 0, 2000, 200
    )
    tof_cfg["min_presence_signal_mcps"] = clamp_float(
        tof_cfg.get("min_presence_signal_mcps"), 0.0, 50.0, 1.0)
    for sig_key in ("sensor1_min_signal_mcps", "sensor2_min_signal_mcps"):
        if sig_key in tof_cfg:
            tof_cfg[sig_key] = clamp_float(tof_cfg.get(sig_key), 0.0, 50.0, 1.0)

    # Remove obsolete older presence tuning from saved config.
    # (sensorN_background_mm is valid again in v2.0.0.)
    for old_key in (
        "require_presence_for_tap",
        "result_interval_ms",
        "presence_result_ratio",
        "enter_intrusion_mm",
        "exit_intrusion_mm",
        "absolute_presence_mm",
        "filter_size",
        "presence_confirm_count",
        "clear_confirm_count",
        "invalid_limit",
    ):
        if old_key in tof_cfg:
            del tof_cfg[old_key]

    tap = cfg["tap"]
    tap["same_card_cooldown_sec"] = clamp_float(tap.get("same_card_cooldown_sec"), 0.0, 3600.0, 3.0)
    tap["next_card_delay_sec"] = clamp_float(tap.get("next_card_delay_sec"), 2.0, 60.0, 5.0)
    tap["scan_interval_ms"] = clamp_int(tap.get("scan_interval_ms"), 0, 5000, 100)
    tap["invalid_card_retry_sec"] = clamp_float(tap.get("invalid_card_retry_sec"), 0.0, 60.0, 1.0)
    tap["gate_unlock_sec"] = clamp_float(tap.get("gate_unlock_sec"), 0.1, 60.0, 12.0)
    tap["ignore_scans_while_gate_busy"] = bool(tap.get("ignore_scans_while_gate_busy", True))
    if "relay_active_low" in tap:
        del tap["relay_active_low"]
    tap["lockout_tap_feedback"] = bool(tap.get("lockout_tap_feedback", True))
    tap["grant_display_ms"] = clamp_int(tap.get("grant_display_ms"), 100, 10000, 2000)
    tap["deny_display_ms"] = clamp_int(tap.get("deny_display_ms"), 100, 10000, 2000)
    tap["standby_delay_ms"] = clamp_int(tap.get("standby_delay_ms"), 0, 10000, 1000)
    tap["standby_frame_ms"] = clamp_int(tap.get("standby_frame_ms"), 50, 5000, 300)
    tap["led_brightness_percent"] = clamp_int(tap.get("led_brightness_percent"), 1, 100, 100)
    return cfg


def _migrate_server_profile(saved):
    """v2.5.0: older saved server/sync sections -> server-decided profile.

    Keeps the operator's key, gate, direction, switch and timeout; everything
    about the removed card sync is dropped. Returns True if changed.
    """
    if not isinstance(saved, dict):
        return False
    old_server = saved.get("server", {})
    if not isinstance(old_server, dict):
        old_server = {}
    try:
        version = int(old_server.get("profile_version", 0))
    except Exception:
        version = 0
    if version >= SERVER_PROFILE_VERSION and "sync" not in saved:
        return False
    server = {"profile_version": SERVER_PROFILE_VERSION}
    for key in ("enabled", "timeout_ms", "api_key", "gate_id", "direction",
                "api_key_header", "decision_timeout_ms", "keep_warm"):
        if key in old_server:
            server[key] = old_server[key]
    saved["server"] = server
    saved.pop("sync", None)
    print("CONFIG MIGRATION: SERVER -> v2.5.0 SERVER-DECIDED ACCESS (card sync removed)")
    return True


def load_config():
    cfg = deep_copy(DEFAULT_CONFIG)
    persist_server_profile = False
    persist_next_delay_upgrade = False

    try:
        with open(CONFIG_FILE, "r") as f:
            saved = json.loads(f.read())

        saved_gate = saved.get("gate", {}) if isinstance(saved, dict) else {}
        if not isinstance(saved_gate, dict):
            saved_gate = {}
        try:
            saved_latch_profile = int(saved_gate.get("latch_release_profile_version", 0))
        except Exception:
            saved_latch_profile = 0

        # v1.7.8 anti-tailgating migration.
        # If an existing config explicitly saved less than 2 seconds, force it
        # to 2 seconds and persist the corrected value. If the key is absent,
        # DEFAULT_CONFIG supplies the requested 5-second default.
        saved_tap = saved.get("tap", {}) if isinstance(saved, dict) else {}
        if not isinstance(saved_tap, dict):
            saved_tap = {}
        if "next_card_delay_sec" in saved_tap:
            try:
                if float(saved_tap.get("next_card_delay_sec")) < 2.0:
                    saved_tap["next_card_delay_sec"] = 2.0
                    saved["tap"] = saved_tap
                    persist_next_delay_upgrade = True
            except Exception:
                saved_tap["next_card_delay_sec"] = 2.0
                saved["tap"] = saved_tap
                persist_next_delay_upgrade = True

        persist_server_profile = _migrate_server_profile(saved)

        deep_merge(cfg, saved)

        if saved_latch_profile < LATCH_RELEASE_PROFILE_VERSION:
            gate_cfg = cfg["gate"]
            gate_cfg["latch_release_profile_version"] = LATCH_RELEASE_PROFILE_VERSION
            gate_cfg["latch_release_jog_degrees"] = LATCH_RELEASE_JOG_DEGREES_DEFAULT
            gate_cfg["latch_release_jog_delay_us"] = LATCH_RELEASE_JOG_DELAY_US_DEFAULT
            gate_cfg["latch_release_shake_cycles"] = LATCH_RELEASE_SHAKE_CYCLES_DEFAULT
            gate_cfg["latch_release_shake_pause_ms"] = LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT
            gate_cfg["latch_release_settle_ms"] = LATCH_RELEASE_SETTLE_MS_DEFAULT
            print(
                "CONFIG MIGRATION: LATCH SHAKE PROFILE -> v{} "
                "({} deg / {} cycles / {} us / {} ms pause / {} ms hold)".format(
                    LATCH_RELEASE_PROFILE_VERSION,
                    LATCH_RELEASE_JOG_DEGREES_DEFAULT,
                    LATCH_RELEASE_SHAKE_CYCLES_DEFAULT,
                    LATCH_RELEASE_JOG_DELAY_US_DEFAULT,
                    LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT,
                    LATCH_RELEASE_SETTLE_MS_DEFAULT,
                )
            )

        cfg = sanitize_config(cfg)

        if persist_next_delay_upgrade or persist_server_profile:
            try:
                tmp = CONFIG_FILE + ".tmp"
                with open(tmp, "w") as f:
                    f.write(json.dumps(cfg))
                try:
                    os.remove(CONFIG_FILE)
                except Exception:
                    pass
                os.rename(tmp, CONFIG_FILE)
                if hasattr(os, "sync"):
                    os.sync()
                print("CONFIG MIGRATION: SAVED TO", CONFIG_FILE)
            except Exception as e:
                print("CONFIG MIGRATION SAVE WARNING:", repr(e))

        print("CONFIG LOADED:", CONFIG_FILE)
        return cfg

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
# MECHANICAL CLOSE / OPEN LIMIT SWITCHES
# ============================================================

def _limit_raw_pressed(pin):
    if pin is None:
        return False
    value = pin.value()
    return value == 0 if GATE_LIMIT_ACTIVE_LOW else value == 1


_limit_conflict_since = 0


def gate_limits_conflict():
    """True only when one motor side reports OPEN and CLOSE simultaneously
    for LIMIT_CONFLICT_CONFIRM_MS (v2.5.1: a short EMI blip on one switch wire
    is no longer a fault that stops the gate)."""
    global _limit_conflict_since
    if not GATE_LIMIT_SWITCHES_ENABLED:
        return False
    motor1_conflict = gate_close_limit_active and gate_open_limit_active
    motor2_conflict = gate_close_limit2_active and gate_open_limit2_active
    if not (motor1_conflict or motor2_conflict):
        _limit_conflict_since = 0
        return False
    now = time.ticks_ms()
    if not _limit_conflict_since:
        _limit_conflict_since = now or 1
        return False
    return time.ticks_diff(now, _limit_conflict_since) >= int(LIMIT_CONFLICT_CONFIRM_MS)


def gate_limit_conflict_detail():
    conflicts = []
    if gate_close_limit_active and gate_open_limit_active:
        conflicts.append("MOTOR #1 GPIO{}+GPIO{}".format(
            GATE_MOTOR1_CLOSE_LIMIT_PIN, GATE_MOTOR1_OPEN_LIMIT_PIN
        ))
    if gate_close_limit2_active and gate_open_limit2_active:
        conflicts.append("MOTOR #2 GPIO{}+GPIO{}".format(
            GATE_MOTOR2_CLOSE_LIMIT_PIN, GATE_MOTOR2_OPEN_LIMIT_PIN
        ))
    return ", ".join(conflicts)


def gate_limits_enforced():
    return bool(GATE_LIMIT_SWITCHES_ENABLED)


def _motor_limit_active(motor_no, movement):
    movement = str(movement).upper()
    if motor_no == 1:
        return bool(gate_open_limit_active if movement == "OPEN" else gate_close_limit_active)
    return bool(gate_open_limit2_active if movement == "OPEN" else gate_close_limit2_active)


def _motor_limit_pin(motor_no, movement):
    movement = str(movement).upper()
    if motor_no == 1:
        return int(GATE_MOTOR1_OPEN_LIMIT_PIN if movement == "OPEN" else GATE_MOTOR1_CLOSE_LIMIT_PIN)
    return int(GATE_MOTOR2_OPEN_LIMIT_PIN if movement == "OPEN" else GATE_MOTOR2_CLOSE_LIMIT_PIN)


endpoint_cycle_latch = {"OPEN": [False, False], "CLOSE": [False, False]}


def _latch_endpoint(motor_no, movement):
    try:
        endpoint_cycle_latch[str(movement).upper()][motor_no - 1] = True
    except Exception:
        pass


def _clear_endpoint_latch(movement=None):
    for key in (("OPEN", "CLOSE") if movement is None else (str(movement).upper(),)):
        endpoint_cycle_latch[key][0] = False
        endpoint_cycle_latch[key][1] = False


def _endpoint_side_ok(motor_no, movement):
    """v2.1.4: live switch, or confirmed earlier in THIS move (bounce-proof).

    The latch only counts while the gate is busy with a cycle; an idle gate
    (LOCKED) always uses the live switch so forced-open detection stays.
    """
    if _motor_limit_active(motor_no, movement):
        return True
    if not GATE_ENDPOINT_CYCLE_LATCH or not gate_busy:
        return False
    try:
        return bool(endpoint_cycle_latch[str(movement).upper()][motor_no - 1])
    except Exception:
        return False


def _all_endpoint_limits_active(movement):
    # v2.1.1: a running seating push (OPEN or CLOSE) must finish first.
    if stepper is not None and stepper.open_preload_active():
        return False
    return bool(_endpoint_side_ok(1, movement) and _endpoint_side_ok(2, movement))


def _missing_endpoint_motors(movement):
    missing = []
    if not _motor_limit_active(1, movement):
        missing.append(1)
    if not _motor_limit_active(2, movement):
        missing.append(2)
    return missing


def _missing_endpoint_text(movement):
    missing = _missing_endpoint_motors(movement)
    if not missing:
        return "NONE"
    return ", ".join("MOTOR #{}".format(m) for m in missing)


def initialize_gate_limit_switches():
    global gate_close_limit_pin, gate_open_limit_pin
    global gate_close_limit2_pin, gate_open_limit2_pin
    global gate_close_limit_active, gate_open_limit_active
    global gate_close_limit2_active, gate_open_limit2_active
    global _gate_close_limit_candidate, _gate_open_limit_candidate
    global _gate_close_limit2_candidate, _gate_open_limit2_candidate
    global _gate_close_limit_changed_at, _gate_open_limit_changed_at
    global _gate_close_limit2_changed_at, _gate_open_limit2_changed_at
    global gate_limits_armed

    if not GATE_LIMIT_SWITCHES_ENABLED:
        gate_limits_armed = False
        print("GATE LIMIT SWITCHES: DISABLED")
        return not GATE_LIMIT_SWITCHES_REQUIRED

    pull = Pin.PULL_UP if GATE_LIMIT_ACTIVE_LOW else Pin.PULL_DOWN
    gate_close_limit_pin = Pin(GATE_MOTOR1_CLOSE_LIMIT_PIN, Pin.IN, pull)
    gate_open_limit_pin = Pin(GATE_MOTOR1_OPEN_LIMIT_PIN, Pin.IN, pull)
    gate_close_limit2_pin = Pin(GATE_MOTOR2_CLOSE_LIMIT_PIN, Pin.IN, pull)
    gate_open_limit2_pin = Pin(GATE_MOTOR2_OPEN_LIMIT_PIN, Pin.IN, pull)

    now = time.ticks_ms()
    gate_close_limit_active = _limit_raw_pressed(gate_close_limit_pin)
    gate_open_limit_active = _limit_raw_pressed(gate_open_limit_pin)
    gate_close_limit2_active = _limit_raw_pressed(gate_close_limit2_pin)
    gate_open_limit2_active = _limit_raw_pressed(gate_open_limit2_pin)

    _gate_close_limit_candidate = gate_close_limit_active
    _gate_open_limit_candidate = gate_open_limit_active
    _gate_close_limit2_candidate = gate_close_limit2_active
    _gate_open_limit2_candidate = gate_open_limit2_active

    _gate_close_limit_changed_at = now
    _gate_open_limit_changed_at = now
    _gate_close_limit2_changed_at = now
    _gate_open_limit2_changed_at = now
    gate_limits_armed = True

    print("GATE LIMIT SWITCHES READY - DUAL-SIDE AUTHORITATIVE")
    print("MOTOR #1 CLOSE -> GPIO{} | ACTIVE={}".format(
        GATE_MOTOR1_CLOSE_LIMIT_PIN, gate_close_limit_active
    ))
    print("MOTOR #1 OPEN  -> GPIO{} | ACTIVE={}".format(
        GATE_MOTOR1_OPEN_LIMIT_PIN, gate_open_limit_active
    ))
    print("MOTOR #2 CLOSE -> GPIO{} | ACTIVE={}".format(
        GATE_MOTOR2_CLOSE_LIMIT_PIN, gate_close_limit2_active
    ))
    print("MOTOR #2 OPEN  -> GPIO{} | ACTIVE={}".format(
        GATE_MOTOR2_OPEN_LIMIT_PIN, gate_open_limit2_active
    ))
    print("LIMIT ACTIVE LOW:", GATE_LIMIT_ACTIVE_LOW)
    print("LIMIT DEBOUNCE  :", GATE_LIMIT_DEBOUNCE_MS, "ms")
    print("IDLE GLITCH FILT:", GATE_IDLE_LIMIT_GLITCH_FILTER_MS, "ms")
    print("LIMITS REQUIRED :", GATE_LIMIT_SWITCHES_REQUIRED)
    print("LIMIT FIND MAX  :", GATE_LIMIT_FIND_MAX_MS, "ms FIXED PER ATTEMPT")
    print("RETRY INTERVAL  :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")

    if gate_limits_conflict():
        print("GATE LIMIT FAULT:", gate_limit_conflict_detail())
        return False

    return True


def _limit_transition_debounce_ms(endpoint, new_state):
    """Choose fast movement debounce or strong stationary glitch filtering."""
    normal_ms = int(GATE_LIMIT_DEBOUNCE_MS)
    idle_filter_ms = int(GATE_IDLE_LIMIT_GLITCH_FILTER_MS)

    endpoint = str(endpoint).upper()
    new_state = bool(new_state)

    # gate_busy remains True throughout an authorized passage, including OPEN
    # and its stationary settle. It must not disable the endpoint bounce filter.
    # Active arrivals still use the fast debounce; sustained release still wins.
    if gate_state in ("OPEN_SEAT_SETTLE", "OPEN", "WAIT_CLEAR"):
        if stepper is None or not stepper.moving:
            if endpoint.endswith("_OPEN") and not new_state:
                return idle_filter_ms
            if endpoint.endswith("_CLOSE") and new_state:
                return idle_filter_ms

    try:
        moving = bool(
            gate_busy
            or (
                stepper is not None
                and stepper.moving
            )
        )
    except Exception:
        moving = bool(gate_busy)

    if moving:
        # v2.5.1: while the barrier moves toward CLOSE, an OPEN switch turning
        # ACTIVE is physically impossible - it is motor/solenoid EMI on the
        # switch wire (field log 05 Oct: GPIO42 blip -> LIMIT CONFLICT -> gate
        # stopped mid-close). Require it to hold like an idle glitch.
        if new_state and endpoint.endswith("_OPEN") and "CLOS" in str(gate_state):
            return max(normal_ms, idle_filter_ms)
        return normal_ms

    endpoint = str(endpoint).upper()
    new_state = bool(new_state)

    # Stationary CLOSED/LOCKED gate:
    # CLOSE becoming released and OPEN becoming active are suspicious changes.
    if gate_state == "LOCKED":
        if endpoint.endswith("_CLOSE") and not new_state:
            return idle_filter_ms
        if endpoint.endswith("_OPEN") and new_state:
            return idle_filter_ms

    # Stationary OPEN gate:
    # OPEN becoming released and CLOSE becoming active are suspicious changes.
    if gate_state == "OPEN":
        if endpoint.endswith("_OPEN") and not new_state:
            return idle_filter_ms
        if endpoint.endswith("_CLOSE") and new_state:
            return idle_filter_ms

    return normal_ms


def _note_open_slip_back(motor_no):
    """v2.1.7: a CLOSE switch that re-presses while the gate is OPENING means
    that arm was pushed back = its motor lost steps (torque / load), not CPU."""
    if gate_state not in ("OPENING", "REOPENING", "OPEN_WAIT_MOTOR1",
                          "OPEN_WAIT_MOTOR2", "OPEN_LIMIT_SEEK", "OPEN_LIMIT_RETRY"):
        return
    try:
        open_slip_back_count[motor_no - 1] += 1
    except Exception:
        pass
    print("OPEN SLIP-BACK: MOTOR #{} CLOSE switch re-pressed while OPENING "
          "(motor lost steps - check TB6600 #{} current/DIP, arm drag, PSU) | "
          "count M1={} M2={}".format(motor_no, motor_no,
                                     open_slip_back_count[0], open_slip_back_count[1]))


def update_gate_limit_switches():
    global gate_close_limit_active, gate_open_limit_active
    global gate_close_limit2_active, gate_open_limit2_active
    global _gate_close_limit_candidate, _gate_open_limit_candidate
    global _gate_close_limit2_candidate, _gate_open_limit2_candidate
    global _gate_close_limit_changed_at, _gate_open_limit_changed_at
    global _gate_close_limit2_changed_at, _gate_open_limit2_changed_at

    if not GATE_LIMIT_SWITCHES_ENABLED:
        return
    if (
        gate_close_limit_pin is None
        or gate_open_limit_pin is None
        or gate_close_limit2_pin is None
        or gate_open_limit2_pin is None
    ):
        return

    now = time.ticks_ms()

    # ---------------- Motor #1 CLOSE ----------------
    raw = _limit_raw_pressed(gate_close_limit_pin)
    if raw != _gate_close_limit_candidate:
        _gate_close_limit_candidate = raw
        _gate_close_limit_changed_at = now
    elif raw != gate_close_limit_active:
        if time.ticks_diff(now, _gate_close_limit_changed_at) >= _limit_transition_debounce_ms("M1_CLOSE", raw):
            gate_close_limit_active = raw
            print("MOTOR #1 CLOSE LIMIT:", "ACTIVE" if raw else "RELEASED")
            if raw:
                _note_open_slip_back(1)
            if raw and not gate_busy and stepper is not None and not stepper.moving:
                stepper.reconcile_motor1_position(
                    config["gate"]["closed_angle"],
                    reason="idle Motor #1 CLOSE limit confirmed",
                )

    # ---------------- Motor #1 OPEN ----------------
    raw = _limit_raw_pressed(gate_open_limit_pin)
    if raw != _gate_open_limit_candidate:
        _gate_open_limit_candidate = raw
        _gate_open_limit_changed_at = now
    elif raw != gate_open_limit_active:
        if time.ticks_diff(now, _gate_open_limit_changed_at) >= _limit_transition_debounce_ms("M1_OPEN", raw):
            gate_open_limit_active = raw
            print("MOTOR #1 OPEN LIMIT:", "ACTIVE" if raw else "RELEASED")

    # ---------------- Motor #2 CLOSE ----------------
    raw = _limit_raw_pressed(gate_close_limit2_pin)
    if raw != _gate_close_limit2_candidate:
        _gate_close_limit2_candidate = raw
        _gate_close_limit2_changed_at = now
    elif raw != gate_close_limit2_active:
        if time.ticks_diff(now, _gate_close_limit2_changed_at) >= _limit_transition_debounce_ms("M2_CLOSE", raw):
            gate_close_limit2_active = raw
            print("MOTOR #2 CLOSE LIMIT:", "ACTIVE" if raw else "RELEASED")
            if raw:
                _note_open_slip_back(2)
            if raw and not gate_busy and stepper is not None and not stepper.moving:
                stepper.reconcile_motor2_position(
                    config["gate"]["closed_angle"],
                    reason="idle Motor #2 CLOSE limit confirmed",
                )

    # ---------------- Motor #2 OPEN ----------------
    raw = _limit_raw_pressed(gate_open_limit2_pin)
    if raw != _gate_open_limit2_candidate:
        _gate_open_limit2_candidate = raw
        _gate_open_limit2_changed_at = now
    elif raw != gate_open_limit2_active:
        if time.ticks_diff(now, _gate_open_limit2_changed_at) >= _limit_transition_debounce_ms("M2_OPEN", raw):
            gate_open_limit2_active = raw
            print("MOTOR #2 OPEN LIMIT:", "ACTIVE" if raw else "RELEASED")


def gate_limit_status():
    raw_m1_close = _limit_raw_pressed(gate_close_limit_pin) if gate_close_limit_pin is not None else False
    raw_m1_open = _limit_raw_pressed(gate_open_limit_pin) if gate_open_limit_pin is not None else False
    raw_m2_close = _limit_raw_pressed(gate_close_limit2_pin) if gate_close_limit2_pin is not None else False
    raw_m2_open = _limit_raw_pressed(gate_open_limit2_pin) if gate_open_limit2_pin is not None else False

    all_close = bool(gate_close_limit_active and gate_close_limit2_active)
    all_open = bool(gate_open_limit_active and gate_open_limit2_active)

    return {
        "enabled": bool(GATE_LIMIT_SWITCHES_ENABLED),
        "required": bool(GATE_LIMIT_SWITCHES_REQUIRED),
        "active_low": bool(GATE_LIMIT_ACTIVE_LOW),
        "debounce_ms": int(GATE_LIMIT_DEBOUNCE_MS),
        "idle_glitch_filter_ms": int(GATE_IDLE_LIMIT_GLITCH_FILTER_MS),
        "all_close_active": all_close,
        "all_open_active": all_open,
        "conflict": bool(gate_limits_conflict()),
        "conflict_detail": gate_limit_conflict_detail(),
        "armed": bool(gate_limits_armed),
        "enforced": bool(gate_limits_enforced()),
        "open_max_wait_ms": int(GATE_OPEN_LIMIT_MAX_WAIT_MS),
        "close_max_wait_ms": int(GATE_CLOSE_LIMIT_MAX_WAIT_MS),
        "find_max_ms": int(GATE_LIMIT_FIND_MAX_MS),
        "retry_interval_ms": int(GATE_LIMIT_RETRY_INTERVAL_MS),
        "motor1": {
            "close_pin": int(GATE_MOTOR1_CLOSE_LIMIT_PIN),
            "open_pin": int(GATE_MOTOR1_OPEN_LIMIT_PIN),
            "close_active": bool(gate_close_limit_active),
            "open_active": bool(gate_open_limit_active),
            "close_raw": bool(raw_m1_close),
            "open_raw": bool(raw_m1_open),
        },
        "motor2": {
            "close_pin": int(GATE_MOTOR2_CLOSE_LIMIT_PIN),
            "open_pin": int(GATE_MOTOR2_OPEN_LIMIT_PIN),
            "close_active": bool(gate_close_limit2_active),
            "open_active": bool(gate_open_limit2_active),
            "close_raw": bool(raw_m2_close),
            "open_raw": bool(raw_m2_open),
        },
        # Backward-compatible aggregate fields. A whole-gate endpoint is now
        # true only when BOTH sides confirm it.
        "close_pin": int(GATE_MOTOR1_CLOSE_LIMIT_PIN),
        "open_pin": int(GATE_MOTOR1_OPEN_LIMIT_PIN),
        "close_active": all_close,
        "open_active": all_open,
        "close_raw": bool(raw_m1_close and raw_m2_close),
        "open_raw": bool(raw_m1_open and raw_m2_open),
        "closed_pin": int(GATE_MOTOR1_CLOSE_LIMIT_PIN),
        "closed_active": all_close,
        "closed_raw": bool(raw_m1_close and raw_m2_close),
    }

def create_components():
    global shared_i2c, clock, tof, rfid_reader
    global matrix, stepper, solenoid, buzzer, ethernet

    # One shared I2C0 bus for DS3231 + both VL53L0X sensors.
    shared_i2c = make_shared_i2c(I2C_FREQUENCY)

    clock = DS3231Clock(
        RTC_SDA_PIN,
        RTC_SCL_PIN,
        address=DS3231_ADDRESS,
        frequency=I2C_FREQUENCY,
        force_set=FORCE_SET_RTC,
        initial_datetime=INITIAL_DATETIME,
        resync_ms=RTC_RESYNC_MS,
        i2c=shared_i2c,
    )

    tof = DualVL53L0XSafety(
        shared_i2c,
        TOF_XSHUT1_PIN,
        TOF_XSHUT2_PIN,
        sensor1_address=TOF_SENSOR1_ADDRESS,
        sensor2_address=TOF_SENSOR2_ADDRESS,
        default_address=TOF_DEFAULT_ADDRESS,
        sensor1_angle=TOF_SENSOR1_ANGLE,
        sensor2_angle=TOF_SENSOR2_ANGLE,
        io_timeout_ms=TOF_IO_TIMEOUT_MS,
        calibration_file=TOF_CALIBRATION_FILE,
        background_file=TOF_BACKGROUND_FILE,
    )
    tof.bus_recover_cb = recover_shared_i2c

    rfid_reader = RDM6300(
        RFID_UART_ID,
        RFID_RX_PIN,
        RFID_TX_PIN,
        RFID_BAUD,
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
        TB6600_1_STEP_PIN,
        TB6600_1_DIR_PIN,
        step2_pin=TB6600_2_STEP_PIN,
        dir2_pin=TB6600_2_DIR_PIN,
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
        motor2_opposite=TB6600_MOTOR2_OPPOSITE,
        max_pulses_per_update=TB6600_MAX_PULSES_PER_UPDATE,
        max_burst_us=TB6600_MAX_BURST_US,
        open_soft_land_enabled=TB6600_OPEN_SOFT_LAND_ENABLED,
        open_decel_start_percent=TB6600_OPEN_DECEL_START_PERCENT,
        open_final_delay_us=TB6600_OPEN_FINAL_DELAY_US,
        use_hardware_rmt=TB6600_USE_HARDWARE_RMT,
        rmt_channel1=TB6600_RMT_CHANNEL_1,
        rmt_channel2=TB6600_RMT_CHANNEL_2,
        rmt_resolution_hz=TB6600_RMT_RESOLUTION_HZ,
        rmt_chunk_pulses=TB6600_RMT_CHUNK_PULSES,
        rmt_chunk_max_us=TB6600_RMT_CHUNK_MAX_US,
        rmt_stall_timeout_ms=TB6600_RMT_STALL_TIMEOUT_MS,
    )

    solenoid = SolenoidRelay(
        SOLENOID_RELAY_PIN,
        active_low=SOLENOID_RELAY_ACTIVE_LOW,
    )

    buzzer = Buzzer(BUZZER_PIN, active_high=BUZZER_ACTIVE_HIGH,
                    buzzer_type=BUZZER_TYPE, volume_percent=BUZZER_VOLUME_PERCENT,
                    tone_hz=BUZZER_TONE_HZ, active_pwm_hz=BUZZER_ACTIVE_PWM_HZ)

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

def i2c_bus_clear():
    """v2.0.5: free a bus whose SDA is held low by a half-reset slave.

    Up to 9 SCL pulses with SDA released, then a STOP condition. Bounded,
    ~0.2 ms total. Safe to call any time no transfer is in progress.
    """
    try:
        sda = Pin(TOF_SDA_PIN, Pin.IN, Pin.PULL_UP)
        scl = Pin(TOF_SCL_PIN, Pin.OPEN_DRAIN, value=1)
        pulses = 0
        for _ in range(9):
            if sda.value():
                break
            scl.value(0)
            time.sleep_us(10)
            scl.value(1)
            time.sleep_us(10)
            pulses += 1
        # STOP: SDA low -> high while SCL high.
        sda_out = Pin(TOF_SDA_PIN, Pin.OPEN_DRAIN, value=0)
        time.sleep_us(10)
        scl.value(1)
        time.sleep_us(10)
        sda_out.value(1)
        time.sleep_us(10)
        return pulses
    except Exception as exc:
        print("I2C BUS CLEAR ERROR:", repr(exc))
        return -1


def make_shared_i2c(frequency):
    if I2C_USE_SOFTWARE and SoftI2C is not None:
        return SoftI2C(
            scl=Pin(TOF_SCL_PIN),
            sda=Pin(TOF_SDA_PIN),
            freq=int(frequency),
            timeout=int(I2C_SOFT_TIMEOUT_US),
        )
    return I2C(
        TOF_I2C_ID,
        sda=Pin(TOF_SDA_PIN),
        scl=Pin(TOF_SCL_PIN),
        freq=int(frequency),
    )


def recover_shared_i2c():
    """Called by the VL53L0X recovery sequence before re-addressing."""
    pulses = i2c_bus_clear()
    if pulses:
        print("I2C BUS CLEAR: SDA released after", pulses, "SCL pulse(s)")
    return rebuild_shared_i2c(i2c_frequency_active, "bus recovery", quiet=True)


def rebuild_shared_i2c(frequency, reason="", quiet=False):
    """Re-create I2C0 at another speed and re-point DS3231 + VL53L0X to it.

    Sensor addresses (0x30/0x31) are already assigned, so no XSHUT sequence is
    needed. Used for the 400 kHz -> 100 kHz fallback.
    """
    global shared_i2c, i2c_frequency_active

    try:
        new_i2c = make_shared_i2c(frequency)
    except Exception as exc:
        print("I2C REBUILD FAILED:", repr(exc))
        return False

    shared_i2c = new_i2c
    i2c_frequency_active = int(frequency)
    try:
        if clock is not None:
            clock.i2c = new_i2c
    except Exception:
        pass
    try:
        if tof is not None:
            tof.rebind_i2c(new_i2c)
    except Exception:
        pass

    if not quiet:
        print("I2C0 NOW", int(frequency), "Hz", ("- " + reason) if reason else "")
    return True


def initialize_runtime_components():
    global manual_button
    manual_button = ManualOpenButton(MANUAL_OPEN_BUTTON_PIN,
        MANUAL_OPEN_BUTTON_HOLD_MS, MANUAL_OPEN_BUTTON_DEBOUNCE_MS,
        long_hold_ms=CONFIG_MODE_HOLD_MS)
    manual_button.initialize()
    """Initialize critical GPIO/device objects without blocking on gate homing.

    v1.9.8 intentionally does NOT home the gate here. The previous
    home_gate_to_close_on_boot() contained an unlimited while-loop, so one noisy
    or missing CLOSE switch prevented RTC, ToF, SD, Web, W5500, LED and the main
    loop from ever starting.

    Gate homing is now prepared after all services initialize and is then
    serviced cooperatively by service_boot_home_nonblocking().
    """
    rfid_reader.initialize()
    matrix.initialize()
    stepper.initialize(config["gate"]["closed_angle"])
    stepper.close_soft_land_enabled = bool(TB6600_CLOSE_SOFT_LAND_ENABLED)
    stepper.rmt_prequeue_enabled = bool(TB6600_RMT_PREQUEUE)
    stepper.rmt_chain_lead_us = max(500, int(TB6600_RMT_CHAIN_LEAD_US))
    stepper.open_accel_steps = max(10, int(TB6600_OPEN_ACCEL_STEPS))
    stepper.open_s_curve = bool(TB6600_OPEN_S_CURVE)
    stepper.close_decel_start_percent = max(10, min(90, int(TB6600_CLOSE_DECEL_START_PERCENT)))
    stepper.close_final_delay_us = max(int(TB6600_RUN_DELAY_US), int(TB6600_CLOSE_FINAL_DELAY_US))
    solenoid.initialize()
    buzzer.initialize()

    if not initialize_gate_limit_switches():
        # Keep the controller alive for diagnostics instead of trapping startup.
        print("BOOT WARNING: CLOSE/OPEN LIMIT SWITCH INITIALIZATION FAILED")
        return False

    return True

def _finish_boot_home_success():
    """Finalize boot homing after BOTH physical CLOSE limits confirm."""
    global gate_state, gate_busy, gate_busy_until, boot_home_complete
    global boot_home_attempt_deadline, boot_home_retry_at
    global boot_home_attempt_count

    if stepper is not None and stepper.moving:
        stepper.stop()

    stepper.reconcile_position(
        config["gate"]["closed_angle"],
        reason="boot BOTH CLOSE limits confirmed",
    )

    solenoid.force_locked()
    gate_state = "LOCKED"
    gate_busy = False
    gate_busy_until = 0
    boot_home_attempt_deadline = 0
    boot_home_retry_at = 0
    boot_home_attempt_count = 0
    boot_home_complete = True

    print()
    print("============================================================")
    print("BOOT HOME COMPLETE - BOTH CLOSE LIMITS CONFIRMED")
    print("============================================================")
    print("MOTOR #1 CLOSE : GPIO", GATE_MOTOR1_CLOSE_LIMIT_PIN, "ACTIVE")
    print("MOTOR #2 CLOSE : GPIO", GATE_MOTOR2_CLOSE_LIMIT_PIN, "ACTIVE")
    print("RFID           : MAY ARM WHEN VL53L0X = NO_PRESENCE")
    print("SYSTEM         : RESPONSIVE / NORMAL MAIN LOOP")
    print("============================================================")

    try:
        if matrix is not None and _presence_state() in ("NO_PRESENCE", "DISABLED"):
            matrix.start_standby()
    except Exception:
        pass

    return True


def prepare_boot_home_nonblocking():
    """Prepare boot homing without entering any blocking loop."""
    global gate_state, gate_busy, gate_busy_until, boot_home_complete
    global boot_home_attempt_deadline, boot_home_retry_at
    global boot_home_attempt_count, boot_home_last_status_at

    boot_home_complete = False
    boot_home_attempt_deadline = 0
    boot_home_retry_at = 0
    boot_home_attempt_count = 0
    boot_home_last_status_at = 0

    if not GATE_LIMIT_SWITCHES_ENABLED:
        if GATE_LIMIT_SWITCHES_REQUIRED:
            gate_state = "BOOT_HOME_ERROR"
            gate_busy = True
            solenoid.force_locked()
            print("BOOT HOME PENDING: LIMIT SWITCHES ARE REQUIRED")
            return False

        gate_state = "LOCKED"
        gate_busy = False
        boot_home_complete = True
        return True

    update_gate_limit_switches()

    if gate_limits_conflict():
        gate_state = "BOOT_HOME_WAIT_LIMITS"
        gate_busy = True
        solenoid.force_locked()
        print(
            "BOOT HOME PENDING: LIMIT CONFLICT -",
            gate_limit_conflict_detail(),
        )
        return False

    if _all_endpoint_limits_active("CLOSE"):
        return _finish_boot_home_success()

    # The main loop will start the first attempt immediately.
    gate_state = "BOOT_HOME_RETRY_WAIT"
    gate_busy = True
    gate_busy_until = 0
    boot_home_retry_at = time.ticks_ms()
    solenoid.force_locked()

    print()
    print("============================================================")
    print("BOOT HOME SCHEDULED - NON-BLOCKING")
    print("============================================================")
    print("MISSING CLOSE   :", _missing_endpoint_text("CLOSE"))
    print("ATTEMPT MAX     :", GATE_BOOT_HOME_MAX_MS, "ms")
    print("RETRY INTERVAL  :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms")
    print("SYSTEM SERVICES : STAY ALIVE WHILE HOMING")
    print("RFID            : BLOCKED UNTIL BOTH CLOSE LIMITS")
    print("============================================================")
    return True


def _boot_home_status(message):
    """Throttle boot-home status logs and keep a visible NOT-READY heartbeat."""
    global boot_home_last_status_at

    now = time.ticks_ms()
    if (
        boot_home_last_status_at
        and time.ticks_diff(now, boot_home_last_status_at)
        < int(BOOT_HOME_STATUS_INTERVAL_MS)
    ):
        return

    boot_home_last_status_at = now
    print("BOOT HOME:", message)

    # Red X is intentionally refreshed while gate reference is not ready.
    # This makes it obvious that the VM is alive instead of looking frozen.
    try:
        if matrix is not None:
            matrix.show_result(False)
    except Exception:
        pass


def service_boot_home_nonblocking():
    """Advance at most one small boot-home step and return immediately."""
    global gate_state, gate_busy, gate_busy_until, boot_home_complete
    global boot_home_attempt_deadline, boot_home_retry_at
    global boot_home_attempt_count

    if boot_home_complete:
        return True

    if not GATE_LIMIT_SWITCHES_ENABLED:
        return False

    update_gate_limit_switches()

    if gate_limits_conflict():
        if stepper is not None and stepper.moving:
            stepper.stop()

        solenoid.force_locked()
        gate_busy = True
        gate_state = "BOOT_HOME_WAIT_LIMITS"
        boot_home_attempt_deadline = 0
        boot_home_retry_at = ticks_after_ms(
            GATE_LIMIT_RETRY_INTERVAL_MS
        )

        _boot_home_status(
            "WAITING - LIMIT CONFLICT {}".format(
                gate_limit_conflict_detail()
            )
        )
        return False

    _confirm_reached_endpoints("CLOSE")

    if _all_endpoint_limits_active("CLOSE"):
        return _finish_boot_home_success()

    # Safety: if presence sensing is enabled, do not move toward CLOSE while a
    # person is detected or while the detector has not become ready yet.
    if config["tof"]["enabled"] and not (stepper is not None and stepper.moving):
        presence_state = _presence_state()

        if presence_state != "NO_PRESENCE":
            if stepper is not None and stepper.moving:
                stepper.stop()

            solenoid.force_locked()
            gate_busy = True
            gate_state = "BOOT_HOME_WAIT_CLEAR"
            boot_home_attempt_deadline = 0
            boot_home_retry_at = ticks_after_ms(
                GATE_LIMIT_RETRY_INTERVAL_MS
            )

            _boot_home_status(
                "WAITING FOR NO_PRESENCE - {}".format(
                    presence_state
                )
            )
            return False

    now = time.ticks_ms()

    # Active attempt.
    if stepper is not None and stepper.moving:
        solenoid.set_released(True)
        gate_busy = True
        gate_state = "BOOT_HOMING_CLOSE"

        if (
            boot_home_attempt_deadline
            and time.ticks_diff(
                now,
                boot_home_attempt_deadline,
            ) >= 0
        ):
            boot_home_attempt_count += 1
            boot_home_attempt_deadline = ticks_after_ms(GATE_BOOT_HOME_MAX_MS)
            print("BOOT CLOSE: NEXT 10000 ms WINDOW | MISSING", _missing_endpoint_text("CLOSE"))

        stepper.update()
        update_gate_limit_switches()
        _confirm_reached_endpoints("CLOSE")

        if _all_endpoint_limits_active("CLOSE"):
            return _finish_boot_home_success()

        return False

    # Rest period between attempts.
    if (
        boot_home_retry_at
        and time.ticks_diff(now, boot_home_retry_at) < 0
    ):
        gate_busy = True
        gate_state = "BOOT_HOME_RETRY_WAIT"
        solenoid.force_locked()
        return False

    # Start one bounded attempt for only the missing side(s).
    m1_move = not gate_close_limit_active
    m2_move = not gate_close_limit2_active

    if not m1_move and not m2_move:
        return _finish_boot_home_success()

    boot_home_attempt_count += 1
    boot_home_retry_at = 0
    boot_home_attempt_deadline = ticks_after_ms(
        GATE_BOOT_HOME_MAX_MS
    )

    print()
    print(
        "BOOT CLOSE FIND ATTEMPT #{} - MAX {} ms".format(
            boot_home_attempt_count,
            GATE_BOOT_HOME_MAX_MS,
        )
    )
    print(
        "MOTOR #1:",
        "MOVE CLOSE" if m1_move else "HOLD - CLOSE LIMIT ACTIVE",
    )
    print(
        "MOTOR #2:",
        "MOVE CLOSE" if m2_move else "HOLD - CLOSE LIMIT ACTIVE",
    )

    solenoid.set_released(True)
    gate_busy = True
    gate_state = "BOOT_HOMING_CLOSE"

    started = stepper.start_jog_relative(
        GATE_BOOT_HOME_MAX_DEGREES,
        movement="CLOSE",
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
        delay_us=GATE_BOOT_HOME_DELAY_US,
        motor1_enabled=m1_move,
        motor2_enabled=m2_move,
        force_bitbang=True,
        continuous_until_limit=True,
    )

    if not started:
        boot_home_attempt_deadline = 0
        boot_home_retry_at = ticks_after_ms(
            GATE_LIMIT_RETRY_INTERVAL_MS
        )
        gate_state = "BOOT_HOME_RETRY_WAIT"
        solenoid.force_locked()
        _boot_home_status(
            "MOVE START FAILED - RETRY SCHEDULED"
        )
        return False

    return False


def home_gate_to_close_on_boot():
    """Compatibility wrapper: boot homing is now cooperative/non-blocking."""
    return prepare_boot_home_nonblocking()

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
        time.sleep_ms(50)

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
        time.sleep_ms(20)

        sd = sdcard.SDCard(
            sd_spi,
            sd_cs,
            baudrate=SD_BAUDRATE,
        )
        print("SD CARD COMMUNICATION OK")

        if hasattr(os, "VfsFat"):
            filesystem = os.VfsFat(sd)
            os.mount(filesystem, SD_MOUNT)
        else:
            os.mount(sd, SD_MOUNT)

        # v1.9.8:
        # Do NOT perform a synchronous write/delete/os.sync self-test during
        # startup. A marginal card can hold the VFS in a write path and make the
        # controller look frozen before the main loop ever starts.
        #
        # A directory read is enough to verify that the filesystem mounted.
        os.listdir(SD_MOUNT)

        sd_ready = True
        make_directory(LOG_DIRECTORY)

        print("SD MOUNT/READ TEST: OK")
        print("SD BOOT WRITE TEST: SKIPPED - FREEZE HARDENING")
        print("MICRO SD READY")
        return True

    except Exception as e:
        print("SD INITIALIZATION FAILED:", repr(e))
        sd_ready = False
        return False


# ============================================================
# ACCESS LOG
# ============================================================

def sd_remount(reason=""):
    """Unmount + re-initialise the SD card (gate idle only)."""
    global sd_ready
    if motion_active():
        return False
    print("SD REMOUNT:", reason)
    try:
        os.umount(SD_MOUNT)
    except Exception:
        pass
    ok = initialize_sd()
    return bool(ok)


def log_access(card_id, status, dt):
    """Queue one access log record; never block the gate-start path on SD I/O."""
    if not sd_ready:
        print("LOG SKIPPED: SD NOT READY")
        return False

    try:
        record = (str(card_id), str(status), tuple(dt))
    except Exception:
        record = (str(card_id), str(status), dt)

    if len(pending_access_logs) >= ACCESS_LOG_QUEUE_MAX:
        # Drop the oldest record rather than allowing an unbounded queue to eat
        # heap if the SD card becomes unhealthy. Keep the newest access events.
        try:
            pending_access_logs.pop(0)
        except Exception:
            pass
        print("ACCESS LOG QUEUE FULL: OLDEST RECORD DROPPED")

    pending_access_logs.append(record)
    return True


def service_access_log_queue():
    """Flush at most one queued record while the physical gate is idle."""
    if not pending_access_logs or not sd_ready:
        return False

    if gate_busy or (stepper is not None and stepper.moving):
        return False

    card_id, status, dt = pending_access_logs[0]

    try:
        filename = "{}/{:04d}-{:02d}-{:02d}.csv".format(
            LOG_DIRECTORY, dt[0], dt[1], dt[2]
        )
        new_file = not file_exists(filename)
        with open(filename, "a") as f:
            if new_file:
                f.write("timestamp,rfid,status\n")
            f.write("{},{},{}\n".format(format_datetime(dt), card_id, status))

        # File close flushes Python/VFS buffers. Deliberately do NOT call
        # os.sync() per tap; a slow SD card must never stall access control.
        pending_access_logs.pop(0)
        return True
    except Exception as e:
        # v2.1.4: a missing log folder (SD wiped) is recreated once.
        if isinstance(e, OSError) and getattr(e, "errno", None) == 2 and not file_exists(LOG_DIRECTORY):
            make_directory(LOG_DIRECTORY)
            print("LOG FOLDER MISSING - RECREATED", LOG_DIRECTORY)
            return False
        # Drop the bad record after reporting once; retrying the same broken SD
        # operation every millisecond would create another apparent freeze.
        try:
            pending_access_logs.pop(0)
        except Exception:
            pass
        print("LOG ERROR:", repr(e))
        return False


# ============================================================
# v2.0.0 EXPLICIT TASK SCHEDULER
# ============================================================
#
# One place decides which expensive task may run right now:
#
#   MOTION (any motor stepping, shake, seek, seat, boot homing):
#       TB6600/limits/solenoid/LED/buzzer ........ ACTIVE
#       VL53L0X .................................. PAUSED (zero I2C)
#       RFID ..................................... PAUSED (UART not parsed)
#       W5500 link + web socket I/O ............... ACTIVE (light, no routing)
#       SD logs / sync processing / RTC NVS ...... DEFERRED
#
#   STATIONARY:
#       VL53L0X .................................. ACTIVE continuous
#       RFID ..................................... ACTIVE unless locked out
#       W5500 + web .............................. ACTIVE full
#       SD logs / sync ........................... when gate idle
#
#   RFID presence lockout:
#       PRESENCE (or sensor not ready) -> RFID disabled immediately.
#       A CLOSE cycle also arms the lockout.
#       Re-armed ONLY when gate LOCKED + idle + fresh NO_PRESENCE has been
#       continuous for tof.rfid_rearm_clear_ms (default 1000 ms).
# ============================================================

MOTION_GATE_STATES = (
    "BOOT_HOMING_CLOSE",
    "LATCH_RELEASE",
    "LATCH_SHAKE_PAUSE",
    "LATCH_SETTLE",
    "OPENING",
    "REOPENING",
    "OPEN_WAIT_MOTOR1",
    "OPEN_WAIT_MOTOR2",
    "OPEN_LIMIT_SEEK",
    "OPEN_LIMIT_RETRY",
    "OPEN_SEAT",
    "OPEN_SEAT_SETTLE",
    "CLOSING",
    "CLOSE_LIMIT_SEEK",
    "CLOSE_LIMIT_RETRY",
)

CLOSE_MOTION_STATES = (
    "CLOSING",
    "CLOSE_LIMIT_SEEK",
    "CLOSE_LIMIT_RETRY",
)


def motion_active():
    """True while the barrier is physically moving or between shake legs."""
    if stepper is not None and stepper.moving:
        return True
    return gate_state in MOTION_GATE_STATES


def _close_motion_guard_active():
    try:
        return bool(
            config["tof"].get("close_motion_guard", False)
            and gate_state in CLOSE_MOTION_STATES
        )
    except Exception:
        return False


def _rfid_rearm_ms():
    try:
        return int(config["tof"].get("rfid_rearm_clear_ms", RFID_REARM_CLEAR_MS))
    except Exception:
        return int(RFID_REARM_CLEAR_MS)


def _set_rfid_lockout(reason):
    global rfid_presence_lockout, rfid_lockout_reason, rfid_rearm_clear_since
    rfid_rearm_clear_since = 0
    if rfid_presence_lockout and rfid_lockout_reason == reason:
        return
    was_locked = rfid_presence_lockout
    rfid_presence_lockout = True
    rfid_lockout_reason = str(reason)
    if not was_locked:
        print("RFID LOCKOUT:", rfid_lockout_reason,
              "| RE-ARM: GATE LOCKED + NO_PRESENCE", _rfid_rearm_ms(), "ms")


def _update_rfid_lockout(moving):
    """Maintain the presence lockout and the 1-second re-arm qualification."""
    global rfid_presence_lockout, rfid_lockout_reason, rfid_rearm_clear_since

    tof_on = bool(config is not None and config["tof"]["enabled"])

    if tof_on:
        state = _presence_state()
        if state == "PRESENCE":
            _set_rfid_lockout("PRESENCE")
        elif state != "NO_PRESENCE":
            _set_rfid_lockout("PRESENCE_SENSOR_" + state)

    if moving and gate_state in CLOSE_MOTION_STATES:
        _set_rfid_lockout("CLOSE_CYCLE")

    if not rfid_presence_lockout:
        return

    closed_idle = bool(
        boot_home_complete
        and not moving
        and not gate_busy
        and gate_state == "LOCKED"
    )
    # While ToF is paused its data is not fresh, so this is naturally False.
    # v2.0.9: debounced idle state - a sensor glitch does not restart it.
    clear_now = bool((not tof_on) or _presence_allows_new_tap())

    if not (closed_idle and clear_now):
        rfid_rearm_clear_since = 0
        return

    now = time.ticks_ms()
    if not rfid_rearm_clear_since:
        rfid_rearm_clear_since = now
        return

    if time.ticks_diff(now, rfid_rearm_clear_since) >= _rfid_rearm_ms():
        rfid_presence_lockout = False
        print(
            "RFID RE-ARMED: GATE LOCKED + NO_PRESENCE FOR",
            _rfid_rearm_ms(),
            "ms | WAS:",
            rfid_lockout_reason,
        )
        rfid_lockout_reason = ""
        rfid_rearm_clear_since = 0


def apply_task_policy():
    """Pause/resume expensive tasks for the current gate phase. Cheap; call
    as often as wanted. Returns True while the barrier is in motion."""
    global task_motion_active, task_rfid_enabled, task_rfid_disable_reason
    global rfid_resume_purged_bytes

    global i2c_quiet_until, solenoid_last_released

    if config_mode_active:
        # v2.3.0: configuration mode owns ToF / RFID / LED; nothing resumes.
        if task_rfid_enabled:
            task_rfid_enabled = False
            print("RFID TASK: DISABLED - CONFIG_MODE")
        task_rfid_disable_reason = "CONFIG_MODE"
        if (tof is not None and tof.hardware_ready and not tof.paused
                and not config_tof_capture_until):
            tof.pause("CONFIG_MODE")
        return False

    moving = motion_active()

    # v2.0.5: every solenoid switch opens a short I2C quiet window.
    if solenoid is not None:
        released_now = bool(solenoid.released)
        if solenoid_last_released is not None and released_now != solenoid_last_released:
            i2c_quiet_until = ticks_after_ms(SOLENOID_I2C_QUIET_MS)
        solenoid_last_released = released_now
    quiet = bool(i2c_quiet_until and time.ticks_diff(time.ticks_ms(), i2c_quiet_until) < 0)

    if moving != task_motion_active:
        task_motion_active = moving
        if MOTION_GC_PAUSE:
            try:
                if moving:
                    gc.disable()
                else:
                    gc.enable()
                    gc.collect()
            except Exception:
                pass
        if moving:
            print(
                "TASK POLICY: MOTION | ToF PAUSED | RFID PAUSED |"
                " SD/SYNC+LED+WEB DEFERRED | GATE:", gate_state,
            )
        else:
            print(
                "TASK POLICY: STATIONARY | ToF ACTIVE | RFID PER LANE RULE |"
                " W5500+WEB FULL | GATE:", gate_state,
            )

    # ---------------- VL53L0X ----------------
    if tof is not None and tof.hardware_ready:
        try:
            if tof.set_gate_open(_exit_sensor_counts_now()):
                print("VL53L0X EXIT SENSOR S2:",
                      ("COUNTED (gate open)" if gate_state in EXIT_SENSOR_GATE_STATES
                       else "COUNTED (gate LOCKED - lane must be clear to tap)")
                      if tof.gate_open_context
                      else "IGNORED (gate moving / busy)")
        except Exception:
            pass

    if (
        tof is not None
        and config is not None
        and config["tof"]["enabled"]
        and tof.hardware_ready
    ):
        want_running = ((not moving) or _close_motion_guard_active()) and not quiet
        if want_running and tof.paused:
            tof.resume("STATIONARY" if not moving else "CLOSE_GUARD")
        elif not want_running and not tof.paused:
            tof.pause("SOLENOID_EMI_QUIET" if quiet else "MOTION")

    # ---------------- RFID ----------------
    _update_rfid_lockout(moving)

    if moving:
        enabled = False
        reason = "MOTION"
    elif rfid_presence_lockout:
        enabled = False
        reason = "LOCKOUT_" + str(rfid_lockout_reason)
    else:
        enabled = True
        reason = ""

    if enabled != task_rfid_enabled:
        task_rfid_enabled = enabled
        if enabled:
            purged = 0
            try:
                if rfid_reader is not None:
                    purged = rfid_reader.discard_pending(RFID_RESUME_DISCARD_BYTES)
            except Exception:
                purged = 0
            rfid_resume_purged_bytes = purged
            print("RFID TASK: ACTIVE | STALE UART BYTES PURGED:", purged)
        else:
            print("RFID TASK: DISABLED -", reason)
    task_rfid_disable_reason = reason

    if matrix is not None and not moving:
        try:
            matrix.set_hold_red(_led_hold_red_wanted())
        except Exception:
            pass

    return moving


def _deny_lockout_active():
    global rfid_deny_until
    if not rfid_deny_until:
        return False
    if time.ticks_diff(time.ticks_ms(), rfid_deny_until) < 0:
        return True
    rfid_deny_until = 0
    return False


def _led_hold_red_wanted():
    """v2.1.11: RED while the gate is idle LOCKED but taps are not allowed:
    presence (S1 or S2), sensor not ready, the 1 s clear re-arm, or the 1 s
    lockout after an invalid card. BLUE standby only when a tap is allowed."""
    if not boot_home_complete or gate_busy or gate_state != "LOCKED":
        return False
    if granted_passage_active or task_motion_active:
        return False
    return bool(rfid_presence_lockout or _deny_lockout_active())


def _rfid_lockout_feedback(card_id):
    """v2.0.2: a card tapped while RFID is locked out (lane not clear).

    NO database lookup and NO access decision - only RED X + warning beep so
    the user sees the gate is alive but blocked. The granted user's own card
    (still held at the reader while passing) is ignored silently.
    """
    global blocked_rfid_warn_card_id, blocked_rfid_warn_at
    global pending_rfid_card_id, pending_rfid_until

    # Keep a tap made during clear qualification, without granting while unsafe.
    # v2.1.11: off by default - the LED is red during that second = no taps.
    if (RFID_ACCEPT_TAP_DURING_REARM and boot_home_complete and not gate_busy and gate_state == "LOCKED"
        and _all_endpoint_limits_active("CLOSE") and _fresh_no_presence_now()):
        pending_rfid_card_id = str(card_id)
        pending_rfid_until = ticks_after_ms(RFID_PENDING_TAP_MS)
        return

    now = time.ticks_ms()
    if granted_passage_active and card_id == granted_passage_card_id:
        return
    if (
        card_id == last_granted_card_id
        and last_granted_at
        and time.ticks_diff(now, last_granted_at) < 15000
    ):
        return
    if (
        card_id == blocked_rfid_warn_card_id
        and blocked_rfid_warn_at
        and time.ticks_diff(now, blocked_rfid_warn_at) < TURNSTILE_WARN_COOLDOWN_MS
    ):
        return

    blocked_rfid_warn_card_id = card_id
    blocked_rfid_warn_at = now

    detail = ""
    try:
        if tof is not None and tof.sensor1 is not None:
            detail = " | S1={} S2={} mm".format(
                "-" if tof.sensor1.median_mm is None else int(tof.sensor1.median_mm),
                "-" if tof.sensor2.median_mm is None else int(tof.sensor2.median_mm),
            )
    except Exception:
        pass
    print("RFID TAP REJECTED - LANE NOT CLEAR ({}){} | {}".format(
        rfid_lockout_reason or task_rfid_disable_reason, detail, card_id))

    if matrix is not None:
        matrix.show_result(False)
    _start_turnstile_warning("RFID TAP WHILE LANE NOT CLEAR", force=True)


MANUAL_OVERRIDE_OPENING_STATES = (
    "UNLOCKING", "REUNLOCKING", "LATCH_RELEASE", "LATCH_SHAKE_PAUSE",
    "LATCH_SETTLE", "OPENING", "REOPENING", "OPEN_WAIT_MOTOR1",
    "OPEN_WAIT_MOTOR2", "OPEN_LIMIT_SEEK", "OPEN_LIMIT_RETRY",
    "OPEN_RETRY_WAIT", "OPEN_SEAT", "OPEN_SEAT_SETTLE",
)


def _manual_reopen_now():
    """Reverse to OPEN from any non-latched state (closing, seek, retry, error)."""
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_solenoid_lock_at, gate_open_since
    global gate_open_limit_deadline, gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason
    global gate_close_limit_deadline, gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason
    global gate_close_slow_mode, gate_close_retry_wait_since

    if stepper is not None and stepper.moving:
        stepper.stop()
    solenoid.set_released(True)
    gate_solenoid_lock_at = 0
    gate_open_since = 0
    gate_busy = True
    gate_busy_until = 0
    gate_latch_settle_until = 0
    gate_open_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_close_limit_deadline = 0
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    gate_close_retry_reason = ""
    gate_close_slow_mode = False
    gate_close_retry_wait_since = 0
    _reset_latch_shake_runtime()
    gate_state = "REUNLOCKING"
    gate_motor_start_at = ticks_after_ms(MANUAL_OVERRIDE_REOPEN_DELAY_MS)
    _pre_motion_gc()
    print("STAFF OVERRIDE: SOLENOID RELEASED - REVERSING TO OPEN IN",
          MANUAL_OVERRIDE_REOPEN_DELAY_MS, "ms")
    return True


def manual_override_open():
    """v2.1.1 staff button: open NOW from any state, no validation at all."""
    global boot_home_complete, gate_open_since, presence_clear_close_at

    update_gate_limit_switches()
    previous = gate_state
    print()
    print("============================================================")
    print("STAFF MANUAL OVERRIDE OPEN | GPIO{} HELD {} ms + RELEASE | FROM {}".format(
        MANUAL_OPEN_BUTTON_PIN,
        manual_button.last_held_ms if manual_button is not None else MANUAL_OPEN_BUTTON_HOLD_MS,
        previous))
    print("LANE / LIMIT / RFID CHECKS: BYPASSED")
    print("============================================================")

    if not boot_home_complete:
        if stepper is not None and stepper.moving:
            stepper.stop()
        boot_home_complete = True
        print("STAFF OVERRIDE: BOOT HOMING SKIPPED - NEXT CLOSE FINDS THE CLOSE SWITCHES")

    if gate_state in MANUAL_OVERRIDE_OPENING_STATES:
        print("STAFF OVERRIDE: GATE ALREADY OPENING - CONTINUING")
    elif gate_state in ("OPEN", "WAIT_CLEAR"):
        gate_open_since = time.ticks_ms()
        presence_clear_close_at = 0
        print("STAFF OVERRIDE: GATE ALREADY OPEN - OPEN TIMER RESTARTED")
    elif (
        gate_state == "LOCKED"
        and not gate_busy
        and _all_endpoint_limits_active("CLOSE")
        and not gate_limits_conflict()
    ):
        # Latched closed: use the normal latch-release + open sequence.
        if not unlock_gate():
            _manual_reopen_now()
    else:
        # CLOSING, endpoint seek/retry, ERROR, boot homing, partly open ...
        _manual_reopen_now()

    _begin_granted_passage("MANUAL_GPIO19")
    if matrix is not None:
        matrix.show_result(True)
    try:
        log_access("MANUAL_GPIO19", "MANUAL_OPEN", clock.current_datetime())
    except Exception as exc:
        print("STAFF OVERRIDE LOG SKIPPED:", repr(exc))
    if config["gate"]["buzzer_enabled"]:
        buzzer.start_beep(config["gate"]["buzzer_grant_ms"])
    return True


def service_manual_button():
    """GPIO19 staff button (v2.3.1, release-based).

    ARMED     hold reached 1.5 s: chirp, "release now to open".
    SHORT     released after 1.5-4.99 s: staff override open (any gate state).
    TOO_SHORT released before 1.5 s: nothing.
    LONG      held 5 s: toggle machine configuration mode (never opens).
    Every press is logged with its hold time and result.
    """
    if manual_button is None:
        return False
    event = manual_button.update()
    if event is None:
        return False
    held = manual_button.last_held_ms
    if event == BUTTON_ARMED:
        if config_mode_active:
            return False
        print("STAFF BUTTON: HELD {} ms - RELEASE NOW TO OPEN"
              " (keep holding to {} ms for configuration mode)".format(
                  MANUAL_OPEN_BUTTON_HOLD_MS, CONFIG_MODE_HOLD_MS))
        if MANUAL_OPEN_ARMED_CHIRP_MS and buzzer is not None and not buzzer.running:
            buzzer.start_beep(MANUAL_OPEN_ARMED_CHIRP_MS)
        return False
    if event == BUTTON_LONG:
        print("STAFF BUTTON: HELD {} ms -> CONFIGURATION MODE {}".format(
            held, "LOCK" if config_mode_active else "UNLOCK"))
        if config_mode_active:
            return exit_config_mode("GPIO19 HOLD {} ms".format(CONFIG_MODE_HOLD_MS))
        return enter_config_mode("GPIO19 HOLD {} ms".format(CONFIG_MODE_HOLD_MS))
    if event == BUTTON_TOO_SHORT:
        print("STAFF BUTTON: RELEASED AFTER {} ms -> NO ACTION (open needs >= {} ms)".format(
            held, MANUAL_OPEN_BUTTON_HOLD_MS))
        return False
    if event == BUTTON_SHORT:
        if config_mode_active:
            print("STAFF BUTTON: RELEASED AFTER {} ms -> IGNORED - MACHINE CONFIGURATION"
                  " MODE (hold {} ms to lock first)".format(held, CONFIG_MODE_HOLD_MS))
            try:
                buzzer.start_pattern(*CONFIG_MODE_REFUSED_BEEPS)
            except Exception:
                pass
            return False
        print("STAFF BUTTON: RELEASED AFTER {} ms -> STAFF OPEN".format(held))
        return manual_override_open()
    return False


# ============================================================
# v2.3.0 MACHINE CONFIGURATION MODE
# ============================================================

def disable_access_point(quiet=False):
    """Wi-Fi AP + station radio OFF (default state since v2.3.0)."""
    global ap
    for getter in (wlan_ap_constant, wlan_sta_constant):
        try:
            iface = network.WLAN(getter())
            iface.active(False)
            if getter is wlan_ap_constant:
                ap = iface
        except Exception as exc:
            if not quiet:
                print("WIFI OFF WARNING:", repr(exc))
    if not quiet:
        print("WIFI AP: OFF")
    return True


def shutdown_web_server():
    global web_server, web_server_ready
    web_server_ready = False
    if web_server is not None:
        try:
            web_server.close()
        except Exception as exc:
            print("WEB SERVER CLOSE WARNING:", repr(exc))
    web_server = None
    print("MACHINE CONFIGURATION WEB: STOPPED")


def _config_mode_refusal_reason():
    if motion_active() or (stepper is not None and stepper.moving):
        return "GATE MOVING"
    if granted_passage_active:
        return "PASSAGE IN PROGRESS"
    if gate_state in EXIT_SENSOR_GATE_STATES or gate_state in ("UNLOCKING", "REUNLOCKING"):
        return "GATE OPEN ({})".format(gate_state)
    return ""


def enter_config_mode(reason=""):
    """Unlock: AP + Machine Configuration Web ON; ToF/RFID/motors OFF."""
    global config_mode_active, config_mode_since, config_mode_toggle_count
    global task_rfid_enabled, task_rfid_disable_reason, pending_rfid_card_id
    global config_tof_capture_until, config_tof_capture_started

    refusal = _config_mode_refusal_reason()
    if refusal:
        print("MACHINE CONFIGURATION MODE: UNLOCK REFUSED -", refusal)
        try:
            buzzer.start_pattern(*CONFIG_MODE_REFUSED_BEEPS)
        except Exception:
            pass
        return False

    config_mode_active = True
    config_mode_since = time.ticks_ms()
    config_mode_toggle_count += 1
    config_tof_capture_until = 0
    config_tof_capture_started = 0

    # Motors / TB6600: stop and never command them while in this mode.
    if stepper is not None and stepper.moving:
        stepper.stop()
    if gate_busy or gate_state != "LOCKED":
        stop_gate_outputs()       # clears retries/timers; solenoid locked
    else:
        solenoid.force_locked()
    try:
        buzzer.stop()
    except Exception:
        pass

    # RFID: disabled; it must re-arm through the normal 1 s clear rule later.
    pending_rfid_card_id = ""
    _set_rfid_lockout("CONFIG_MODE")
    task_rfid_enabled = False
    task_rfid_disable_reason = "CONFIG_MODE"

    # VL53L0X: ranging stopped (no I2C traffic).
    try:
        if tof is not None and tof.hardware_ready and not tof.paused:
            tof.pause("CONFIG_MODE")
    except Exception:
        pass

    if matrix is not None:
        matrix.set_config_lock(True)

    print()
    print("============================================================")
    print("MACHINE CONFIGURATION MODE: UNLOCKED |", reason)
    print("============================================================")
    print("DISABLED : VL53L0X | RFID | NEMA17 + TB6600 | LED = RED X")
    print("LOCK BACK: HOLD GPIO{} FOR {} ms".format(MANUAL_OPEN_BUTTON_PIN, CONFIG_MODE_HOLD_MS))

    initialize_access_point()
    initialize_web_server()
    print("MACHINE CONFIGURATION WEB:", "READY" if web_server_ready else "FAILED",
          "| AP http://{}/".format(config["ap"]["ip"]))
    print("============================================================")

    # The AP start takes ~0.4 s; start the melody afterwards so it plays fully.
    try:
        buzzer.start_melody(CONFIG_MODE_UNLOCK_MELODY)
    except Exception as exc:
        print("CONFIG MELODY ERROR:", repr(exc))
    return True


def exit_config_mode(reason=""):
    """Lock back: AP + web OFF, re-seek both CLOSE switches, resume tasks."""
    global config_mode_active, config_mode_toggle_count
    global config_tof_capture_until, config_tof_capture_started

    shutdown_web_server()
    disable_access_point()

    config_mode_active = False
    config_mode_toggle_count += 1
    config_tof_capture_until = 0
    config_tof_capture_started = 0

    if matrix is not None:
        matrix.set_config_lock(False)

    # RFID stays locked out until GATE LOCKED + NO_PRESENCE for the re-arm time.
    # Card frames received while in configuration mode are thrown away.
    _set_rfid_lockout("CONFIG_MODE")
    try:
        if rfid_reader is not None:
            rfid_reader.discard_pending(RFID_RESUME_DISCARD_BYTES)
    except Exception:
        pass

    print()
    print("============================================================")
    print("MACHINE CONFIGURATION MODE: LOCKED |", reason)
    print("AP + WEB OFF | VL53L0X + RFID RESUME | CLOSE RE-SEEK (BOTH SWITCHES)")
    print("============================================================")

    # Re-seat the barrier with the boot-homing retry (waits for NO_PRESENCE,
    # bounded attempts, retries every GATE_LIMIT_RETRY_INTERVAL_MS).
    update_gate_limit_switches()
    _clear_endpoint_latch()
    prepare_boot_home_nonblocking()

    try:
        buzzer.start_melody(CONFIG_MODE_LOCK_MELODY)
    except Exception as exc:
        print("CONFIG MELODY ERROR:", repr(exc))
    return True


def _config_tof_capture_service():
    """Background capture asked for from the web while in configuration mode:
    run the VL53L0X just for the capture, then pause it again."""
    global config_tof_capture_until, config_tof_capture_started
    if not config_tof_capture_until or tof is None:
        return
    now = time.ticks_ms()
    done = False
    service_tof_recovery()
    service_tof_runtime(run_presence_flow=False)
    service_tof_persistence()
    if time.ticks_diff(now, config_tof_capture_started) >= 500:
        if tof.ready and not tof.background_save_pending:
            print("CONFIG MODE: VL53L0X BACKGROUND CAPTURE SAVED - VL53L0X PAUSED AGAIN")
            done = True
    if time.ticks_diff(now, config_tof_capture_until) >= 0:
        print("CONFIG MODE: VL53L0X BACKGROUND CAPTURE TIMEOUT - VL53L0X PAUSED AGAIN")
        done = True
    if done:
        config_tof_capture_until = 0
        config_tof_capture_started = 0
        try:
            tof.pause("CONFIG_MODE")
        except Exception:
            pass


def service_config_mode_pass():
    """One main-loop pass while in configuration mode (motors never run)."""
    global pending_reboot_at
    buzzer.update()
    # GPIO reads only: keeps the CLOSE/OPEN switch states current, so the
    # re-seek on exit knows if an arm was pushed off its switch meanwhile.
    update_gate_limit_switches()
    _config_tof_capture_service()
    flush_motion_prints()
    clock.update()
    service_access_log_queue()
    service_ethernet_link()
    service_web_server(moving=False)
    if pending_reboot_at is not None:
        if time.ticks_diff(time.ticks_ms(), pending_reboot_at) >= 0:
            if matrix is not None:
                matrix.clear()
            solenoid.force_locked()
            machine.reset()


def service_rfid_runtime():
    """Parse RFID only while the scheduler allows it.

    While the gate MOVES the UART is never read. While RFID is locked out by
    PRESENCE / the 1 s re-arm dwell, frames are still read (same bounded
    budget) only to give RED X feedback; they can never open the gate, and
    the UART is purged again when RFID re-arms.
    """
    global pending_rfid_card_id, pending_rfid_until
    if pending_rfid_card_id:
        if (time.ticks_diff(time.ticks_ms(), pending_rfid_until) >= 0
            or not _fresh_no_presence_now() or gate_busy or gate_state != "LOCKED"):
            pending_rfid_card_id = ""
        elif task_rfid_enabled and _rfid_lane_ready():
            card_id = pending_rfid_card_id
            pending_rfid_card_id = ""
            check_access(card_id)
            return 1
    if rfid_reader is None or config is None:
        return 0
    if task_rfid_enabled:
        return rfid_reader.update(check_access, config["tap"]["scan_interval_ms"])
    if (
        boot_home_complete
        and not task_motion_active
        and rfid_presence_lockout
        and config["tap"].get("lockout_tap_feedback", True)
    ):
        return rfid_reader.update(_rfid_lockout_feedback, config["tap"]["scan_interval_ms"])
    return 0


def _report_tof_error(exc, label):
    global tof_runtime_error_text, tof_runtime_error_print_at, tof_runtime_error_count

    now = time.ticks_ms()
    error_text = repr(exc)
    tof_runtime_error_count += 1
    if (
        error_text != tof_runtime_error_text
        or not tof_runtime_error_print_at
        or time.ticks_diff(now, tof_runtime_error_print_at) >= 1000
    ):
        print(label, error_text, "| count:", tof_runtime_error_count,
              "| OTHER SYSTEM FUNCTIONS CONTINUE")
        tof_runtime_error_text = error_text
        tof_runtime_error_print_at = now
        tof_runtime_error_count = 0


def service_tof_runtime(moving=False, force=False, run_presence_flow=True):
    """Stationary ToF service.

    tof.update() self-schedules its I2C polls (about 2 short polls per sample
    per sensor) so it is safe to call on every loop pass. The presence
    application flow is throttled to PRESENCE_FLOW_INTERVAL_MS.
    """
    global presence_flow_next_at

    if tof is None:
        return False

    try:
        if tof.running:
            tof.update()
    except Exception as exc:
        _report_tof_error(exc, "VL53L0X RUNTIME ERROR:")

    if not run_presence_flow:
        return True

    now = time.ticks_ms()
    if (
        not force
        and presence_flow_next_at
        and time.ticks_diff(now, presence_flow_next_at) < 0
    ):
        return True
    presence_flow_next_at = time.ticks_add(now, int(PRESENCE_FLOW_INTERVAL_MS))

    try:
        service_presence_flow()
        return True
    except Exception as exc:
        _report_tof_error(exc, "PRESENCE FLOW ERROR:")
        return False


def service_tof_recovery():
    """v2.0.1: re-address VL53L0X sensors that reset to 0x29 (supply dip /
    EMI during motor motion). Stationary only; one short step per call."""
    if tof is None or not tof.hardware_ready:
        return False
    if motion_active():
        return False
    if i2c_quiet_until and time.ticks_diff(time.ticks_ms(), i2c_quiet_until) < 0:
        return False
    try:
        return tof.service_recovery()
    except Exception as exc:
        _report_tof_error(exc, "VL53L0X RECOVERY ERROR:")
        return False


_pass_worst = ["", 0]   # v2.5.1: slowest service of the last stationary pass


def _pass_mark(name, t0):
    now = time.ticks_us()
    dt = time.ticks_diff(now, t0)
    if dt > _pass_worst[1]:
        _pass_worst[0] = name
        _pass_worst[1] = dt
    return now


def service_loop_monitor():
    """v2.0.1: report main-loop passes that take too long (lag diagnosis)."""
    global loop_prev_top_us, loop_max_gap_ms, loop_slow_count
    global loop_slow_print_at, loop_window_max_ms, loop_window_started

    now_us = time.ticks_us()
    if loop_prev_top_us:
        gap_ms = time.ticks_diff(now_us, loop_prev_top_us) // 1000
        if gap_ms > loop_window_max_ms:
            loop_window_max_ms = gap_ms
        if gap_ms > loop_max_gap_ms:
            loop_max_gap_ms = gap_ms
        if gap_ms >= int(LOOP_SLOW_WARN_MS):
            loop_slow_count += 1
            now_ms = time.ticks_ms()
            if (
                not loop_slow_print_at
                or time.ticks_diff(now_ms, loop_slow_print_at) >= int(LOOP_SLOW_PRINT_INTERVAL_MS)
            ):
                loop_slow_print_at = now_ms
                print(
                    "MAIN LOOP SLOW PASS:", gap_ms, "ms | count:", loop_slow_count,
                    "| gate:", gate_state,
                    "| web busy:", bool(web_server is not None and web_server.busy),
                    "| server:", tap_client.state,
                    "| slowest:", _pass_worst[0], _pass_worst[1] // 1000, "ms",
                )
    loop_prev_top_us = now_us
    _pass_worst[0] = ""
    _pass_worst[1] = 0

    now_ms = time.ticks_ms()
    if not loop_window_started:
        loop_window_started = now_ms
    elif time.ticks_diff(now_ms, loop_window_started) >= 10000:
        loop_window_started = now_ms
        loop_window_max_ms_last[0] = loop_window_max_ms
        loop_window_max_ms = 0


def service_tof_motion_safety(force=False):
    """During motion ToF is PAUSED. Only when the optional close_motion_guard
    is enabled is the sensor polled during CLOSE motion (anti-pinch)."""
    if tof is None or not tof.running:
        return False
    try:
        tof.update()
        return True
    except Exception as exc:
        _report_tof_error(exc, "VL53L0X MOTION-GUARD ERROR:")
        return False


def service_tof_persistence():
    """Write a newly captured background to flash only while the gate is idle."""
    if tof is None or not tof.background_save_pending:
        return False
    if gate_busy or motion_active():
        return False
    try:
        return tof.save_background_now()
    except Exception as exc:
        print("VL53L0X BACKGROUND SAVE ERROR:", repr(exc))
        return False


def service_i2c_health():
    """Drop the shared I2C from 400 kHz to 100 kHz on repeated bus errors."""
    global i2c_health_next_check_at

    if tof is None or i2c_frequency_active <= int(I2C_FALLBACK_FREQUENCY):
        return False

    now = time.ticks_ms()
    if i2c_health_next_check_at and time.ticks_diff(now, i2c_health_next_check_at) < 0:
        return False
    i2c_health_next_check_at = time.ticks_add(now, 1000)

    if motion_active():
        return False

    try:
        errors = tof.recent_i2c_errors()
    except Exception:
        errors = 0

    if errors < int(I2C_RUNTIME_ERROR_DOWNGRADE_COUNT):
        return False

    return rebuild_shared_i2c(
        I2C_FALLBACK_FREQUENCY,
        "{} I2C errors in 5 s at {} Hz".format(errors, i2c_frequency_active),
    )


def service_ethernet_link():
    """Cheap W5500 link monitor + v2.4.1 chip health check (self-healing)."""
    global eth_link_up, eth_link_next_check_at, eth_link_changes

    if ethernet is None:
        return
    # v2.4.1: detects a W5500 reset (motor/solenoid supply dip) and restores
    # its configuration at once - rate-limited inside (HEALTH_PERIOD_MS).
    ethernet.service()
    now = time.ticks_ms()
    if eth_link_next_check_at and time.ticks_diff(now, eth_link_next_check_at) < 0:
        return
    eth_link_next_check_at = time.ticks_add(now, int(ETH_LINK_CHECK_MS))

    up = bool(ethernet.ready and ethernet.is_connected())
    if up != eth_link_up:
        if eth_link_up is not None:
            eth_link_changes += 1
        eth_link_up = up
        print("W5500 LINK:", "UP" if up else "DOWN", "| changes:", eth_link_changes)


def task_status_payload():
    now = time.ticks_ms()
    rearm_remaining = 0
    if rfid_presence_lockout and rfid_rearm_clear_since:
        rearm_remaining = max(
            0, _rfid_rearm_ms() - time.ticks_diff(now, rfid_rearm_clear_since)
        )
    return {
        "motion_active": bool(task_motion_active),
        "tof_running": bool(tof is not None and tof.running),
        "tof_pause_reason": (tof.pause_reason if tof is not None else "UNAVAILABLE"),
        "tof_close_motion_guard": bool(config["tof"].get("close_motion_guard", False)),
        "rfid_enabled": bool(task_rfid_enabled),
        "rfid_disable_reason": task_rfid_disable_reason,
        "rfid_lockout": bool(rfid_presence_lockout),
        "idle_glitches_filtered": int(idle_glitch_count),
        "rfid_lockout_reason": rfid_lockout_reason,
        "rfid_rearm_clear_ms": _rfid_rearm_ms(),
        "rfid_rearm_remaining_ms": rearm_remaining,
        "rfid_resume_purged_bytes": rfid_resume_purged_bytes,
        "w5500_link": bool(eth_link_up),
        "w5500_link_changes": eth_link_changes,
        "web_dispatch_allowed": not bool(task_motion_active),
        "i2c_hz": i2c_frequency_active,
        "server_decision_pending": tap_decision is not None,
        "tof_recovery_needed": bool(tof is not None and tof.recovery_needed),
        "tof_recovery_count": (tof.recovery_count if tof is not None else 0),
        "loop_max_ms_last_10s": loop_window_max_ms_last[0],
        "loop_max_ms_since_boot": loop_max_gap_ms,
        "loop_slow_passes": loop_slow_count,
    }


# ============================================================
# SIMPLE BOOLEAN PRESENCE / RFID ACCESS FLOW
# ============================================================

def _presence_snapshot():
    """Full ToF status (dict). Used by Web/API only - NOT on hot paths."""
    if tof is None:
        return {
            "enabled": False,
            "ready": False,
            "data_valid": False,
            "fresh_pair": False,
            "global_state": "UNAVAILABLE",
            "presence_detected": False,
        }

    try:
        status = tof.status()
        if isinstance(status, dict):
            return status
    except Exception as e:
        return {
            "enabled": bool(config and config.get("tof", {}).get("enabled", False)),
            "ready": False,
            "data_valid": False,
            "fresh_pair": False,
            "global_state": "ERROR",
            "presence_detected": False,
            "error": repr(e),
        }

    return {
        "enabled": bool(config and config.get("tof", {}).get("enabled", False)),
        "ready": False,
        "data_valid": False,
        "fresh_pair": False,
        "global_state": "UNAVAILABLE",
        "presence_detected": False,
    }


def _exit_sensor_counts_now():
    """v2.1.10: S2 counts while the gate is OPEN (passage) AND while it is
    idle LOCKED. Live capture 04 Oct 2026: a person standing in the middle of
    the closed lane gave S2 3.5-8 MCPS at 300-380 mm (solid PRESENCE) while
    S1 saw nothing; the closed arm itself only returns ~0.3 MCPS, which the
    weak-echo filter already rejects. Ignoring S2 when LOCKED let people tap
    while the lane was not clear."""
    if gate_state in EXIT_SENSOR_GATE_STATES:
        return True
    return bool(TOF_EXIT_SENSOR_WHEN_LOCKED and gate_state == "LOCKED" and not gate_busy)


def _idle_lane_context():
    """Gate idle and LOCKED: no grant, not moving, nothing for safety to stop."""
    return bool(
        boot_home_complete
        and gate_state == "LOCKED"
        and not gate_busy
        and not granted_passage_active
        and not task_motion_active
    )


def _idle_presence_confirm_ms():
    try:
        return int(config["tof"].get("idle_presence_confirm_ms", 200))
    except Exception:
        return 200


def _presence_state():
    """v2.0.9: debounced lane state.

    While the gate is idle LOCKED a raw PRESENCE must persist for
    idle_presence_confirm_ms before it is reported. Shorter bursts are
    sensor glitches (seen on S1 as 1-sample 296-449 mm returns on an empty
    lane) and are reported as NO_PRESENCE, so they no longer lock RFID,
    restart the 1 s re-arm or sound the warning beep. In every other gate
    state (open, closing, passage) the raw state is returned instantly.
    """
    global idle_presence_since, idle_glitch_count, idle_glitch_log_at
    global idle_presence_confirmed

    raw = _presence_state_raw()
    if raw != "PRESENCE":
        if idle_presence_since and not idle_presence_confirmed:
            idle_glitch_count += 1
            now = time.ticks_ms()
            if not idle_glitch_log_at or time.ticks_diff(now, idle_glitch_log_at) >= 10000:
                idle_glitch_log_at = now
                print("VL53L0X IDLE GLITCH FILTERED: PRESENCE <", _idle_presence_confirm_ms(),
                      "ms ignored | total:", idle_glitch_count)
        idle_presence_since = 0
        idle_presence_confirmed = False
        return raw

    confirm_ms = _idle_presence_confirm_ms()
    if confirm_ms <= 0 or not _idle_lane_context():
        # Not idle: instant safety state. A presence already active here
        # counts as confirmed (never logged as a glitch).
        idle_presence_since = idle_presence_since or time.ticks_ms()
        idle_presence_confirmed = True
        return raw

    if idle_presence_confirmed:
        return "PRESENCE"
    now = time.ticks_ms()
    if not idle_presence_since:
        idle_presence_since = now
    if time.ticks_diff(now, idle_presence_since) >= confirm_ms:
        idle_presence_confirmed = True
        return "PRESENCE"
    return "NO_PRESENCE"


def _presence_state_raw():
    """Return DISABLED / NOT_READY / PRESENCE / NO_PRESENCE.

    v2.0.0: reads driver attributes directly (no status-dict build) so it is
    cheap enough for every loop pass. During a stale gap or a motion pause the
    last real result is retained.
    """
    if config is None or not config["tof"]["enabled"]:
        return "DISABLED"

    if tof is None or not tof.hardware_ready or not tof.ready:
        return "NOT_READY"

    # v2.0.1: sensors being re-addressed after a reset = no trustworthy data.
    if tof.recovery_needed:
        return "NOT_READY"

    state = str(tof.global_state)
    if state == "CLEAR":
        state = "NO_PRESENCE"
    if state not in ("PRESENCE", "NO_PRESENCE"):
        return "NOT_READY"
    return state


def _fresh_no_presence_now():
    """True only when BOTH sensors freshly confirm NO_PRESENCE right now."""
    if config is None or not config["tof"]["enabled"]:
        return True
    if tof is None:
        return False
    try:
        return bool(tof.fresh_clear_now())
    except Exception:
        return False


def _presence_allows_new_tap():
    """Fresh sensor data + (debounced) NO_PRESENCE right now.

    v2.0.0: the clear-dwell is now enforced continuously by the scheduler's
    RFID lockout (1000 ms), not lazily by the first card frame.
    v2.0.9: uses the idle-debounced state, so a 1-sample sensor glitch at
    the moment of the tap no longer rejects the card.
    """
    if config is None or not config["tof"]["enabled"]:
        return True
    if tof is None or not tof.ready or tof.paused or tof.recovery_needed:
        return False
    try:
        if not tof._fresh_pair(time.ticks_ms()):
            return False
    except Exception:
        return False
    return _presence_state() == "NO_PRESENCE"


def _rfid_lane_ready():
    """True only when the complete lane is ready for a NEW transaction."""
    if config is None:
        return False

    if not task_rfid_enabled:
        return False

    if granted_passage_active or gate_busy or gate_state != "LOCKED":
        return False

    if GATE_LIMIT_SWITCHES_ENABLED and gate_limits_conflict():
        return False

    if GATE_LIMIT_SWITCHES_REQUIRED and not _all_endpoint_limits_active("CLOSE"):
        return False

    if anti_tailgate_until and time.ticks_diff(
        time.ticks_ms(),
        anti_tailgate_until,
    ) < 0:
        return False

    return _presence_allows_new_tap()


def _exit_sensor_enabled():
    try:
        return bool(
            tof is not None and tof.sensor2 is not None
            and tof.sensor2.role == "exit"
            and config["tof"].get("exit_confirm_required", True)
        )
    except Exception:
        return False


def _clear_close_delay_ms():
    """Return the NO_PRESENCE -> CLOSE delay.

    v2.0.7: if someone entered (S1) but the exit beam (S2) never confirmed
    them leaving, they may be standing in the blind zone between the beams,
    so the longer unconfirmed-exit delay is used.
    """
    if granted_passage_active and not granted_person_seen:
        return GATE_UNUSED_OPEN_CLOSE_MS
    # v2.1.12: one fixed clear->close delay (2 s) for every clear lane.
    if GATE_CLEAR_CLOSE_MS:
        return int(GATE_CLEAR_CLOSE_MS)
    if (
        granted_passage_active
        and granted_person_seen
        and not passage_exit_confirmed
        and _exit_sensor_enabled()
    ):
        try:
            return int(config["tof"].get("unconfirmed_exit_close_delay_ms", 3000))
        except Exception:
            return 3000
    try:
        return int(
            config["tof"].get(
                "clear_close_delay_ms",
                PRESENCE_CLEAR_CLOSE_DELAY_MS,
            )
        )
    except Exception:
        return int(PRESENCE_CLEAR_CLOSE_DELAY_MS)


def _request_auto_close_if_clear(reason=""):
    """Request delayed close from an invalid/blocked RFID while clear.

    The request is remembered during OPENING, but the close timer itself never
    starts until BOTH OPEN endpoints are confirmed and gate_state == OPEN.
    """
    global auto_close_requested, auto_close_reason

    if config is None:
        return False

    if not config["tof"].get("invalid_rfid_close_enabled", True):
        return False

    if gate_state == "LOCKED":
        return False

    if not _fresh_no_presence_now():
        return False

    auto_close_requested = True
    auto_close_reason = str(reason or "RFID_INVALID_WHILE_CLEAR")

    print(
        "AUTO CLOSE REQUESTED:",
        auto_close_reason,
        "| FRESH NO_PRESENCE",
    )

    if gate_state == "OPEN":
        lock_gate()

    return True


def _start_turnstile_warning(reason="", force=False):
    """Start a short non-blocking fastlane warning cadence."""
    global turnstile_warn_last_at

    if buzzer is None or config is None or not config["gate"]["buzzer_enabled"]:
        return False

    now = time.ticks_ms()

    if (
        not force
        and turnstile_warn_last_at
        and time.ticks_diff(now, turnstile_warn_last_at) < TURNSTILE_WARN_COOLDOWN_MS
    ):
        return False

    buzzer.start_pattern(
        TURNSTILE_WARN_BEEPS,
        TURNSTILE_WARN_ON_MS,
        TURNSTILE_WARN_OFF_MS,
    )
    turnstile_warn_last_at = now

    if reason:
        print("TURNSTILE WARNING BEEP:", reason)

    return True


def _warn_blocked_rfid(card_id, reason):
    """RED X + warning cadence for a blocked NEW RFID."""
    global blocked_rfid_warn_card_id, blocked_rfid_warn_at

    now = time.ticks_ms()

    if (
        card_id == blocked_rfid_warn_card_id
        and blocked_rfid_warn_at
        and time.ticks_diff(now, blocked_rfid_warn_at) < TURNSTILE_WARN_COOLDOWN_MS
    ):
        return False

    blocked_rfid_warn_card_id = card_id
    blocked_rfid_warn_at = now

    print("RFID WARNING: BLOCKED DURING", reason, "|", card_id)

    if matrix is not None:
        matrix.show_result(False)

    _start_turnstile_warning(
        "RFID BLOCKED - " + str(reason)
    )
    return True


def _reset_passage_tracking():
    global passage_s1_seen, passage_s2_seen, passage_exit_confirmed
    global passage_s1_prev, passage_s2_prev, passage_tailgate_count
    global passage_exit_confirmed_at
    passage_s1_seen = False
    passage_s2_seen = False
    passage_exit_confirmed = False
    passage_s1_prev = False
    passage_s2_prev = False
    passage_tailgate_count = 0
    passage_exit_confirmed_at = 0


def _update_passage_tracking():
    """v2.0.7 entry -> exit sequence for the active grant.

    S1 (entry beam) sees the person first, S2 (exit beam, angled through the
    barrier) sees them last. EXIT CONFIRMED = S2 saw the person and then
    cleared. A new S1 presence AFTER an exit was confirmed means another
    person is following (tailgating): warn and require a new exit.
    """
    global passage_s1_seen, passage_s2_seen, passage_exit_confirmed
    global passage_s1_prev, passage_s2_prev, passage_tailgate_count
    global passage_exit_confirmed_at

    if not granted_passage_active or tof is None or tof.sensor1 is None:
        return
    if not _exit_sensor_enabled() or gate_state not in EXIT_SENSOR_GATE_STATES:
        return

    now = time.ticks_ms()
    s1 = bool(tof.sensor1.effective_present(now))
    s2 = bool(tof.sensor2.effective_present(now))

    if s1 and not passage_s1_prev:
        if passage_exit_confirmed:
            passage_tailgate_count += 1
            passage_exit_confirmed = False
            passage_s2_seen = False
            print("TAILGATE SUSPECTED: NEW PERSON ON ENTRY BEAM AFTER EXIT (#{})".format(
                passage_tailgate_count))
            print("  CLOSE NOW WAITS FOR THIS PERSON TO EXIT VIA S2")
            _start_turnstile_warning("TAILGATE - SECOND PERSON ENTERING", force=True)
        elif not passage_s1_seen:
            print("PASSAGE: ENTERING (S1 entry beam)")
        passage_s1_seen = True

    if s2 and not passage_s2_prev:
        if not passage_s2_seen:
            print("PASSAGE: CROSSING BARRIER (S2 exit beam)")
        passage_s2_seen = True

    if passage_s2_seen and not s2 and passage_s2_prev and not passage_exit_confirmed:
        passage_exit_confirmed = True
        passage_exit_confirmed_at = now
        print("PASSAGE: EXIT CONFIRMED (S2 cleared) -> NORMAL CLOSE DELAY")

    passage_s1_prev = s1
    passage_s2_prev = s2


def _begin_granted_passage(card_id):
    """Own the lane until BOTH physical CLOSE limits confirm."""
    global granted_passage_active, granted_passage_card_id
    global granted_person_seen, presence_clear_close_at, rfid_clear_ready_since
    global auto_close_requested, auto_close_reason

    _reset_passage_tracking()
    granted_passage_active = True
    granted_passage_card_id = str(card_id)
    granted_person_seen = False
    presence_clear_close_at = 0
    auto_close_requested = False
    auto_close_reason = ""
    rfid_clear_ready_since = 0

    print()
    print("PRESENCE FLOW: GRANTED SESSION START")
    print("RFID             :", granted_passage_card_id)
    print("PRE-ENTRY STATE  : NO_PRESENCE is expected before person enters")
    print("NEXT RFID        : BLOCKED UNTIL BOTH CLOSE LIMITS CONFIRM")
    print("CLEAR CLOSE DELAY:", PRESENCE_CLEAR_CLOSE_DELAY_MS, "ms")
    return True


def _end_granted_passage(reason=""):
    global granted_passage_active, granted_passage_card_id
    global granted_person_seen, presence_clear_close_at, rfid_clear_ready_since
    global auto_close_requested, auto_close_reason

    was_active = granted_passage_active
    old_card_id = granted_passage_card_id
    _reset_passage_tracking()

    granted_passage_active = False
    granted_passage_card_id = ""
    granted_person_seen = False
    presence_clear_close_at = 0
    auto_close_requested = False
    auto_close_reason = ""
    rfid_clear_ready_since = 0

    if was_active:
        print(
            "PRESENCE FLOW: GRANTED SESSION COMPLETE",
            old_card_id,
            reason,
        )

    return was_active


def _cancel_no_presence_close(reason=""):
    global presence_clear_close_at

    if not presence_clear_close_at:
        return False

    presence_clear_close_at = 0

    if reason:
        print("NO_PRESENCE CLOSE TIMER CANCELLED:", reason)

    return True


def _arm_no_presence_close(reason=""):
    """Arm the configured NO_PRESENCE -> CLOSE delay."""
    global presence_clear_close_at, presence_clear_close_delay_ms

    if presence_clear_close_at:
        return False

    delay_ms = _clear_close_delay_ms()
    presence_clear_close_at = ticks_after_ms(delay_ms)
    presence_clear_close_delay_ms = int(delay_ms)

    print()
    print("PRESENCE FLOW: NO_PRESENCE CLOSE ARMED")
    print("CLOSE ARMED IN   :", delay_ms, "ms")
    print("RFID             : BLOCKED UNTIL BOTH CLOSE LIMITS")
    print("FINAL CLOSE RULE : timer + NO_PRESENCE safety")

    if reason:
        print("REASON           :", reason)

    return True


def service_presence_flow():
    """Simple PRESENCE / NO_PRESENCE turnstile workflow.

    v1.9.5:
    lock_gate() exclusively owns the clear-delay countdown. This removes the
    old double-timer path.

    A normal granted passage closes after:
        GRANTED -> PRESENCE -> NO_PRESENCE -> gate fully OPEN -> delay -> CLOSE

    A blocked/different RFID may also request close when the passage is freshly
    NO_PRESENCE. Unused openings close after two seconds of fresh NO_PRESENCE.
    """
    global presence_last_state, presence_state_changed_at
    global granted_person_seen, presence_clear_close_at, rfid_clear_ready_since

    if config is None:
        return

    state = _presence_state()
    now = time.ticks_ms()
    previous = presence_last_state

    _update_passage_tracking()

    if state == "DISABLED":
        presence_last_state = state
        presence_clear_close_at = 0
        rfid_clear_ready_since = 0
        return

    if state == "NOT_READY":
        if state != previous:
            presence_last_state = state
            presence_state_changed_at = now
            print(
                "PRESENCE FLOW STATE:",
                previous if previous is not None else "INITIAL",
                "-> NOT_READY | SENSOR NOT INITIALIZED/READY",
                "| GRANTED:",
                granted_passage_active,
                "| GATE:",
                gate_state,
            )

        rfid_clear_ready_since = 0
        _cancel_no_presence_close("SENSOR NOT READY")
        return

    state_changed = state != previous

    if state_changed:
        presence_last_state = state
        presence_state_changed_at = now
        print(
            "PRESENCE FLOW STATE:",
            previous if previous is not None else "INITIAL",
            "->",
            state,
            "| GRANTED:",
            granted_passage_active,
            "| GATE:",
            gate_state,
        )

    if state == "PRESENCE":
        rfid_clear_ready_since = 0
        _cancel_no_presence_close("PRESENCE DETECTED")

        if granted_passage_active:
            if not granted_person_seen:
                granted_person_seen = True
                print(
                    "GRANTED + PRESENCE:"
                    " AUTHORIZED USER IS PASSING / RFID STILL BLOCKED"
                )
        elif state_changed and previous == "NO_PRESENCE":
            print(
                "PRESENCE WITHOUT ACTIVE GRANT"
                " - NEW RFID REMAINS BLOCKED"
            )
            _start_turnstile_warning(
                "PRESENCE WITHOUT ACTIVE GRANT"
            )
        return

    # NO_PRESENCE
    if (
        not granted_passage_active
        and gate_state == "LOCKED"
        and _all_endpoint_limits_active("CLOSE")
    ):
        if state_changed:
            print(
                "NO_PRESENCE + BOTH CLOSE: LED STANDBY"
                " / RFID RE-ARMS AFTER {} ms CONTINUOUS CLEAR".format(
                    _rfid_rearm_ms()
                )
            )
            if matrix is not None:
                matrix.start_standby()

        presence_clear_close_at = 0
        return

    # Do not start CLOSE timing while the motors are still opening/seeking.
    if gate_state != "OPEN":
        return

    normal_passage_clear = bool(
        granted_passage_active
        and granted_person_seen
    )
    invalid_rfid_clear_close = bool(
        auto_close_requested
    )
    orphan_open_clear = bool(
        not granted_passage_active
    )

    # An unused opening also closes after two seconds of fresh clear readings.

    reason = (
        auto_close_reason
        if invalid_rfid_clear_close
        else
        (
            "AUTHORIZED PERSON EXITED (S2 CONFIRMED)"
            if passage_exit_confirmed or not _exit_sensor_enabled()
            else "EXIT NOT CONFIRMED BY S2 - LONGER {} ms CLEAR REQUIRED".format(
                _clear_close_delay_ms())
        )
        if normal_passage_clear
        else
        "OPEN GATE HAS NO ACTIVE GRANT"
    )

    if not presence_clear_close_at:
        print(
            "NO_PRESENCE: REQUESTING AUTOMATIC CLOSE |",
            reason,
        )

    # lock_gate() arms/serves exactly one non-blocking timer.
    lock_gate()




# ============================================================
# GATE APPLICATION LOGIC
# ============================================================

def _reset_latch_shake_runtime():
    global gate_latch_shake_leg
    global gate_latch_shake_total_legs
    global gate_latch_shake_pause_until
    global gate_latch_single_open

    gate_latch_single_open = False
    gate_latch_shake_leg = 0
    gate_latch_shake_total_legs = 0
    gate_latch_shake_pause_until = 0


def _update_open_peer_wait_state():
    """Expose the asymmetric OPEN condition without allowing early CLOSE.

    The motor whose own OPEN limit is already active has already been stopped by
    _confirm_reached_endpoints("OPEN"). The other motor remains enabled and keeps
    moving toward its own OPEN limit. The whole gate remains busy, the solenoid
    stays released, and NO OPEN-hold/CLOSE timer is started here.
    """
    global gate_state

    m1_open = bool(gate_open_limit_active)
    m2_open = bool(gate_open_limit2_active)

    if m1_open and not m2_open:
        new_state = "OPEN_WAIT_MOTOR2"
        reached_motor = 1
        waiting_motor = 2
        waiting_pin = GATE_MOTOR2_OPEN_LIMIT_PIN
    elif m2_open and not m1_open:
        new_state = "OPEN_WAIT_MOTOR1"
        reached_motor = 2
        waiting_motor = 1
        waiting_pin = GATE_MOTOR1_OPEN_LIMIT_PIN
    else:
        return False

    if gate_state != new_state:
        gate_state = new_state
        print()
        print("============================================================")
        print("OPEN SYNC WAIT - ONE MOTOR ARRIVED FIRST")
        print("============================================================")
        print("MOTOR #{}      : OPEN LIMIT CONFIRMED -> 500 ms SLOW PUSH, THEN HOLD".format(
            reached_motor
        ))
        print("MOTOR #{}      : CONTINUE OPEN UNTIL GPIO{} CONFIRMS".format(
            waiting_motor, waiting_pin
        ))
        print("CLOSE PREP     : BLOCKED UNTIL BOTH OPEN LIMITS CONFIRM")
        print("OPEN HOLD TIMER: NOT STARTED")
        print("SOLENOID       : REMAINS RELEASED")
        print("============================================================")

    return True


def _start_open_seat():
    """Apply OPEN settle only after BOTH motors have confirmed OPEN."""
    global gate_state, gate_open_seat_settle_until, gate_solenoid_lock_at

    update_gate_limit_switches()
    _confirm_reached_endpoints("OPEN")

    if GATE_LIMIT_SWITCHES_REQUIRED and not _all_endpoint_limits_active("OPEN"):
        print("OPEN SEAT BLOCKED: WAITING FOR", _missing_endpoint_text("OPEN"))
        return _start_master_endpoint_seek("OPEN")

    if not GATE_OPEN_SEAT_ENABLED or not config["gate"]["motor_enabled"]:
        return _mark_gate_open()

    if stepper is None or stepper.moving:
        return False

    degrees = float(GATE_OPEN_SEAT_DEGREES)
    if degrees <= 0.0:
        return _mark_gate_open()

    # v1.7.16: each side owns an OPEN switch. Never push a motor farther after
    # its switch is active. Normally both are active here, so the configured
    # seat becomes a short mechanical settle rather than extra STEP pulses.
    seat_motor1 = not (gate_limits_enforced() and gate_open_limit_active)
    seat_motor2 = not (gate_limits_enforced() and gate_open_limit2_active)

    solenoid.set_released(True)
    gate_solenoid_lock_at = 0
    gate_open_seat_settle_until = 0

    print()
    print("============================================================")
    print("OPEN SOFT-SEAT / ANTI-BOUNCE PRELOAD")
    print("============================================================")
    print("SEAT DISTANCE :", degrees, "deg")
    print("SEAT DELAY    :", GATE_OPEN_SEAT_DELAY_US, "us half-pulse")
    print("MOTOR #1      :", "STEP" if seat_motor1 else "HOLD - OWN OPEN LIMIT ACTIVE")
    print("MOTOR #2      :", "STEP" if seat_motor2 else "HOLD - OWN OPEN LIMIT ACTIVE")
    print("SOLENOID      : REMAINS UNLOCKED")
    print("============================================================")

    started = stepper.start_jog_relative(
        degrees,
        movement="OPEN",
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
        delay_us=GATE_OPEN_SEAT_DELAY_US,
        motor1_enabled=seat_motor1,
        motor2_enabled=seat_motor2,
    )
    if not started:
        _gate_error(stepper.last_error or "TB6600 OPEN SEAT START FAILED")
        return False

    gate_state = "OPEN_SEAT"
    if not stepper.moving:
        return _finish_open_seat()
    return True

def _finish_open_seat():
    global gate_state, gate_open_seat_settle_until

    update_gate_limit_switches()
    _confirm_reached_endpoints("OPEN")

    if GATE_LIMIT_SWITCHES_REQUIRED and not _all_endpoint_limits_active("OPEN"):
        gate_open_seat_settle_until = 0
        print("OPEN SEAT FINALIZATION BLOCKED: WAITING FOR", _missing_endpoint_text("OPEN"))
        return _start_master_endpoint_seek("OPEN")

    # The extra movement is treated as preload against the OPEN stop, not as a
    # new logical angle. Re-anchor so normal CLOSE still uses the configured
    # OPEN/CLOSED coordinates.
    if stepper is not None and not stepper.moving:
        stepper.reconcile_position(
            config["gate"]["open_angle"],
            reason="OPEN soft-seat preload re-anchor",
        )

    solenoid.set_released(True)
    settle_ms = int(GATE_OPEN_SEAT_SETTLE_MS)

    if settle_ms <= 0:
        gate_open_seat_settle_until = 0
        return _mark_gate_open()

    gate_state = "OPEN_SEAT_SETTLE"
    gate_open_seat_settle_until = ticks_after_ms(settle_ms)
    print("OPEN SOFT-SEAT COMPLETE - HOLDING", settle_ms, "ms BEFORE OPEN STATE")
    return True


def _mark_gate_open(open_limit_confirmed=True, timed_out=False):
    global gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_open_since
    global gate_solenoid_lock_at, gate_open_seat_settle_until
    global gate_open_limit_deadline, gate_open_limit_confirmed, gate_open_limit_timed_out
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason

    # v1.7.19 non-negotiable dual-OPEN rule:
    # never enter OPEN and never start the close countdown until BOTH independent
    # OPEN limit switches have been debounced active.
    update_gate_limit_switches()
    _confirm_reached_endpoints("OPEN")
    if GATE_LIMIT_SWITCHES_REQUIRED and not _all_endpoint_limits_active("OPEN"):
        print("OPEN FINALIZATION BLOCKED: BOTH OPEN LIMITS ARE REQUIRED")
        print("STILL WAITING FOR:", _missing_endpoint_text("OPEN"))
        solenoid.set_released(True)
        gate_solenoid_lock_at = 0
        gate_busy_until = 0
        return _start_master_endpoint_seek("OPEN")

    open_limit_confirmed = True
    timed_out = False

    if stepper is not None and not stepper.moving:
        stepper.reconcile_position(
            config["gate"]["open_angle"],
            reason=(
                "BOTH OPEN limits confirmed"
                if open_limit_confirmed
                else "OPEN limit timeout fallback"
            ),
        )

    gate_motor_start_at = 0
    gate_latch_settle_until = 0
    gate_open_seat_settle_until = 0
    gate_open_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_open_limit_confirmed = bool(open_limit_confirmed)
    gate_open_limit_timed_out = bool(timed_out)
    _reset_latch_shake_runtime()

    # Normal OPEN is reached only after BOTH Motor #1 GPIO38 and Motor #2 GPIO42
    # have confirmed. The first-arriving motor may have been waiting stationary
    # while the late side completed OPEN.
    solenoid.set_released(True)
    if gate_open_limit_confirmed:
        gate_solenoid_lock_at = ticks_after_ms(SOLENOID_LOCK_AFTER_OPEN_DELAY_MS)
    else:
        gate_solenoid_lock_at = 0

    # The gate-open hold timer starts only NOW: after OPEN limit confirmation,
    # or after the explicit fixed endpoint-find timeout.
    gate_state = "OPEN"
    # v2.1.1: the close rules are served every pass from here on
    # (2 s no presence -> close, 3 s hard cap); no separate hold timer.
    gate_open_since = time.ticks_ms()
    gate_busy_until = 0

    if gate_open_limit_confirmed:
        print("GATE STATUS: BOTH OPEN LIMITS CONFIRMED @", config["gate"]["open_angle"], "deg")
        print("OPEN SYNC COMPLETE: BOTH MOTORS STOPPED AT THEIR OWN OPEN LIMITS")
        print("SOLENOID: BOTH OPEN LIMITS CONFIRMED - LOCK IN", SOLENOID_LOCK_AFTER_OPEN_DELAY_MS, "ms")
        print("CLOSE PREPARATION: BOTH SIDES READY")
        print("AUTO CLOSE RULE: {} ms NO_PRESENCE -> CLOSE | MAX OPEN {} ms (closes even if not clear)".format(
            GATE_UNUSED_OPEN_CLOSE_MS, GATE_OPEN_MAX_HOLD_MS))
    else:
        print("GATE STATUS: OPEN LIMIT TIMEOUT FALLBACK")
        print("SOLENOID: REMAINS UNLOCKED - OPEN LIMIT WAS NOT CONFIRMED")
        print("CLOSE TIMER: STARTED AFTER FIXED {} ms OPEN TIMEOUT".format(GATE_OPEN_LIMIT_MAX_WAIT_MS))

def _mark_gate_locked(from_close_limit=False, arm_anti_tailgate=None):
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_close_slow_mode, gate_close_retry_wait_since, gate_open_since
    global gate_solenoid_lock_at, gate_open_seat_settle_until
    global gate_close_limit_deadline, anti_tailgate_until
    global gate_open_limit_confirmed, gate_open_limit_timed_out
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason
    global gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason

    # v2.1.8: the endpoint latch belongs to ONE motion cycle. Once LOCKED it
    # must not survive, or a later idle CLOSE recovery "confirms" a switch
    # that is no longer pressed and loops (live log: Motor #2, card refused).
    _clear_endpoint_latch()

    if stepper is not None and not stepper.moving:
        stepper.reconcile_position(
            config["gate"]["closed_angle"],
            reason=(
                "BOTH CLOSE limits confirmed after closing"
                if from_close_limit
                else "CLOSE endpoint"
            ),
        )

    if solenoid is not None:
        solenoid.force_locked()

    gate_busy = False
    gate_busy_until = 0
    gate_motor_start_at = 0
    gate_latch_settle_until = 0
    gate_solenoid_lock_at = 0
    gate_open_seat_settle_until = 0
    gate_close_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    gate_close_retry_reason = ""
    gate_open_limit_confirmed = False
    gate_open_limit_timed_out = False
    _reset_latch_shake_runtime()
    gate_state = "LOCKED"
    gate_close_slow_mode = False
    gate_close_retry_wait_since = 0
    gate_open_since = 0

    # Anti-tailgating / next-user timer is ONLY armed after a real successful
    # user gate cycle closes on the physical CLOSE switch. An idle forced-open
    # recovery triggered by CLOSE_LIMIT_NOT_CONFIRMED must NOT penalize the user
    # with a new 5-second anti-tailgating delay because no access was granted.
    if arm_anti_tailgate is None:
        arm_anti_tailgate = bool(from_close_limit)

    # With live presence safety enabled, the lane itself is the next-user guard:
    # NO_PRESENCE + BOTH CLOSE immediately re-arms RFID. The old time-only delay
    # is kept only as a fallback when ToF is disabled.
    if from_close_limit and arm_anti_tailgate and not config["tof"]["enabled"]:
        delay_sec = max(2.0, float(config["tap"].get("next_card_delay_sec", 5.0)))
        anti_tailgate_until = ticks_after_ms(delay_sec * 1000)
        print("GATE STATUS: BOTH CLOSE LIMITS CONFIRMED / LOCKED")
        print("ANTI-TAILGATING NEXT-TAP DELAY:", delay_sec, "seconds")
    else:
        anti_tailgate_until = 0
        if from_close_limit:
            print("GATE STATUS: BOTH CLOSE LIMITS CONFIRMED / LOCKED")
            if config["tof"]["enabled"]:
                print("RFID RE-ARM RULE: NO_PRESENCE + BOTH CLOSE LIMITS")
            else:
                print("ANTI-TAILGATING DELAY: NOT ARMED - NO SUCCESSFUL USER CYCLE")
        else:
            print("GATE STATUS: LOCKED @", config["gate"]["closed_angle"], "deg")

    _end_granted_passage("BOTH CLOSE LIMITS CONFIRMED" if from_close_limit else "LOCKED")

    if (
        matrix is not None
        and _presence_state() in ("NO_PRESENCE", "DISABLED")
        and _all_endpoint_limits_active("CLOSE")
    ):
        matrix.start_standby()

def _gate_error(message):
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_solenoid_lock_at, gate_open_seat_settle_until
    global gate_open_limit_deadline, gate_close_limit_deadline
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason
    global gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason
    print("GATE ERROR:", message)
    stepper.stop()
    solenoid.force_locked()
    gate_busy = False
    gate_busy_until = 0
    gate_motor_start_at = 0
    gate_latch_settle_until = 0
    gate_solenoid_lock_at = 0
    gate_open_seat_settle_until = 0
    gate_open_limit_deadline = 0
    gate_close_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    gate_close_retry_reason = ""
    _reset_latch_shake_runtime()
    gate_state = "ERROR"
    _end_granted_passage("GATE ERROR")
    _gate_error_rehome_at[0] = ticks_after_ms(GATE_ERROR_REHOME_MS)
    print("GATE ERROR RECOVERY: RE-HOME TO CLOSE IN", GATE_ERROR_REHOME_MS, "ms")


_gate_error_rehome_at = [0]
_gate_error_rehome_count = [0]
_idle_close_missing_since = [0]
_idle_close_reseek_count = [0]


def _service_idle_close_guard():
    """v2.5.2: an idle LOCKED gate must sit on BOTH CLOSE limits. If one arm
    leaves its CLOSE switch (pushed, slipped, or stopped mid-way) for
    IDLE_CLOSE_RESEEK_MS, re-home it with the boot-home routine (moves only the
    missing motor, waits for NO_PRESENCE, retries until both CLOSE limits).
    Before v2.5.2 only an RFID tap started this recovery."""
    if (gate_state != "LOCKED" or gate_busy or config_mode_active
            or not boot_home_complete or not GATE_LIMIT_SWITCHES_ENABLED
            or (stepper is not None and stepper.moving)):
        _idle_close_missing_since[0] = 0
        return
    if _all_endpoint_limits_active("CLOSE") or gate_limits_conflict():
        _idle_close_missing_since[0] = 0
        return
    now = time.ticks_ms()
    if not _idle_close_missing_since[0]:
        _idle_close_missing_since[0] = now or 1
        return
    if time.ticks_diff(now, _idle_close_missing_since[0]) < int(IDLE_CLOSE_RESEEK_MS):
        return
    _idle_close_missing_since[0] = 0
    _idle_close_reseek_count[0] += 1
    print("IDLE CLOSE GUARD #{}: {} OFF ITS CLOSE LIMIT -> RE-HOMING".format(
        _idle_close_reseek_count[0], _missing_endpoint_text("CLOSE")))
    prepare_boot_home_nonblocking()


def _service_gate_error_recovery():
    """v2.5.1: ERROR is never final. After GATE_ERROR_REHOME_MS the gate is
    re-homed to CLOSE with the boot-home routine (it waits for NO_PRESENCE and
    for a limit conflict to clear, and retries until both CLOSE limits)."""
    if gate_state != "ERROR" or config_mode_active:
        return
    if stepper is not None and stepper.moving:
        return
    if not _gate_error_rehome_at[0]:
        _gate_error_rehome_at[0] = ticks_after_ms(GATE_ERROR_REHOME_MS)
        return
    if time.ticks_diff(time.ticks_ms(), _gate_error_rehome_at[0]) < 0:
        return
    _gate_error_rehome_at[0] = 0
    _gate_error_rehome_count[0] += 1
    print("GATE ERROR RECOVERY #{}: RE-HOMING TO BOTH CLOSE LIMITS".format(_gate_error_rehome_count[0]))
    prepare_boot_home_nonblocking()

def _safe_to_close():
    if tof is None:
        return True
    return tof.safe_to_close()


def _tof_motion_guard_live():
    """Obstruction-reopen during CLOSE motion is evaluated only when the
    VL53L0X is actually being polled (tof.close_motion_guard=True). With the
    default strict policy ToF is paused while moving, so retained data must
    never trigger a false reopen."""
    return bool(tof is not None and tof.running)


def _wait_for_clear():
    global gate_busy, gate_busy_until, gate_state
    gate_busy = True
    gate_busy_until = 0
    if gate_state != "WAIT_CLEAR":
        print("GATE CLOSE WAIT: VL53L0X PASSAGE NOT CLEAR")
    gate_state = "WAIT_CLEAR"


def _expected_master_limit_active(movement):
    # Compatibility helper retained for older callers. Whole-gate authority now
    # requires BOTH matching endpoint switches.
    return _all_endpoint_limits_active(movement)


def _confirm_motor_endpoint(motor_no, movement):
    """Stop one motor on its own physical endpoint and anchor its coordinate."""
    movement = str(movement).upper()
    angle = (
        config["gate"]["open_angle"]
        if movement == "OPEN"
        else config["gate"]["closed_angle"]
    )
    pin_no = _motor_limit_pin(motor_no, movement)
    reason = "Motor #{} {} limit GPIO{}".format(motor_no, movement, pin_no)
    _latch_endpoint(motor_no, movement)

    if (movement == "OPEN" and stepper.moving and gate_state in
        ("OPENING", "REOPENING", "OPEN_WAIT_MOTOR1", "OPEN_WAIT_MOTOR2",
         "OPEN_LIMIT_SEEK", "OPEN_LIMIT_RETRY")):
        if stepper.preload_motor_open(motor_no, angle, GATE_OPEN_PUSH_MS, GATE_OPEN_PUSH_DELAY_US):
            return True
    if (movement == "CLOSE" and GATE_CLOSE_PUSH_MS and stepper.moving and gate_state in
        ("CLOSING", "CLOSE_LIMIT_SEEK", "CLOSE_LIMIT_RETRY")):
        if stepper.preload_motor_open(motor_no, angle, GATE_CLOSE_PUSH_MS, GATE_CLOSE_PUSH_DELAY_US):
            return True

    if motor_no == 1:
        changed = stepper.confirm_motor1_endpoint(angle, reason)
    else:
        changed = stepper.confirm_motor2_endpoint(angle, reason)

    if changed:
        print(
            "GATE {} ENDPOINT: MOTOR #{} LIMIT CONFIRMED - OTHER SIDE MAY CONTINUE".format(
                movement, motor_no
            )
        )
    return True


def _confirm_reached_endpoints(movement):
    movement = str(movement).upper()
    stepper.service_open_preloads()
    if _motor_limit_active(1, movement):
        _confirm_motor_endpoint(1, movement)
    if _motor_limit_active(2, movement):
        _confirm_motor_endpoint(2, movement)
    return _all_endpoint_limits_active(movement)


def _confirm_master_endpoint(movement):
    # Backward-compatible name from v1.7.12-v1.7.15: now means Motor #1 only.
    return _confirm_motor_endpoint(1, movement)


def _confirm_both_endpoint(movement):
    """Finalize an endpoint only after both side-specific limits are active."""
    movement = str(movement).upper()
    if not _all_endpoint_limits_active(movement):
        return False

    _confirm_motor_endpoint(1, movement)
    _confirm_motor_endpoint(2, movement)
    if stepper.moving:
        stepper.stop()

    angle = (
        config["gate"]["open_angle"]
        if movement == "OPEN"
        else config["gate"]["closed_angle"]
    )
    stepper.reconcile_position(
        angle,
        reason="BOTH {} limits confirmed (M1 GPIO{}, M2 GPIO{})".format(
            movement,
            _motor_limit_pin(1, movement),
            _motor_limit_pin(2, movement),
        ),
    )
    print("GATE {} ENDPOINT: BOTH MOTOR LIMITS CONFIRMED".format(movement))
    return True


def _start_fast_close_recovery_jog(
    max_degrees,
    motor1_enabled,
    motor2_enabled,
    reason,
    slow=False,
):
    """Run CLOSE endpoint recovery at the normal fast CLOSE speed.

    The old recovery path intentionally used:
        fixed 4000 us half-pulse + force_bitbang=True

    That made a missing-side retry crawl. v1.9.6 instead:
      1. stops any previous move;
      2. recreates RMT for a clean movement;
      3. uses the normal CLOSE acceleration/run profile;
      4. moves only the side(s) whose CLOSE limit is still missing.

    Endpoint callbacks still stop each motor independently as soon as its own
    CLOSE switch becomes active.
    """
    if stepper is None:
        return False

    if stepper.moving:
        stepper.stop()

    if GATE_CLOSE_RECOVERY_USE_RMT:
        if not stepper.rearm_rmt_for_new_motion(
            str(reason or "FAST_CLOSE_RECOVERY")
        ):
            return False

    started = stepper.start_jog_relative(
        max_degrees,
        movement="CLOSE",
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
        delay_us=(GATE_CLOSE_SLOW_RETRY_DELAY_US if slow else GATE_CLOSE_LIMIT_SEEK_DELAY_US),
        motor1_enabled=bool(motor1_enabled),
        motor2_enabled=bool(motor2_enabled),
        force_bitbang=not bool(GATE_CLOSE_RECOVERY_USE_RMT),
        normal_speed_profile=bool(
            GATE_CLOSE_RECOVERY_USE_NORMAL_SPEED and not slow
        ),
        continuous_until_limit=True,
    )

    if started:
        print(
            "CLOSE RECOVERY SPEED:",
            "SLOW SMOOTH {} us half-pulse (blocked-close retry)".format(GATE_CLOSE_SLOW_RETRY_DELAY_US)
            if slow else "FULL NORMAL CLOSE SPEED",
        )
        print(
            "CLOSE RECOVERY ENGINE:",
            stepper.status().get(
                "pulse_engine",
                "UNKNOWN",
            ),
        )

    return started


def _start_fast_open_recovery_jog(
    max_degrees,
    motor1_enabled,
    motor2_enabled,
    reason,
):
    """v2.0.6: OPEN endpoint seek/retry at the normal OPEN speed.

    Mirrors the v1.9.6 CLOSE recovery: clean RMT re-arm, normal
    acceleration (TB6600_START_DELAY_US -> TB6600_RUN_DELAY_US) and only the
    side(s) whose own OPEN limit is still missing move. Each motor still stops
    the instant its own OPEN switch becomes active. Previously this used a
    fixed 3200 us half-pulse bit-bang crawl, which looked like the gate was
    randomly opening in slow motion.
    """
    if stepper is None:
        return False

    if stepper.moving:
        stepper.stop()

    use_rmt = bool(GATE_OPEN_RECOVERY_USE_RMT)
    if use_rmt:
        if not stepper.rearm_rmt_for_new_motion(str(reason or "FAST_OPEN_RECOVERY")):
            use_rmt = False

    started = stepper.start_jog_relative(
        max_degrees,
        movement="OPEN",
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
        delay_us=GATE_OPEN_LIMIT_SEEK_DELAY_US,
        motor1_enabled=bool(motor1_enabled),
        motor2_enabled=bool(motor2_enabled),
        force_bitbang=not use_rmt,
        normal_speed_profile=bool(GATE_OPEN_RECOVERY_USE_NORMAL_SPEED),
        ease_out_fast_ms=(GATE_OPEN_RECOVERY_FAST_MS if GATE_OPEN_RECOVERY_EASE_OUT else 0),
        ease_out_slow_ms=(GATE_OPEN_RECOVERY_SLOW_MS if GATE_OPEN_RECOVERY_EASE_OUT else 0),
        ease_out_final_delay_us=GATE_OPEN_RECOVERY_EASE_FINAL_DELAY_US,
        ease_out_launch_steps=GATE_OPEN_RECOVERY_EASE_LAUNCH_STEPS,
        continuous_until_limit=True,
    )

    if started:
        if GATE_OPEN_RECOVERY_EASE_OUT:
            label = "FAST {} ms -> EASE-OUT {} ms (like normal OPEN)".format(
                GATE_OPEN_RECOVERY_FAST_MS, GATE_OPEN_RECOVERY_SLOW_MS)
        elif GATE_OPEN_RECOVERY_USE_NORMAL_SPEED:
            label = "STANDARD NORMAL OPEN PROFILE"
        else:
            label = "FIXED SLOW"
        print("OPEN RECOVERY SPEED :", label)
        print("OPEN RECOVERY ENGINE:", stepper.status().get("pulse_engine", "UNKNOWN"))

    return started


def _start_master_endpoint_seek(movement):
    """Seek only the motor side(s) whose requested endpoint is still missing."""
    global gate_state, gate_close_limit_deadline, gate_open_limit_deadline

    movement = str(movement).upper()
    if movement not in ("OPEN", "CLOSE"):
        _gate_error("INVALID ENDPOINT SEEK DIRECTION")
        return False

    update_gate_limit_switches()
    _confirm_reached_endpoints(movement)

    if _all_endpoint_limits_active(movement):
        _confirm_both_endpoint(movement)
        if movement == "OPEN":
            return _start_open_seat()
        _mark_gate_locked(from_close_limit=True)
        return True

    if movement == "OPEN":
        if not gate_open_limit_deadline:
            gate_open_limit_deadline = ticks_after_ms(GATE_OPEN_LIMIT_MAX_WAIT_MS)
        if time.ticks_diff(time.ticks_ms(), gate_open_limit_deadline) >= 0:
            return _handle_open_limit_timeout()
        delay_us = GATE_OPEN_LIMIT_SEEK_DELAY_US
        max_degrees = GATE_OPEN_LIMIT_SEEK_MAX_DEGREES
        gate_state = "OPEN_LIMIT_SEEK"
    else:
        if not gate_close_limit_deadline:
            gate_close_limit_deadline = ticks_after_ms(GATE_CLOSE_LIMIT_MAX_WAIT_MS)
        delay_us = GATE_CLOSE_LIMIT_SEEK_DELAY_US
        max_degrees = GATE_CLOSE_LIMIT_SEEK_MAX_DEGREES
        gate_state = "CLOSE_LIMIT_SEEK"

    m1_move = not _endpoint_side_ok(1, movement)
    m2_move = not _endpoint_side_ok(2, movement)

    print()
    print("{} LIMIT SEEK: INDEPENDENT SIDE RECOVERY".format(movement))
    print("MOTOR #1 {} LIMIT: GPIO{} | {}".format(
        movement, _motor_limit_pin(1, movement),
        "HOLD" if not m1_move else "SEEK"
    ))
    print("MOTOR #2 {} LIMIT: GPIO{} | {}".format(
        movement, _motor_limit_pin(2, movement),
        "HOLD" if not m2_move else "SEEK"
    ))
    print("MISSING         :", _missing_endpoint_text(movement))

    if movement == "CLOSE":
        print("SEEK PROFILE    : SLOW SMOOTH CLOSE")
        print("SPEED PROFILE   : FIXED", GATE_CLOSE_SLOW_RETRY_DELAY_US, "us half-pulse")

        started = _start_fast_close_recovery_jog(
            max_degrees,
            m1_move,
            m2_move,
            "CLOSE_LIMIT_SEEK_SLOW",
            slow=True,
        )
    else:
        print("SEEK PROFILE    : STANDARD NORMAL OPEN")
        print(
            "SPEED PROFILE   :",
            TB6600_START_DELAY_US,
            "->",
            TB6600_RUN_DELAY_US,
            "us half-pulse",
        )
        started = _start_fast_open_recovery_jog(
            max_degrees,
            m1_move,
            m2_move,
            "OPEN_LIMIT_SEEK_FAST",
        )

    if not started:
        _gate_error(
            stepper.last_error
            or "{} LIMIT SEEK START FAILED".format(movement)
        )
        return False

    return True


def _update_master_endpoint_seek(movement):
    movement = str(movement).upper()

    if gate_limits_conflict():
        _gate_error("LIMIT CONFLICT: " + gate_limit_conflict_detail())
        return

    if movement == "OPEN":
        solenoid.set_released(True)

    update_gate_limit_switches()
    _confirm_reached_endpoints(movement)

    if _all_endpoint_limits_active(movement):
        if movement == "OPEN":
            _confirm_both_endpoint("OPEN")
            _start_open_seat()
        else:
            _complete_close_limit_from_recovery()
        return

    if (
        movement == "CLOSE"
        and config["tof"]["enabled"]
        and config["tof"]["reopen_on_obstruction"]
        and _tof_motion_guard_live()
        and not _safe_to_close()
    ):
        _reopen_after_obstruction()
        return

    deadline = gate_open_limit_deadline if movement == "OPEN" else gate_close_limit_deadline
    if deadline and time.ticks_diff(time.ticks_ms(), deadline) >= 0:
        if movement == "OPEN":
            _handle_open_limit_timeout()
        else:
            _schedule_close_limit_retry(
                "CLOSE LIMIT(S) NOT DETECTED WITHIN FIXED {} ms ATTEMPT; MISSING {}".format(
                    GATE_CLOSE_LIMIT_MAX_WAIT_MS,
                    _missing_endpoint_text("CLOSE"),
                )
            )
        return

    if stepper.moving:
        stepper.update()

    update_gate_limit_switches()
    _confirm_reached_endpoints(movement)

    if _all_endpoint_limits_active(movement):
        if movement == "OPEN":
            _confirm_both_endpoint("OPEN")
            _start_open_seat()
        else:
            _complete_close_limit_from_recovery()
        return

    deadline = gate_open_limit_deadline if movement == "OPEN" else gate_close_limit_deadline
    if deadline and time.ticks_diff(time.ticks_ms(), deadline) >= 0:
        if movement == "OPEN":
            _handle_open_limit_timeout()
        else:
            _schedule_close_limit_retry(
                "CLOSE LIMIT(S) NOT DETECTED WITHIN FIXED {} ms ATTEMPT; MISSING {}".format(
                    GATE_CLOSE_LIMIT_MAX_WAIT_MS,
                    _missing_endpoint_text("CLOSE"),
                )
            )
        return

    if not stepper.moving:
        _start_master_endpoint_seek(movement)

def _handle_open_limit_timeout():
    """Stop this OPEN find attempt and retry only the still-missing side(s)."""
    global gate_busy, gate_busy_until, gate_state
    global gate_open_limit_deadline, gate_open_limit_confirmed, gate_open_limit_timed_out
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason

    if stepper is not None and stepper.moving and stepper.continuous_until_limit:
        gate_open_retry_count += 1
        gate_open_limit_deadline = ticks_after_ms(GATE_OPEN_LIMIT_MAX_WAIT_MS)
        print("OPEN RETRY: CONTINUE CURRENT SLOW PROFILE | MISSING", _missing_endpoint_text("OPEN"))
        return True

    if stepper is not None and stepper.moving:
        stepper.stop()

    gate_busy = True
    gate_busy_until = 0
    gate_open_limit_deadline = 0
    gate_open_limit_confirmed = False
    gate_open_limit_timed_out = True
    gate_open_retry_count += 1
    gate_open_retry_reason = "OPEN_LIMIT_NOT_FOUND"
    gate_open_retry_at = ticks_after_ms(GATE_LIMIT_RETRY_INTERVAL_MS)
    gate_state = "OPEN_RETRY_WAIT"

    solenoid.set_released(True)

    print()
    print("============================================================")
    print("OPEN LIMIT RETRY SCHEDULED")
    print("============================================================")
    print("FAILED ATTEMPTS:", gate_open_retry_count)
    print("MISSING        :", _missing_endpoint_text("OPEN"))
    print("MOTORS         : IMMEDIATE NEXT ATTEMPT - NO REST")
    print("SOLENOID       : REMAINS RELEASED")
    print("RETRY IN       :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")
    print("NEXT ATTEMPT   : MAX", GATE_OPEN_LIMIT_MAX_WAIT_MS, "ms")
    print("RFID           : BLOCKED UNTIL BOTH OPEN LIMITS CONFIRMED")
    print("============================================================")
    return True

def _start_open_limit_retry():
    """Start one OPEN retry using only the motor side(s) still missing."""
    global gate_state, gate_open_limit_deadline, gate_open_retry_at

    gate_open_retry_at = 0
    update_gate_limit_switches()

    if gate_limits_conflict():
        _gate_error("LIMIT CONFLICT DURING OPEN RETRY: " + gate_limit_conflict_detail())
        return False

    _confirm_reached_endpoints("OPEN")
    if _all_endpoint_limits_active("OPEN"):
        _confirm_both_endpoint("OPEN")
        return _start_open_seat()

    if stepper.moving:
        stepper.stop()

    solenoid.set_released(True)
    gate_open_limit_deadline = ticks_after_ms(GATE_OPEN_LIMIT_MAX_WAIT_MS)
    gate_state = "OPEN_LIMIT_RETRY"

    m1_move = not gate_open_limit_active
    m2_move = not gate_open_limit2_active

    print()
    print("OPEN LIMIT BACKGROUND RETRY START")
    print("ATTEMPT MAX :", GATE_OPEN_LIMIT_MAX_WAIT_MS, "ms FIXED")
    print("RETRY DELAY :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")
    print("MOTOR #1    :", "OPEN" if m1_move else "HOLD - OWN LIMIT ACTIVE")
    print("MOTOR #2    :", "OPEN" if m2_move else "HOLD - OWN LIMIT ACTIVE")
    print("PULSE MODE  : EASE-OUT OPEN / RMT")

    started = _start_fast_open_recovery_jog(
        GATE_OPEN_LIMIT_SEEK_MAX_DEGREES,
        m1_move,
        m2_move,
        "OPEN_LIMIT_RETRY_FAST",
    )
    if not started:
        _gate_error(stepper.last_error or "OPEN LIMIT RETRY START FAILED")
        return False

    return True

def _schedule_close_limit_retry(reason, count_failure=True):
    """Stop active CLOSE recovery and retry the still-missing side(s)."""
    global gate_busy, gate_busy_until, gate_state
    global gate_close_slow_mode, gate_close_retry_wait_since
    global gate_close_limit_deadline, gate_close_retry_at
    global gate_close_retry_count, gate_close_retry_reason

    if (
        stepper is not None and stepper.moving and stepper.continuous_until_limit
        and (gate_close_slow_mode or not count_failure)
    ):
        # Already on the slow retry (or an idle recovery): keep moving, renew.
        if count_failure:
            gate_close_retry_count += 1
        gate_close_limit_deadline = ticks_after_ms(GATE_CLOSE_LIMIT_MAX_WAIT_MS)
        print("CLOSE RETRY: NEXT 10000 ms WINDOW | MISSING", _missing_endpoint_text("CLOSE"))
        return True

    # v2.1.1: the first (full speed) window expired without BOTH CLOSE switches
    # = the arm was blocked or delayed. Every further attempt runs slow.
    if count_failure and not gate_close_slow_mode:
        gate_close_slow_mode = True
        print("CLOSE BLOCKED/DELAYED: SWITCHING TO SLOW SMOOTH CLOSE RETRIES")

    if stepper is not None and stepper.moving:
        stepper.stop()

    gate_busy = True
    gate_busy_until = 0
    gate_close_limit_deadline = 0
    if count_failure:
        gate_close_retry_count += 1
    gate_close_retry_reason = str(reason or "CLOSE_LIMIT_NOT_FOUND")
    gate_close_retry_at = ticks_after_ms(GATE_LIMIT_RETRY_INTERVAL_MS)
    gate_close_retry_wait_since = 0
    gate_state = "CLOSE_RETRY_WAIT"

    print()
    print("============================================================")
    print("CLOSE LIMIT RETRY SCHEDULED")
    print("============================================================")
    print("REASON         :", gate_close_retry_reason)
    print("FAILED ATTEMPTS:", gate_close_retry_count)
    print("MISSING        :", _missing_endpoint_text("CLOSE"))
    print("MOTORS         : IMMEDIATE NEXT ATTEMPT - NO REST")
    print("RETRY IN       :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")
    print("NEXT ATTEMPT   : MAX", GATE_CLOSE_LIMIT_MAX_WAIT_MS, "ms @ SLOW",
          GATE_CLOSE_SLOW_RETRY_DELAY_US, "us")
    print("CLOSE RECOVERY : ONLY MISSING MOTOR SIDE(S) WILL MOVE")
    print("RFID NEW ACCESS: BLOCKED UNTIL BOTH CLOSE LIMITS CONFIRMED")
    print("============================================================")
    return True

def _close_retry_is_idle_rfid_recovery():
    return str(gate_close_retry_reason).startswith("RFID_IDLE_CLOSE_RECOVERY")


def _complete_close_limit_from_recovery():
    """Finish CLOSE recovery only after both side-specific CLOSE limits."""
    if not _all_endpoint_limits_active("CLOSE"):
        return False
    idle_rfid_recovery = _close_retry_is_idle_rfid_recovery()
    _confirm_both_endpoint("CLOSE")
    _mark_gate_locked(
        from_close_limit=True,
        arm_anti_tailgate=not idle_rfid_recovery,
    )
    return True

def _trigger_close_recovery_from_rfid(card_id):
    """Arm CLOSE recovery only after the stationary CLOSE loss survives filtering."""
    update_gate_limit_switches()

    if gate_limits_conflict():
        _gate_error("LIMIT CONFLICT WHILE RFID REQUESTED CLOSE RECOVERY: " + gate_limit_conflict_detail())
        return False

    if _all_endpoint_limits_active("CLOSE"):
        print("RFID CLOSE RECOVERY CANCELLED: BOTH CLOSE LIMITS ARE ACTIVE")
        return False

    if gate_state in (
        "CLOSING",
        "CLOSE_LIMIT_SEEK",
        "CLOSE_LIMIT_RETRY",
        "CLOSE_RETRY_WAIT",
    ):
        print("RFID CLOSE RECOVERY: ALREADY ACTIVE -", gate_state)
        return True

    if gate_busy:
        print("RFID CLOSE RECOVERY DEFERRED: GATE BUSY -", gate_state)
        return False

    if gate_state != "LOCKED":
        print("RFID CLOSE RECOVERY DEFERRED: GATE STATE -", gate_state)
        return False

    print()
    print("============================================================")
    print("RFID DETECTED FORCED-OPEN / LOST CLOSE POSITION")
    print("============================================================")
    print("CARD ID        :", card_id)
    print("MISSING CLOSE  :", _missing_endpoint_text("CLOSE"))
    print("ACTION         : ARM INDEPENDENT-SIDE CLOSE RECOVERY")
    print("RETRY IN       :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")
    print("ATTEMPT MAX    :", GATE_CLOSE_LIMIT_MAX_WAIT_MS, "ms FIXED")
    print("IDLE FILTER    :", GATE_IDLE_LIMIT_GLITCH_FILTER_MS, "ms STABLE LOSS REQUIRED")
    print("============================================================")

    # v2.1.8: idle recovery is a NEW cycle - trust only the live switches.
    _clear_endpoint_latch("CLOSE")
    return _schedule_close_limit_retry(
        "RFID_IDLE_CLOSE_RECOVERY {}".format(card_id),
        count_failure=False,
    )

def _start_close_limit_retry():
    """Retry CLOSE using only the motor side(s) whose own limit is missing."""
    global gate_state, gate_close_limit_deadline, gate_close_retry_at
    global gate_close_retry_wait_since

    gate_close_retry_at = 0

    update_gate_limit_switches()
    if gate_limits_conflict():
        _gate_error("LIMIT CONFLICT DURING CLOSE RETRY: " + gate_limit_conflict_detail())
        return False

    _confirm_reached_endpoints("CLOSE")
    if _all_endpoint_limits_active("CLOSE"):
        return _complete_close_limit_from_recovery()

    if not _safe_to_close():
        now = time.ticks_ms()
        if not gate_close_retry_wait_since:
            gate_close_retry_wait_since = now
            print("CLOSE RETRY DEFERRED: PASSAGE NOT CLEAR - WAITING UP TO",
                  GATE_CLOSE_RETRY_CLEAR_WAIT_MS, "ms")
        if time.ticks_diff(now, gate_close_retry_wait_since) < int(GATE_CLOSE_RETRY_CLEAR_WAIT_MS):
            gate_close_retry_at = ticks_after_ms(GATE_LIMIT_RETRY_INTERVAL_MS)
            gate_state = "CLOSE_RETRY_WAIT"
            return True
        print("CLOSE RETRY: LANE STILL NOT CLEAR AFTER", GATE_CLOSE_RETRY_CLEAR_WAIT_MS,
              "ms - CONTINUING SLOWLY")
    gate_close_retry_wait_since = 0

    if stepper.moving:
        stepper.stop()

    gate_close_limit_deadline = ticks_after_ms(GATE_CLOSE_LIMIT_MAX_WAIT_MS)
    gate_state = "CLOSE_LIMIT_RETRY"

    m1_move = not gate_close_limit_active
    m2_move = not gate_close_limit2_active

    print()
    print("CLOSE LIMIT BACKGROUND RETRY START")
    print("ATTEMPT MAX :", GATE_CLOSE_LIMIT_MAX_WAIT_MS, "ms FIXED")
    print("RETRY DELAY :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")
    print("MOTOR #1    :", "CLOSE" if m1_move else "HOLD - OWN LIMIT ACTIVE")
    print("MOTOR #2    :", "CLOSE" if m2_move else "HOLD - OWN LIMIT ACTIVE")
    slow = True   # v2.1.2: every CLOSE retry / recovery runs slow
    print("PULSE MODE  :", "SLOW SMOOTH CLOSE" if slow else "FULL SPEED CLOSE", "/ RMT")
    if slow:
        print("SPEED PROFILE: FIXED", GATE_CLOSE_SLOW_RETRY_DELAY_US, "us half-pulse")
    else:
        print("SPEED PROFILE:", TB6600_START_DELAY_US, "->", TB6600_RUN_DELAY_US, "us half-pulse")

    started = _start_fast_close_recovery_jog(
        GATE_CLOSE_LIMIT_SEEK_MAX_DEGREES,
        m1_move,
        m2_move,
        "CLOSE_LIMIT_RETRY_SLOW" if slow else "CLOSE_LIMIT_RETRY_FAST",
        slow=slow,
    )
    if not started:
        _gate_error(stepper.last_error or "CLOSE LIMIT RETRY START FAILED")
        return False

    return True

def _start_close_motor():
    global gate_busy, gate_busy_until, gate_state, gate_close_limit_deadline
    global gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason
    global gate_close_slow_mode, gate_close_retry_wait_since, gate_solenoid_lock_at
    global gate_open_since

    # v2.1.2: every new close cycle starts with the fast-then-smooth attempt.
    gate_close_slow_mode = False
    gate_close_retry_wait_since = 0
    gate_open_since = 0
    if gate_solenoid_lock_at and solenoid is not None:
        solenoid.force_locked()
        gate_solenoid_lock_at = 0
        print("SOLENOID: LOCKED BEFORE CLOSE")

    gate_busy_until = 0
    gate_close_limit_deadline = 0
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    if gate_close_retry_reason != "OPEN_LIMIT_TIMEOUT":
        gate_close_retry_reason = ""

    update_gate_limit_switches()

    if not config["gate"]["motor_enabled"]:
        if _all_endpoint_limits_active("CLOSE"):
            _mark_gate_locked(from_close_limit=True)
            return True
        _gate_error("MOTOR DISABLED BUT BOTH CLOSE LIMITS ARE NOT ACTIVE")
        return False

    if stepper.moving:
        stepper.stop()

    _pre_motion_gc()
    _clear_endpoint_latch("CLOSE")

    if gate_state in ("OPEN", "WAIT_CLEAR"):
        stepper.reconcile_position(
            config["gate"]["open_angle"],
            reason="pre-CLOSE gate state " + gate_state,
        )

    gate_busy = True
    gate_state = "CLOSING"
    gate_close_limit_deadline = ticks_after_ms(GATE_CLOSE_LIMIT_MAX_WAIT_MS)

    if not stepper.rearm_rmt_for_new_motion("AUTO_CLOSE"):
        _gate_error(stepper.last_error or "TB6600 RMT REARM FAILED BEFORE CLOSE")
        return False

    print()
    print("GATE CLOSE REQUEST - AUTOMATIC / DUAL-SIDE LIMITS")
    print("MOTOR POSITION:", stepper.status()["angle"], "deg")
    print("CLOSE TARGET  :", config["gate"]["closed_angle"], "deg")
    print("MOTOR #1 LIMIT: GPIO", GATE_MOTOR1_CLOSE_LIMIT_PIN)
    print("MOTOR #2 LIMIT: GPIO", GATE_MOTOR2_CLOSE_LIMIT_PIN)
    print("SYNC POLICY   : SAME PROFILE; EACH MOTOR STOPS ON OWN LIMIT")
    print("CLOSE PROFILE : FAST {} us for first {} %, SMOOTH to {} us, SLOW into CLOSE switch".format(
        TB6600_RUN_DELAY_US, TB6600_CLOSE_DECEL_START_PERCENT, TB6600_CLOSE_FINAL_DELAY_US))
    print("PULSE ENGINE  :", stepper.status().get("pulse_engine", "UNKNOWN"))

    started = stepper.start_move_to_angle(
        config["gate"]["closed_angle"],
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
        movement="CLOSE",
        continuous_until_limit=True,
    )
    if not started:
        _gate_error(stepper.last_error or "TB6600 CLOSE START FAILED")
        return False

    _confirm_reached_endpoints("CLOSE")

    if _all_endpoint_limits_active("CLOSE"):
        _confirm_both_endpoint("CLOSE")
        _mark_gate_locked(from_close_limit=True)
    elif not stepper.moving:
        _start_master_endpoint_seek("CLOSE")
    else:
        print("GATE STATUS: CLOSING - BOTH CLOSE LIMITS REQUIRED")
    return True

def _latch_shake_movement_for_leg(leg_index):
    # Leg 0 is CLOSE. Every following leg alternates direction.
    # With 2 cycles the default pattern is:
    #   0 C, 1 O, 2 C, 3 O, 4 C
    # v2.1.6 SLOW_OPEN: the single leg is OPEN.
    if gate_latch_single_open:
        return "OPEN"
    return "CLOSE" if (int(leg_index) % 2) == 0 else "OPEN"


def _start_latch_release_leg():
    """Start the current leg of the alternating latch-release shake."""
    global gate_state
    global gate_latch_shake_pause_until

    gate_cfg = config["gate"]
    degrees = float(
        gate_cfg.get(
            "latch_release_jog_degrees",
            LATCH_RELEASE_JOG_DEGREES_DEFAULT,
        )
    )
    delay_us = int(
        gate_cfg.get(
            "latch_release_jog_delay_us",
            LATCH_RELEASE_JOG_DELAY_US_DEFAULT,
        )
    )

    if gate_latch_single_open:
        degrees = float(LATCH_SLOW_OPEN_DEGREES)
        delay_us = int(LATCH_SLOW_OPEN_DELAY_US)

    if gate_latch_shake_total_legs <= 0:
        return _finish_latch_release_jog()

    if gate_latch_shake_leg >= gate_latch_shake_total_legs:
        return _finish_latch_release_jog()

    movement = _latch_shake_movement_for_leg(gate_latch_shake_leg)

    # Keep the latch coil continuously powered during every direction change.
    solenoid.set_released(True)

    gate_latch_shake_pause_until = 0
    gate_state = "LATCH_RELEASE"

    print()
    print(
        "LATCH SHAKE LEG {}/{}".format(
            gate_latch_shake_leg + 1,
            gate_latch_shake_total_legs,
        )
    )
    print("DIRECTION    :", movement)
    print("DISTANCE     :", degrees, "deg")
    print("HALF-PULSE   :", delay_us, "us")
    print("SOLENOID     : REMAINS UNLOCKED")

    started = stepper.start_jog_relative(
        degrees,
        movement=movement,
        enabled=True,
        direction_inverted=gate_cfg["direction_inverted"],
        delay_us=delay_us,
        # The 5-degree latch shake is deliberately software-timed. It is short
        # and slow, and must never leave the RMT peripheral in fallback state
        # before the full-speed normal OPEN movement.
        force_bitbang=True,
    )
    if not started:
        _gate_error(
            stepper.last_error
            or "TB6600 LATCH SHAKE LEG {} FAILED".format(
                gate_latch_shake_leg + 1
            )
        )
        return False

    if not stepper.moving:
        # Extremely small move rounded to zero steps.
        return _complete_latch_release_leg()

    return True


def _complete_latch_release_leg():
    """Advance to the next OPEN/CLOSE shake leg or finish the sequence."""
    global gate_state
    global gate_latch_shake_leg
    global gate_latch_shake_pause_until

    gate_latch_shake_leg += 1

    if gate_latch_shake_leg >= gate_latch_shake_total_legs:
        print()
        print("LATCH SHAKE MOTION COMPLETE")
        return _finish_latch_release_jog()

    pause_ms = int(
        config["gate"].get(
            "latch_release_shake_pause_ms",
            LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT,
        )
    )

    if pause_ms <= 0:
        return _start_latch_release_leg()

    # A short pause makes the direction reversal mechanically distinct and
    # gives the energized solenoid a chance to retract while side-load changes.
    solenoid.set_released(True)
    gate_state = "LATCH_SHAKE_PAUSE"
    gate_latch_shake_pause_until = ticks_after_ms(pause_ms)

    next_movement = _latch_shake_movement_for_leg(gate_latch_shake_leg)
    print(
        "LATCH SHAKE PAUSE:",
        pause_ms,
        "ms -> next",
        next_movement,
    )
    return True
def _start_latch_release_jog():
    """Start the v1.7.4 two-cycle wide alternating latch-release shake.

    The solenoid is already energized and has already received the configured
    pre-motor delay.  Unlike the old single CLOSE preload, this sequence moves
    CLOSE/OPEN repeatedly so a sticky strike/latch sees a real reversal of load.

    The final leg is always CLOSE.  The temporary shake position is discarded
    before the normal full OPEN movement starts.

    This function is NOT used for obstruction reopening.
    """
    global gate_busy
    global gate_busy_until
    global gate_state
    global gate_latch_settle_until
    global gate_latch_shake_leg
    global gate_latch_shake_total_legs
    global gate_latch_shake_pause_until

    gate_cfg = config["gate"]

    if not gate_cfg.get("motor_enabled", True):
        return _start_open_motor(reopening=False)

    if not gate_cfg.get("latch_release_jog_enabled", True):
        print("LATCH RELEASE SHAKE: DISABLED - STARTING OPEN")
        return _start_open_motor(reopening=False)

    shake_degrees = float(
        gate_cfg.get(
            "latch_release_jog_degrees",
            LATCH_RELEASE_JOG_DEGREES_DEFAULT,
        )
    )
    shake_delay_us = int(
        gate_cfg.get(
            "latch_release_jog_delay_us",
            LATCH_RELEASE_JOG_DELAY_US_DEFAULT,
        )
    )
    shake_cycles = int(
        gate_cfg.get(
            "latch_release_shake_cycles",
            LATCH_RELEASE_SHAKE_CYCLES_DEFAULT,
        )
    )
    pause_ms = int(
        gate_cfg.get(
            "latch_release_shake_pause_ms",
            LATCH_RELEASE_SHAKE_PAUSE_MS_DEFAULT,
        )
    )

    if shake_degrees <= 0.0 or shake_cycles <= 0:
        print("LATCH RELEASE SHAKE: ZERO MOVEMENT - STARTING OPEN")
        return _start_open_motor(reopening=False)

    if str(LATCH_RELEASE_MODE).upper() == "SLOW_OPEN":
        return _start_latch_slow_open()

    # One initial CLOSE preload + two legs (OPEN/CLOSE) for each cycle.
    # Default 2 cycles: C O C O C = 5 legs.
    gate_latch_shake_leg = 0
    gate_latch_shake_total_legs = 1 + (shake_cycles * 2)
    gate_latch_shake_pause_until = 0

    # Keep the solenoid continuously energized for the full sequence.
    solenoid.set_released(True)

    gate_busy = True
    gate_busy_until = 0
    gate_latch_settle_until = 0
    gate_state = "LATCH_RELEASE"

    print()
    print("============================================================")
    print("LATCH RELEASE ASSIST - WIDE ALTERNATING SHAKE")
    print("============================================================")
    print(
        "PROFILE VERSION:",
        gate_cfg.get(
            "latch_release_profile_version",
            LATCH_RELEASE_PROFILE_VERSION,
        ),
    )
    print("AMPLITUDE      :", shake_degrees, "deg each leg")
    print("SHAKE CYCLES   :", shake_cycles)
    print("TOTAL LEGS     :", gate_latch_shake_total_legs)
    print("SHAKE DELAY    :", shake_delay_us, "us half-pulse")
    print("DIR PAUSE      :", pause_ms, "ms")
    print(
        "PATTERN        : initial CLOSE +",
        shake_cycles,
        "x (OPEN -> CLOSE)",
    )
    print("FINAL POSITION : CLOSE preload")
    print("SOLENOID       : REMAINS UNLOCKED")
    print(
        "NORMAL OPEN    : starts only after all shake legs + final hold"
    )
    print("============================================================")

    return _start_latch_release_leg()


def _start_latch_slow_open():
    """v2.1.6: one smooth slow OPEN leg, then normal OPEN (no reversals)."""
    global gate_busy, gate_busy_until, gate_state, gate_latch_settle_until
    global gate_latch_shake_leg, gate_latch_shake_total_legs
    global gate_latch_shake_pause_until, gate_latch_single_open

    if float(LATCH_SLOW_OPEN_DEGREES) <= 0.0:
        print("LATCH SLOW OPEN: ZERO DISTANCE - STARTING OPEN")
        return _start_open_motor(reopening=False)

    gate_latch_single_open = True
    gate_latch_shake_leg = 0
    gate_latch_shake_total_legs = 1
    gate_latch_shake_pause_until = 0
    solenoid.set_released(True)
    gate_busy = True
    gate_busy_until = 0
    gate_latch_settle_until = 0
    gate_state = "LATCH_RELEASE"

    print()
    print("============================================================")
    print("LATCH RELEASE ASSIST - SMOOTH SLOW OPEN (ONE DIRECTION)")
    print("============================================================")
    print("DISTANCE       :", LATCH_SLOW_OPEN_DEGREES, "deg OPEN")
    print("SPEED          :", LATCH_SLOW_OPEN_DELAY_US, "us half-pulse")
    print("REVERSALS      : NONE (no CLOSE preload, no hold)")
    print("THEN           : normal OPEN continues from this position")
    print("SOLENOID       : REMAINS UNLOCKED")
    print("============================================================")
    return _start_latch_release_leg()


def _finish_latch_release_jog():
    """Finish the shake, re-anchor CLOSED, hold preload, then begin OPEN."""
    global gate_state
    global gate_latch_settle_until

    if gate_latch_single_open:
        # v2.1.6: OPEN-only lift-off - the tracked position is real (it never
        # pushed into a stop), so keep it and go straight into normal OPEN.
        _reset_latch_shake_runtime()
        print("LATCH SLOW OPEN COMPLETE -> NORMAL OPEN (no hold)")
        return _start_open_motor(reopening=False)

    # The shake is a mechanical pressure-release action, not a trusted absolute
    # position measurement.  It can include movement against the CLOSED stop.
    # Therefore discard the temporary accumulated shake position.
    if stepper is not None and not stepper.moving:
        stepper.reconcile_position(
            config["gate"]["closed_angle"],
            reason="wide latch-shake re-anchor to CLOSED",
        )

    settle_ms = int(
        config["gate"].get(
            "latch_release_settle_ms",
            LATCH_RELEASE_SETTLE_MS_DEFAULT,
        )
    )

    # The last shake leg is CLOSE. Hold that final preload while the solenoid
    # remains energized, then launch the normal full OPEN move.
    solenoid.set_released(True)

    print("POSITION RE-ANCHOR:", config["gate"]["closed_angle"], "deg CLOSED")
    print("SOLENOID        : REMAINS UNLOCKED")
    print("FINAL CLOSE HOLD:", settle_ms, "ms before normal OPEN")

    _reset_latch_shake_runtime()

    if settle_ms <= 0:
        gate_latch_settle_until = 0
        return _start_open_motor(reopening=False)

    gate_state = "LATCH_SETTLE"
    gate_latch_settle_until = ticks_after_ms(settle_ms)

    print("LATCH HOLD/SETTLE:", settle_ms, "ms")
    return True


def _start_open_motor(reopening=False):
    """Start synchronized OPEN; stop each motor independently on its own limit."""
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_solenoid_lock_at, gate_open_limit_deadline
    global gate_open_limit_confirmed, gate_open_limit_timed_out
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason

    gate_motor_start_at = 0
    gate_latch_settle_until = 0
    _reset_latch_shake_runtime()

    gate_solenoid_lock_at = 0
    gate_open_limit_confirmed = False
    gate_open_limit_timed_out = False
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    solenoid.set_released(True)

    if not config["gate"]["motor_enabled"]:
        _gate_error("TB6600 MOTOR IS DISABLED IN CONFIG / WEB UI")
        return False

    if float(config["gate"]["open_angle"]) == float(config["gate"]["closed_angle"]):
        print("WARNING: OPEN ANGLE EQUALS CLOSE ANGLE - MOTOR HAS NO DISTANCE TO MOVE")

    _clear_endpoint_latch()
    gate_busy = True
    gate_busy_until = 0
    gate_state = "REOPENING" if reopening else "OPENING"
    gate_open_limit_deadline = ticks_after_ms(GATE_OPEN_LIMIT_MAX_WAIT_MS)

    if not stepper.rearm_rmt_for_new_motion("REOPEN" if reopening else "AUTO_OPEN"):
        _gate_error(stepper.last_error or "TB6600 RMT REARM FAILED BEFORE OPEN")
        return False

    print("SOLENOID PRE-UNLOCK COMPLETE: STARTING BOTH NEMA MOTORS")
    print("OPEN LIMIT EXTRA SEEK:", GATE_OPEN_LIMIT_MAX_WAIT_MS, "ms FIXED IF EITHER SIDE MISSES")
    print("MOTOR #1 OPEN LIMIT: GPIO", GATE_MOTOR1_OPEN_LIMIT_PIN)
    print("MOTOR #2 OPEN LIMIT: GPIO", GATE_MOTOR2_OPEN_LIMIT_PIN)
    print("SYNC POLICY: SAME RMT SPEED PROFILE; EACH MOTOR STOPS ON OWN OPEN LIMIT")
    print("PULSE ENGINE:", stepper.status().get("pulse_engine", "UNKNOWN"))

    started = stepper.start_move_to_angle(
        config["gate"]["open_angle"],
        enabled=True,
        direction_inverted=config["gate"]["direction_inverted"],
        movement="OPEN",
        continuous_until_limit=True,
    )
    if not started:
        _gate_error(stepper.last_error or "TB6600 OPEN START FAILED")
        return False

    update_gate_limit_switches()
    _confirm_reached_endpoints("OPEN")

    if _all_endpoint_limits_active("OPEN"):
        _confirm_both_endpoint("OPEN")
        _start_open_seat()
    elif not stepper.moving:
        _start_master_endpoint_seek("OPEN")
    else:
        _update_open_peer_wait_state()
        if gate_state not in ("OPEN_WAIT_MOTOR1", "OPEN_WAIT_MOTOR2"):
            print("GATE STATUS:", "REOPENING" if reopening else "OPENING")
    return True

def _reopen_after_obstruction():
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_solenoid_lock_at
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason
    global gate_close_limit_deadline, gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason

    print("GATE SAFETY: OBSTRUCTION DETECTED WHILE CLOSING - PREPARING REOPEN")
    if stepper.moving:
        stepper.stop()

    # Release first, then wait non-blockingly before reversing the NEMA motor.
    solenoid.set_released(True)
    gate_solenoid_lock_at = 0
    gate_busy = True
    gate_busy_until = 0
    gate_latch_settle_until = 0
    gate_close_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    gate_close_retry_reason = ""
    _reset_latch_shake_runtime()
    gate_state = "REUNLOCKING"
    gate_motor_start_at = ticks_after_ms(config["gate"]["motor_start_delay_ms"])

    print("SOLENOID: UNLOCKED FOR REOPEN")
    print("NEMA REOPEN DELAY:", config["gate"]["motor_start_delay_ms"], "ms")
    return True


def unlock_gate():
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_solenoid_lock_at
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason
    global gate_close_limit_deadline, gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason

    if gate_busy:
        print("GATE OPEN REQUEST IGNORED: GATE BUSY")
        return False

    if gate_state != "LOCKED":
        print("GATE OPEN REQUEST BLOCKED: GATE STATE IS", gate_state)
        return False

    update_gate_limit_switches()

    if GATE_LIMIT_SWITCHES_ENABLED and gate_limits_conflict():
        print("GATE OPEN REQUEST BLOCKED: LIMIT CONFLICT -", gate_limit_conflict_detail())
        return False

    if GATE_LIMIT_SWITCHES_REQUIRED and not _all_endpoint_limits_active("CLOSE"):
        print("GATE OPEN REQUEST BLOCKED: BOTH CLOSE LIMITS NOT CONFIRMED")
        print("MISSING CLOSE:", _missing_endpoint_text("CLOSE"))
        return False

    if _all_endpoint_limits_active("OPEN") and not _all_endpoint_limits_active("CLOSE"):
        print("GATE OPEN REQUEST BLOCKED: GATE IS ALREADY AT BOTH OPEN LIMITS")
        return False

    if stepper is not None and not stepper.moving:
        stepper.reconcile_position(
            config["gate"]["closed_angle"],
            reason="pre-OPEN BOTH CLOSE limits confirmed",
        )

    gate_busy = True
    gate_busy_until = 0
    gate_latch_settle_until = 0
    gate_solenoid_lock_at = 0
    gate_close_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    gate_close_retry_reason = ""
    _reset_latch_shake_runtime()
    gate_state = "UNLOCKING"

    print()
    print("GATE OPEN REQUEST")
    print("MOTOR ENABLED      :", config["gate"]["motor_enabled"])
    print("M1 CLOSE / OPEN    :", gate_close_limit_active, "/", gate_open_limit_active,
          "GPIO", GATE_MOTOR1_CLOSE_LIMIT_PIN, "/", GATE_MOTOR1_OPEN_LIMIT_PIN)
    print("M2 CLOSE / OPEN    :", gate_close_limit2_active, "/", gate_open_limit2_active,
          "GPIO", GATE_MOTOR2_CLOSE_LIMIT_PIN, "/", GATE_MOTOR2_OPEN_LIMIT_PIN)
    print("CLOSED ANGLE       :", config["gate"]["closed_angle"])
    print("OPEN ANGLE         :", config["gate"]["open_angle"])
    print("DIR INVERTED       :", config["gate"]["direction_inverted"])
    print("SOLENOID ACTIVE LOW:", SOLENOID_RELAY_ACTIVE_LOW)
    print("PRE-MOTOR DELAY    :", config["gate"]["motor_start_delay_ms"], "ms")
    print("LATCH PROFILE VER  :", config["gate"]["latch_release_profile_version"])
    print("LATCH SHAKE ENABLED:", config["gate"]["latch_release_jog_enabled"])
    print("LATCH SHAKE DEG    :", config["gate"]["latch_release_jog_degrees"])
    print("LATCH SHAKE CYCLES :", config["gate"]["latch_release_shake_cycles"])
    print("LATCH SHAKE DELAY  :", config["gate"]["latch_release_jog_delay_us"], "us")
    print("LATCH DIR PAUSE    :", config["gate"]["latch_release_shake_pause_ms"], "ms")
    print("LATCH FINAL HOLD   :", config["gate"]["latch_release_settle_ms"], "ms")

    solenoid.set_released(True)
    gate_motor_start_at = ticks_after_ms(config["gate"]["motor_start_delay_ms"])

    print("GATE STATUS: UNLOCKING - WAITING BEFORE NEMA START")
    _pre_motion_gc()
    return True

def _open_max_hold_expired():
    """v2.1.1: True once the gate has been fully OPEN for GATE_OPEN_MAX_HOLD_MS."""
    if not GATE_OPEN_MAX_HOLD_MS or not gate_open_since:
        return False
    if gate_state not in ("OPEN", "WAIT_CLEAR"):
        return False
    return time.ticks_diff(time.ticks_ms(), gate_open_since) >= int(GATE_OPEN_MAX_HOLD_MS)


def lock_gate():
    global presence_clear_close_at, presence_clear_close_delay_ms

    if gate_state == "OPEN" and GATE_LIMIT_SWITCHES_REQUIRED:
        if not gate_open_limit_confirmed:
            print("CLOSE BLOCKED: DUAL OPEN CONFIRMATION WAS NOT COMPLETED")
            return False

    # v2.1.1 hard cap: never stay open longer than GATE_OPEN_MAX_HOLD_MS after
    # BOTH OPEN limits, even while the lane is not clear.
    if _open_max_hold_expired():
        print("AUTO CLOSE: MAX OPEN {} ms REACHED | LANE {} | CLOSING NOW".format(
            GATE_OPEN_MAX_HOLD_MS, _presence_state()))
        started = _start_close_motor()
        if started:
            presence_clear_close_at = 0
        return started

    # v1.9.8 includes the v1.9.7 one-shot close-timer fix.
    #
    # The timer may START only from a fresh two-sensor NO_PRESENCE. Once it
    # expires, its deadline stays latched. It is NOT reset to zero just because
    # one final safety check temporarily reports STALE; that old behavior caused
    # the same 1000 ms timer to be re-armed forever.
    if config["tof"]["enabled"] and gate_state in ("OPEN", "WAIT_CLEAR"):
        state = _presence_state()

        if state == "PRESENCE":
            _cancel_no_presence_close("PRESENCE DETECTED")
            _wait_for_clear()
            return True

        if state != "NO_PRESENCE":
            _cancel_no_presence_close("PRESENCE SENSOR NOT READY")
            _wait_for_clear()
            return True

        if not presence_clear_close_at:
            if not _fresh_no_presence_now():
                _wait_for_clear()
                return True

            reason = (
                auto_close_reason
                if auto_close_requested
                else
                "CLOSE REQUEST / FRESH NO_PRESENCE"
            )
            _arm_no_presence_close(reason)
            _wait_for_clear()
            return True

        # v2.1.1: S2 may confirm the exit just after the timer was armed with
        # the longer "exit not seen" delay; a shorter delay now applies.
        shorter = _clear_close_delay_ms()
        if shorter < presence_clear_close_delay_ms:
            candidate = ticks_after_ms(shorter)
            if time.ticks_diff(candidate, presence_clear_close_at) < 0:
                presence_clear_close_at = candidate
                print("CLOSE TIMER SHORTENED TO", shorter, "ms: EXIT CONFIRMED")
            presence_clear_close_delay_ms = shorter

        if time.ticks_diff(
            time.ticks_ms(),
            presence_clear_close_at,
        ) < 0:
            _wait_for_clear()
            return True

        # Timer is expired and remains latched here. Do NOT clear it yet.

    if not _safe_to_close():
        _wait_for_clear()
        return True

    print("CLOSE PREPARATION COMPLETE: BOTH MOTORS START CLOSE TOGETHER")

    started = _start_close_motor()

    if started:
        presence_clear_close_at = 0

    return started


def stop_gate_outputs():
    global gate_busy, gate_busy_until, gate_state, gate_motor_start_at, gate_latch_settle_until
    global gate_solenoid_lock_at, gate_open_seat_settle_until
    global gate_open_limit_deadline, gate_close_limit_deadline
    global gate_open_retry_at, gate_open_retry_count, gate_open_retry_reason
    global gate_close_retry_at, gate_close_retry_count, gate_close_retry_reason
    stepper.stop()
    buzzer.stop()
    solenoid.force_locked()
    gate_busy = False
    gate_busy_until = 0
    gate_motor_start_at = 0
    gate_latch_settle_until = 0
    gate_solenoid_lock_at = 0
    gate_open_seat_settle_until = 0
    gate_open_limit_deadline = 0
    gate_close_limit_deadline = 0
    gate_open_retry_at = 0
    gate_open_retry_count = 0
    gate_open_retry_reason = ""
    gate_close_retry_at = 0
    gate_close_retry_count = 0
    gate_close_retry_reason = ""
    _reset_latch_shake_runtime()
    gate_state = "STOPPED"
    _end_granted_passage("OUTPUTS STOPPED")

def update_gate():
    global gate_solenoid_lock_at, gate_busy_until

    solenoid.update()
    buzzer.update()

    if not gate_busy:
        _service_gate_error_recovery()
        _service_idle_close_guard()
        return

    update_gate_limit_switches()

    if GATE_LIMIT_SWITCHES_ENABLED and gate_limits_conflict():
        _gate_error("LIMIT CONFLICT: " + gate_limit_conflict_detail())
        return

    if gate_state in ("UNLOCKING", "REUNLOCKING"):
        if gate_motor_start_at and time.ticks_diff(time.ticks_ms(), gate_motor_start_at) >= 0:
            if gate_state == "UNLOCKING":
                _start_latch_release_jog()
            else:
                _start_open_motor(reopening=True)
        return

    if gate_state == "LATCH_RELEASE":
        # Intentional exception: CLOSE limits on BOTH sides are ignored only
        # during the small latch-release shake because CLOSE legs deliberately
        # preload the mechanism before normal OPEN begins.
        stepper.update()
        if not stepper.moving:
            if stepper.last_result == "ERROR":
                _gate_error(stepper.last_error or "TB6600 LATCH SHAKE ERROR")
            else:
                _complete_latch_release_leg()
        return

    if gate_state == "LATCH_SHAKE_PAUSE":
        solenoid.set_released(True)
        if (
            gate_latch_shake_pause_until
            and time.ticks_diff(time.ticks_ms(), gate_latch_shake_pause_until) >= 0
        ):
            _start_latch_release_leg()
        return

    if gate_state == "LATCH_SETTLE":
        if gate_latch_settle_until and time.ticks_diff(
            time.ticks_ms(), gate_latch_settle_until
        ) >= 0:
            _start_open_motor(reopening=False)
        return

    if gate_state in (
        "OPENING",
        "REOPENING",
        "OPEN_WAIT_MOTOR1",
        "OPEN_WAIT_MOTOR2",
    ):
        solenoid.set_released(True)
        gate_solenoid_lock_at = 0
        gate_busy_until = 0

        # First stop/anchor any side whose own OPEN switch has already arrived.
        # The peer remains enabled and continues with the same movement.
        _confirm_reached_endpoints("OPEN")

        if _all_endpoint_limits_active("OPEN"):
            _confirm_both_endpoint("OPEN")
            _start_open_seat()
            return

        _update_open_peer_wait_state()

        if stepper.moving:
            if gate_open_limit_deadline and time.ticks_diff(time.ticks_ms(), gate_open_limit_deadline) >= 0:
                _handle_open_limit_timeout()
            stepper.update()

        update_gate_limit_switches()
        _confirm_reached_endpoints("OPEN")

        if _all_endpoint_limits_active("OPEN"):
            print("OPEN SYNC: LATE MOTOR HAS NOW REACHED ITS OPEN LIMIT")
            _confirm_both_endpoint("OPEN")
            _start_open_seat()
            return

        _update_open_peer_wait_state()

        # If commanded step travel ends before the late side reaches its physical
        # switch, transition to the existing slow/retry OPEN-limit recovery.
        # The side already at OPEN remains disabled throughout that recovery.
        if not stepper.moving:
            print(
                "NORMAL OPEN STEP COUNT COMPLETE - ONLY MISSING SIDE(S) WILL SEEK:",
                _missing_endpoint_text("OPEN"),
            )
            _start_master_endpoint_seek("OPEN")
        return

    if gate_state in ("OPEN_LIMIT_SEEK", "OPEN_LIMIT_RETRY"):
        solenoid.set_released(True)
        gate_solenoid_lock_at = 0
        _update_master_endpoint_seek("OPEN")
        return

    if gate_state == "OPEN_RETRY_WAIT":
        solenoid.set_released(True)
        gate_solenoid_lock_at = 0

        _confirm_reached_endpoints("OPEN")
        if _all_endpoint_limits_active("OPEN"):
            _confirm_both_endpoint("OPEN")
            _start_open_seat()
            return

        if gate_open_retry_at and time.ticks_diff(
            time.ticks_ms(), gate_open_retry_at
        ) >= 0:
            _start_open_limit_retry()
        return

    if gate_state == "OPEN_SEAT":
        solenoid.set_released(True)
        gate_solenoid_lock_at = 0
        stepper.update()

        if not stepper.moving:
            if stepper.last_result == "ERROR":
                _gate_error(stepper.last_error or "TB6600 OPEN SEAT ERROR")
            else:
                _finish_open_seat()
        return

    if gate_state == "OPEN_SEAT_SETTLE":
        solenoid.set_released(True)
        gate_solenoid_lock_at = 0
        if (
            gate_open_seat_settle_until
            and time.ticks_diff(time.ticks_ms(), gate_open_seat_settle_until) >= 0
        ):
            _mark_gate_open(open_limit_confirmed=True, timed_out=False)
        return

    if gate_state == "OPEN":
        if gate_open_limit_confirmed and gate_solenoid_lock_at:
            if time.ticks_diff(time.ticks_ms(), gate_solenoid_lock_at) >= 0:
                solenoid.force_locked()
                gate_solenoid_lock_at = 0
                print("SOLENOID: LOCKED 1000 ms AFTER BOTH OPEN LIMITS CONFIRMED")
            else:
                solenoid.set_released(True)
        elif not gate_open_limit_confirmed:
            solenoid.set_released(True)

        # v2.1.1: serve the auto-close rules immediately (the solenoid lock
        # deadline keeps being serviced in WAIT_CLEAR).
        lock_gate()
        return

    if gate_state == "WAIT_CLEAR":
        if gate_solenoid_lock_at and time.ticks_diff(time.ticks_ms(), gate_solenoid_lock_at) >= 0:
            solenoid.force_locked()
            gate_solenoid_lock_at = 0
        # v1.9.8: lock_gate() owns one latched NO_PRESENCE deadline.
        # Repeated calls cannot re-arm a timer that already expired.
        lock_gate()
        return

    if gate_state == "CLOSING":
        _confirm_reached_endpoints("CLOSE")

        if _all_endpoint_limits_active("CLOSE"):
            _confirm_both_endpoint("CLOSE")
            _mark_gate_locked(from_close_limit=True)
            return

        if (
            config["tof"]["enabled"]
            and config["tof"]["reopen_on_obstruction"]
            and _tof_motion_guard_live()
            and not _safe_to_close()
        ):
            _reopen_after_obstruction()
            return

        if stepper.moving:
            if gate_close_limit_deadline and time.ticks_diff(time.ticks_ms(), gate_close_limit_deadline) >= 0:
                _schedule_close_limit_retry("CLOSE WINDOW EXPIRED")
            stepper.update()

        update_gate_limit_switches()
        _confirm_reached_endpoints("CLOSE")

        if _all_endpoint_limits_active("CLOSE"):
            _confirm_both_endpoint("CLOSE")
            _mark_gate_locked(from_close_limit=True)
            return

        if not stepper.moving:
            print("NORMAL CLOSE STEP COUNT COMPLETE - SEEKING MISSING PHYSICAL CLOSE LIMIT(S):", _missing_endpoint_text("CLOSE"))
            _start_master_endpoint_seek("CLOSE")
        return

    if gate_state in ("CLOSE_LIMIT_SEEK", "CLOSE_LIMIT_RETRY"):
        _update_master_endpoint_seek("CLOSE")
        return

    if gate_state == "CLOSE_RETRY_WAIT":
        _confirm_reached_endpoints("CLOSE")
        if _all_endpoint_limits_active("CLOSE"):
            _complete_close_limit_from_recovery()
            return

        if gate_close_retry_at and time.ticks_diff(
            time.ticks_ms(), gate_close_retry_at
        ) >= 0:
            _start_close_limit_retry()
        return

def service_critical_tasks():
    """Keep physical safety responsive during W5500 cooperative waits.

    Follows exactly the same task policy as the main loop. The web server is
    deliberately NOT serviced here (avoids re-entrant HTTP routing).
    """
    if wdt is not None:
        wdt.feed()
    moving = apply_task_policy()

    if moving:
        update_gate_limit_switches()
        update_gate()
        service_tof_motion_safety()
        update_gate_limit_switches()
        if stepper is not None and stepper.moving:
            update_gate()
        return

    service_tap_decision()
    service_tof_runtime(moving=False)

    # Endpoint state must be refreshed BEFORE interpreting a new RFID frame.
    update_gate_limit_switches()
    service_rfid_runtime()
    update_gate()
    apply_task_policy()

    if matrix is not None and not motion_active():
        matrix.update()
    if clock is not None and not motion_active():
        clock.update()


# ============================================================
# RFID ACCESS DECISION
# ============================================================

def cleanup_recent_cards(now):
    if len(recent_cards) <= RECENT_CARD_CACHE_LIMIT:
        return
    oldest_card_id = None
    oldest_age = -1
    for card_id, entry in recent_cards.items():
        try:
            stamp = entry[0]
        except Exception:
            stamp = entry
        age = time.ticks_diff(now, stamp)
        if age > oldest_age:
            oldest_age = age
            oldest_card_id = card_id
    if oldest_card_id is not None:
        try:
            del recent_cards[oldest_card_id]
        except Exception:
            pass


def check_tap_allowed(card_id):
    global anti_tailgate_until
    now = time.ticks_ms()

    # v1.9.8: controller may be online while cooperative boot homing is still
    # pending. Never authorize a card until BOTH CLOSE references are confirmed.
    if not boot_home_complete:
        return False, "BOOT_HOME_PENDING"

    # v2.5.0: the server is deciding the previous tap.
    if tap_decision is not None:
        return False, "SERVER_DECISION_PENDING"

    # v2.1.11: 1 s after an invalid card every tap is ignored (LED red).
    if _deny_lockout_active():
        return False, "DENY_LOCKOUT"

    # A granted transaction owns the lane until BOTH CLOSE switches confirm.
    if granted_passage_active:
        return False, "GRANTED_PASSAGE_ACTIVE"

    if anti_tailgate_until:
        if time.ticks_diff(now, anti_tailgate_until) < 0:
            return False, "ANTI_TAILGATE_DELAY"
        anti_tailgate_until = 0

    if gate_busy:
        return False, "ANTI_TAILGATE_GATE_BUSY"

    if gate_state != "LOCKED":
        return False, "GATE_NOT_LOCKED"

    if GATE_LIMIT_SWITCHES_ENABLED and gate_limits_conflict():
        return False, "LIMIT_CONFLICT"

    if GATE_LIMIT_SWITCHES_REQUIRED and not _all_endpoint_limits_active("CLOSE"):
        return False, "CLOSE_LIMIT_NOT_CONFIRMED"

    if _all_endpoint_limits_active("OPEN") and not _all_endpoint_limits_active("CLOSE"):
        return False, "GATE_ALREADY_OPEN"

    # v1.9.3:
    # A remembered NO_PRESENCE result is not enough. New RFID requires BOTH
    # sensors to be fresh and clear, then remain clear for RFID_CLEAR_STABLE_MS.
    if config["tof"]["enabled"]:
        # v2.0.0: cheap direct checks; the 1000 ms clear dwell is already
        # guaranteed by the scheduler's RFID lockout before UART is parsed.
        presence_state = _presence_state()

        if presence_state == "NOT_READY":
            return False, "TOF_NOT_READY"

        if presence_state == "PRESENCE":
            return False, "PRESENCE_BLOCKED"

        if not _presence_allows_new_tap():
            return False, "TOF_NOT_CLEAR"

    previous = recent_cards.get(card_id)
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

def check_access(card_id):
    global last_processed_tap_time, rfid_deny_until
    global last_rfid_card_id, last_rfid_status, last_rfid_timestamp, last_rfid_lookup_ms
    global last_granted_card_id, last_granted_at

    allowed, reason = check_tap_allowed(card_id)
    if not allowed:
        if reason == "SERVER_DECISION_PENDING":
            return              # repeated frames of the card being decided
        print("RFID IGNORED:", reason, card_id)

        # v2.1.11: silent - the deny beep + red LED already told the user.
        if reason == "DENY_LOCKOUT":
            return

        # Existing forced-open CLOSE recovery stays unchanged.
        if reason == "CLOSE_LIMIT_NOT_CONFIRMED":
            _trigger_close_recovery_from_rfid(card_id)
            return

        # During an active granted passage, a different/new RFID is a direct
        # anti-tailgating attempt: RED X + turnstile warning cadence.
        if reason == "GRANTED_PASSAGE_ACTIVE":
            # The RDM6300 may repeat the same card frame while the card is still
            # physically near the reader. That is not a "new RFID". Ignore the
            # already-granted card for the whole transaction, but warn immediately
            # if a DIFFERENT card is presented before both gate sides close.
            if card_id == granted_passage_card_id:
                return

            print("ANTI-TAILGATING WARNING: NEW RFID WHILE GRANTED USER OWNS THE LANE")
            _warn_blocked_rfid(card_id, reason)
            _request_auto_close_if_clear(
                "INVALID RFID DURING GRANTED PASSAGE " + str(card_id)
            )
            return

        # Lane state itself also blocks a tap.
        if reason in ("PRESENCE_BLOCKED", "TOF_NOT_CLEAR"):
            _warn_blocked_rfid(card_id, reason)
            return

        if reason == "GATE_NOT_LOCKED":
            _warn_blocked_rfid(card_id, reason)
            _request_auto_close_if_clear(
                "INVALID RFID WHILE GATE NOT LOCKED " + str(card_id)
            )
            return

        if reason in ("ANTI_TAILGATE_DELAY", "ANTI_TAILGATE_GATE_BUSY"):
            now = time.ticks_ms()
            immediate_same_granted_card = bool(
                card_id == last_granted_card_id
                and last_granted_at
                and time.ticks_diff(now, last_granted_at) < 1200
            )
            if not immediate_same_granted_card:
                print("ANTI-TAILGATING WARNING: NEXT USER MUST WAIT")
                _warn_blocked_rfid(card_id, reason)
                _request_auto_close_if_clear(
                    "INVALID RFID / " + str(reason) + " " + str(card_id)
                )
        return

    # v2.5.0: the web service decides. The request leaves now (warm
    # keep-alive connection) and apply_tap_decision() acts on the reply.
    start_tap_decision(card_id)


def apply_tap_decision(r):
    """Act on the server's answer (or on the fail-closed timeout)."""
    global last_processed_tap_time, rfid_deny_until
    global last_rfid_card_id, last_rfid_status, last_rfid_timestamp, last_rfid_lookup_ms
    global last_granted_card_id, last_granted_at

    card_id = r["card"]
    authorized = bool(r["granted"])
    now = time.ticks_ms()
    lookup_ms = r["ms"]

    dt = clock.current_datetime()
    status = "GRANTED" if authorized else "DENIED"

    last_processed_tap_time = now
    recent_cards[card_id] = (now, status)
    cleanup_recent_cards(now)

    last_rfid_card_id = card_id
    last_rfid_status = status
    last_rfid_timestamp = format_datetime(dt)
    last_rfid_lookup_ms = lookup_ms

    print()
    print("========================================")
    print("ACCESS RESULT (SERVER)")
    print("========================================")
    print("RFID      :", card_id)
    print("STATUS    :", status)
    print("REASON    :", r["reason"], ("| " + str(r["full_name"])) if r.get("full_name") else "")
    print("SERVER    :", lookup_ms, "ms | HTTP", r["http_status"], "| attempts", r["attempts"],
          ("| " + r["error"]) if r.get("error") else "")
    print("TIMESTAMP :", format_datetime(dt))
    print("PRESENCE  :", _presence_state())

    log_access(card_id, status, dt)

    if authorized:
        last_granted_card_id = card_id
        last_granted_at = now

        gate_opened = unlock_gate()
        if gate_opened:
            # Establish lane ownership first so the following presence-service
            # pass cannot immediately return the LED to standby.
            _begin_granted_passage(card_id)
            matrix.show_result(True)

            if config["gate"]["buzzer_enabled"]:
                buzzer.start_beep(config["gate"]["buzzer_grant_ms"])

            print("GATE CYCLE: STARTED / GREEN GRANT INDICATOR ACTIVE")
        else:
            # Authorization was valid but the physical gate could not accept
            # the OPEN request. Avoid a misleading green result.
            matrix.show_result(False)
            _start_turnstile_warning(
                "AUTHORIZED CARD - GATE COULD NOT OPEN",
                force=True,
            )
            print("GATE CYCLE ERROR: COULD NOT OPEN")
    else:
        rfid_deny_until = ticks_after_ms(RFID_DENY_LOCKOUT_MS)
        matrix.show_result(False, duration_ms=RFID_DENY_LOCKOUT_MS)
        print("RFID DENIED: RED + NEXT TAP ALLOWED IN", RFID_DENY_LOCKOUT_MS, "ms")
        if config["gate"]["buzzer_enabled"]:
            buzzer.start_pattern(
                config["gate"]["buzzer_deny_beeps"],
                config["gate"]["buzzer_deny_on_ms"],
                config["gate"]["buzzer_deny_off_ms"],
            )

        _request_auto_close_if_clear(
            "SERVER DENIED RFID " + str(card_id)
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
# v2.5.0 SERVER-DECIDED ACCESS (no card list on the machine)
# ============================================================

def configure_tap_client():
    tap_client.configure(config["server"], eth=ethernet)


def _eth_link():
    return bool(ethernet is not None and ethernet.ready and ethernet.is_connected())


def start_tap_decision(card_id):
    """Send the tap to the server now; the gate acts when the reply comes."""
    global tap_decision, tap_seq
    tap_seq += 1
    tap_id = "{}-{}".format(BOOT_TAG, tap_seq)
    tap_decision = {"card": card_id, "started": time.ticks_ms(), "tap_id": tap_id}
    print("RFID -> SERVER:", card_id, "| tap", tap_id, "| link", "UP" if _eth_link() else "DOWN",
          "| warm" if tap_client.state == "READY" else "| cold")
    tap_client.request(card_id, tap_id, link_up=_eth_link())
    service_tap_decision()


def service_tap_decision(moving=False):
    """One non-blocking step of the server connection; apply a finished
    decision. Also keeps the warm connection to the server alive."""
    global tap_decision
    if moving:
        # No tap can be decided while the motors run; keep the warm socket
        # maintenance out of the motion pass.
        return
    tap_client.service(_eth_link())
    r = tap_client.take_result()
    if r is None:
        return
    tap_decision = None
    apply_tap_decision(r)


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

    eth_ifconfig = ethernet.ifconfig() if ethernet else None
    eth_link = ethernet.is_connected() if ethernet else False
    motor_status = stepper.status()
    solenoid_status = solenoid.status()
    tof_status = tof.status() if tof is not None else {
        "enabled": False,
        "ready": False,
        "global_state": "UNAVAILABLE",
        "safe_to_tap": True,
        "safe_to_close": True,
    }

    # Application-level presence semantics override the old sensor-only tap flag.
    integration_presence_state = _presence_state()
    tof_status["safe_to_tap"] = bool(_rfid_lane_ready())
    tof_status["granted_passage_active"] = bool(granted_passage_active)
    tof_status["granted_passage_card_id"] = granted_passage_card_id
    tof_status["granted_person_seen"] = bool(granted_person_seen)
    tof_status["integration_state"] = integration_presence_state
    tof_status["passage"] = {
        "s1_entry_seen": bool(passage_s1_seen),
        "s2_exit_seen": bool(passage_s2_seen),
        "exit_confirmed": bool(passage_exit_confirmed),
        "tailgate_count": int(passage_tailgate_count),
    }
    tof_status["clear_close_remaining_ms"] = (
        max(0, time.ticks_diff(presence_clear_close_at, time.ticks_ms()))
        if presence_clear_close_at else 0
    )
    tof_status["tap_rule"] = "GATE LOCKED + NO_PRESENCE {} ms + BOTH CLOSE LIMITS".format(
        _rfid_rearm_ms()
    )

    return {
        "firmware": FIRMWARE_VERSION,
        "tasks": task_status_payload(),
        "time": format_datetime(clock.current_datetime()),
        "rtc_ready": clock.ready,
        "sd_ready": sd_ready,
        "gate_busy": gate_busy,
        "boot_home_complete": bool(boot_home_complete),
        "tof": tof_status,
        "gate": {
            "boot_home_complete": bool(boot_home_complete),
            "boot_home_attempt_count": int(boot_home_attempt_count),
            "boot_home_retry_remaining_ms": (
                max(
                    0,
                    time.ticks_diff(
                        boot_home_retry_at,
                        time.ticks_ms(),
                    ),
                )
                if boot_home_retry_at
                else 0
            ),
            "state": gate_state,
            "busy": gate_busy,
            "motor_enabled": config["gate"]["motor_enabled"],
            "motor_moving": motor_status["moving"],
            "motor_initialized": motor_status["initialized"],
            "motor_angle": motor_status["angle"],
            "motor1_angle": motor_status.get("motor1_angle", motor_status["angle"]),
            "motor2_angle": motor_status.get("motor2_angle", motor_status["angle"]),
            "motor_move_mode": motor_status.get("move_mode", "NORMAL"),
            "motor_pulse_engine": motor_status.get("pulse_engine", "BITBANG"),
            "motor_rmt_enabled": motor_status.get("rmt_enabled", False),
            "motor_rmt_pending_m1": motor_status.get("rmt_pending_motor1_pulses", 0),
            "motor_rmt_pending_m2": motor_status.get("rmt_pending_motor2_pulses", 0),
            "motor_target_angle": motor_status["target_angle"],
            "motor_progress_percent": motor_status["progress_percent"],
            "motor_last_result": motor_status["last_result"],
            "motor_last_error": motor_status["last_error"],
            "open_angle": config["gate"]["open_angle"],
            "closed_angle": config["gate"]["closed_angle"],
            "solenoid_released": solenoid_status["released"],
            "motor_driver_count": 2,
            "motor2_opposite": motor_status.get("motor2_opposite", True),
            "driver1": motor_status.get("driver1", {}),
            "driver2": motor_status.get("driver2", {}),
            "limits": gate_limit_status(),
            "motor_start_delay_ms": config["gate"]["motor_start_delay_ms"],
            "motor_start_delay_remaining_ms": (
                max(0, time.ticks_diff(gate_motor_start_at, time.ticks_ms()))
                if gate_state in ("UNLOCKING", "REUNLOCKING") and gate_motor_start_at
                else 0
            ),
            "latch_release_profile_version": config["gate"]["latch_release_profile_version"],
            "latch_release_jog_enabled": config["gate"]["latch_release_jog_enabled"],
            "latch_release_jog_degrees": config["gate"]["latch_release_jog_degrees"],
            "latch_release_jog_delay_us": config["gate"]["latch_release_jog_delay_us"],
            "latch_release_shake_cycles": config["gate"]["latch_release_shake_cycles"],
            "latch_release_shake_pause_ms": config["gate"]["latch_release_shake_pause_ms"],
            "latch_release_shake_leg": gate_latch_shake_leg,
            "latch_release_shake_total_legs": gate_latch_shake_total_legs,
            "latch_release_settle_ms": config["gate"]["latch_release_settle_ms"],
            "latch_release_settle_remaining_ms": (
                max(0, time.ticks_diff(gate_latch_settle_until, time.ticks_ms()))
                if gate_state == "LATCH_SETTLE" and gate_latch_settle_until
                else 0
            ),
            # Backward-compatible key remains fixed at 1000 ms.
            "solenoid_unlock_ms": config["gate"]["solenoid_unlock_ms"],
            "solenoid_policy": "FIRST OPEN MOTOR STOPS/WAITS; LATE MOTOR CONTINUES; CLOSE TIMER STARTS ONLY AFTER BOTH GPIO{}+GPIO{} OPEN LIMITS".format(GATE_MOTOR1_OPEN_LIMIT_PIN, GATE_MOTOR2_OPEN_LIMIT_PIN),
            "open_soft_land_enabled": bool(TB6600_OPEN_SOFT_LAND_ENABLED),
            "open_decel_start_percent": int(TB6600_OPEN_DECEL_START_PERCENT),
            "open_final_delay_us": int(TB6600_OPEN_FINAL_DELAY_US),
            "open_seat_enabled": bool(GATE_OPEN_SEAT_ENABLED),
            "open_seat_degrees": float(GATE_OPEN_SEAT_DEGREES),
            "open_seat_delay_us": int(GATE_OPEN_SEAT_DELAY_US),
            "open_seat_settle_ms": int(GATE_OPEN_SEAT_SETTLE_MS),
            "solenoid_lock_after_open_ms": int(SOLENOID_LOCK_AFTER_OPEN_DELAY_MS),
            "solenoid_lock_remaining_ms": (
                max(0, time.ticks_diff(gate_solenoid_lock_at, time.ticks_ms()))
                if gate_solenoid_lock_at
                else 0
            ),
            "solenoid_remaining_ms": solenoid_status["remaining_ms"],
            "open_limit_confirmed": bool(gate_open_limit_confirmed),
            "open_limit_timed_out": bool(gate_open_limit_timed_out),
            "open_waiting_for_motor": (
                1 if gate_state == "OPEN_WAIT_MOTOR1"
                else 2 if gate_state == "OPEN_WAIT_MOTOR2"
                else 0
            ),
            "open_retry_count": int(gate_open_retry_count),
            "open_retry_reason": gate_open_retry_reason,
            "open_retry_remaining_ms": (
                max(0, time.ticks_diff(gate_open_retry_at, time.ticks_ms()))
                if gate_open_retry_at else 0
            ),
            "open_limit_wait_remaining_ms": (
                max(0, time.ticks_diff(gate_open_limit_deadline, time.ticks_ms()))
                if gate_open_limit_deadline else 0
            ),
            "close_limit_wait_remaining_ms": (
                max(0, time.ticks_diff(gate_close_limit_deadline, time.ticks_ms()))
                if gate_close_limit_deadline else 0
            ),
            "limit_find_max_ms": int(GATE_LIMIT_FIND_MAX_MS),
            "limit_retry_interval_ms": int(GATE_LIMIT_RETRY_INTERVAL_MS),
            "close_retry_count": int(gate_close_retry_count),
            "close_retry_reason": str(gate_close_retry_reason),
            "close_retry_remaining_ms": (
                max(0, time.ticks_diff(gate_close_retry_at, time.ticks_ms()))
                if gate_close_retry_at else 0
            ),
            "anti_tailgate_next_tap_delay_sec": float(config["tap"]["next_card_delay_sec"]),
            "anti_tailgate_remaining_ms": (
                max(0, time.ticks_diff(anti_tailgate_until, time.ticks_ms()))
                if anti_tailgate_until else 0
            ),
        },
        "last_rfid": {
            "card_id": last_rfid_card_id,
            "status": last_rfid_status,
            "timestamp": last_rfid_timestamp,
            "lookup_ms": last_rfid_lookup_ms,
        },
        "config_mode": {
            "active": bool(config_mode_active),
            "since_ms": (time.ticks_diff(time.ticks_ms(), config_mode_since)
                         if config_mode_active else 0),
            "hold_ms": CONFIG_MODE_HOLD_MS,
            "disabled": ["VL53L0X", "RFID", "NEMA17+TB6600"] if config_mode_active else [],
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
            "link_changes": eth_link_changes,
            "web_ui": ("http://{}/".format(eth_ifconfig[0]) if (eth_ifconfig and WEB_BIND_ALL_INTERFACES and web_server_ready) else ""),
            "ifconfig": eth_ifconfig,
            "error": ethernet.last_error if ethernet else "",
            "driver": ethernet.status() if ethernet else {},
        },
        "server": tap_client.status(),
    }


def validate_and_update_config(incoming):
    global config

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
    if tof is not None:
        tof.configure(config["tof"])
    if not gate_busy:
        solenoid.force_locked()
    configure_tap_client()
    return True


def server_self_test():
    """'Test Server' button: health URL reachability through the W5500 and
    the tap client's readiness (no test tap is sent - it would be logged)."""
    result = {"ok": False, "steps": []}
    url = config["server"]["health_url"].strip()
    step = {"name": "health", "url": url, "ok": False}
    started = time.ticks_ms()
    try:
        status, _h, body = ethernet.http_request(
            url, method="GET", timeout_ms=config["server"]["timeout_ms"], max_body=4096,
            service_callback=service_critical_tasks)
        step.update({"status": status, "ok": 200 <= status < 300,
                     "preview": bytes(body[:200]).decode("utf-8", "ignore")})
    except Exception as e:
        step.update({"status": 0, "error": repr(e)})
    step["ms"] = time.ticks_diff(time.ticks_ms(), started)
    result["steps"].append(step)
    tap = tap_client.status()
    result["steps"].append({"name": "tap decisions", "ok": tap["ready"],
                            "error": tap["not_ready_reason"], "state": tap["state"],
                            "url": tap["url"]})
    result["tap"] = tap
    result["ok"] = all(x.get("ok") for x in result["steps"])
    result["status"] = step.get("status", 0)
    result["url"] = url
    return result


def route_http(method, target, body_bytes):
    global pending_reboot_at, config_tof_capture_until, config_tof_capture_started

    path, query_text = split_path_query(target)
    query = parse_query(query_text)

    if path == "/favicon.ico":
        return 204, "text/plain", ""

    if path == "/api/status" and method == "GET":
        return json_response(get_status_payload())

    if path == "/api/tof/status" and method == "GET":
        tof_payload = _presence_snapshot()
        tof_payload["integration_state"] = _presence_state()
        tof_payload["safe_to_tap"] = bool(_rfid_lane_ready())
        tof_payload["tap_rule"] = "GATE LOCKED + NO_PRESENCE {} ms + BOTH CLOSE LIMITS".format(
            _rfid_rearm_ms()
        )
        tof_payload["tasks"] = task_status_payload()
        tof_payload["granted_passage_active"] = bool(granted_passage_active)
        tof_payload["granted_passage_card_id"] = granted_passage_card_id
        tof_payload["granted_person_seen"] = bool(granted_person_seen)
        tof_payload["clear_close_remaining_ms"] = (
            max(0, time.ticks_diff(presence_clear_close_at, time.ticks_ms()))
            if presence_clear_close_at else 0
        )
        return json_response(tof_payload)

    if path == "/api/tof/background/capture" and method == "POST":
        if tof is None or not tof.hardware_ready:
            return json_response({"ok": False, "error": "VL53L0X not ready"}, 409)
        if gate_busy or gate_state != "LOCKED" or motion_active():
            return json_response({"ok": False, "error": "Gate must be LOCKED and idle"}, 409)
        if (
            config["tof"].get("sensor1_background_mm", 0)
            or config["tof"].get("sensor2_background_mm", 0)
        ):
            return json_response({
                "ok": False,
                "error": "Configured sensorN_background_mm overrides capture; set it to 0 first",
            }, 409)
        if config_mode_active:
            tof.resume("BACKGROUND_CAPTURE")
            config_tof_capture_started = time.ticks_ms()
            config_tof_capture_until = ticks_after_ms(CONFIG_MODE_TOF_CAPTURE_TIMEOUT_MS)
        tof.request_background_capture()
        _set_rfid_lockout("BACKGROUND_CAPTURE")
        return json_response({
            "ok": True,
            "message": "Background capture started - keep the lane EMPTY. Saved automatically.",
        })

    if path == "/api/config" and method == "GET":
        return json_response(config)

    if path == "/api/config" and method == "POST":
        try:
            incoming = json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
            validate_and_update_config(incoming)
            return json_response({
                "ok": True,
                "message": "Configuration saved. ToF/gate/tap settings apply immediately; AP/Ethernet changes require restart.",
                "reboot_required": True,
            })
        except Exception as e:
            return json_response({"ok": False, "error": str(e)}, 400)

    if path == "/api/server/test" and method == "POST":
        if not (ethernet and ethernet.ready and ethernet.is_connected()):
            return json_response({"ok": False, "error": "W5500 link down - check Ethernet cable"}, 502)
        return json_response(server_self_test())

    if path == "/api/reboot" and method == "POST":
        pending_reboot_at = ticks_after_ms(700)
        return json_response({"ok": True, "message": "ESP32 restarting..."})

    return json_response({"ok": False, "error": "Not found"}, 404)


def initialize_web_server():
    global web_server, web_server_ready
    web_server_ready = False
    try:
        ip = "0.0.0.0" if WEB_BIND_ALL_INTERFACES else ap.ifconfig()[0]
        web_server = CooperativeWebServer(
            route_callback=route_http,
            root_file=WEBUI_FILE,
            file_chunk_size=WEBUI_CHUNK_SIZE,
            recv_chunk_size=WEB_RECV_CHUNK_SIZE,
            send_chunk_size=WEB_SEND_CHUNK_SIZE,
            client_timeout_ms=WEB_CLIENT_TIMEOUT_MS,
        )
        web_server.max_dispatch_defer_ms = int(WEB_MAX_DISPATCH_DEFER_MS)
        web_server_ready = web_server.initialize(ip, 80)
        # v2.4.1: the W5500 runs its own TCP/IP, so the Ethernet side of the
        # page is served by hardware listening sockets.
        if ethernet is not None and ethernet.ready:
            listener = ethernet.web_listener(80)
            if listener is not None:
                web_server.add_listener(listener)
                print("CONFIG WEB ON ETHERNET: http://{}/".format(ethernet.ifconfig()[0]))
        return web_server_ready
    except Exception as e:
        print("WEB SERVER ERROR:", repr(e))
        web_server_ready = False
        return False


def service_web_server(moving=False):
    """Web UI/API on Wi-Fi AP + W5500.

    Stationary: full service. Moving: socket I/O only every
    WEB_MOTION_SERVICE_INTERVAL_MS and no request routing (held until stop).
    """
    global web_motion_next_service_at

    if not web_server_ready or web_server is None:
        return

    if moving:
        now = time.ticks_ms()
        if (
            web_motion_next_service_at
            and time.ticks_diff(now, web_motion_next_service_at) < 0
        ):
            return
        web_motion_next_service_at = time.ticks_add(
            now, int(WEB_MOTION_SERVICE_INTERVAL_MS)
        )
        web_server.update(allow_dispatch=False)
        return

    web_server.update(allow_dispatch=True)


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
print("2x VL53L0X SAFETY")
print("MICRO SD SPI")
print("W5500 ETHERNET")
print("1x ULN2003A + 2x TB6600 + 2x NEMA17")
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

# v2.0.0 VL53L0X: hardware init + continuous ranging start. The calibrated
# background is loaded (no relearning). If init fails at 400 kHz the shared
# bus is rebuilt at the 100 kHz fallback and the init is retried once.
print("I2C0 FREQUENCY:", i2c_frequency_active, "Hz")
tof.initialize(config["tof"])
if (
    config["tof"]["enabled"]
    and not tof.hardware_ready
    and i2c_frequency_active > int(I2C_FALLBACK_FREQUENCY)
):
    print("VL53L0X INIT FAILED AT", i2c_frequency_active, "Hz - RETRYING AT", I2C_FALLBACK_FREQUENCY, "Hz")
    if rebuild_shared_i2c(I2C_FALLBACK_FREQUENCY, "VL53L0X boot fallback"):
        if not clock.ready:
            clock.initialize()
        tof.initialize(config["tof"])
memory_report("VL53L0X")

initialize_sd()
memory_report("SD")


# v2.3.0: Wi-Fi AP and the Machine Configuration Web stay OFF until GPIO19
# is held for CONFIG_MODE_HOLD_MS (see enter_config_mode()).
disable_access_point()
print("MACHINE CONFIGURATION WEB: LOCKED - HOLD GPIO{} FOR {} ms TO UNLOCK".format(
    MANUAL_OPEN_BUTTON_PIN, CONFIG_MODE_HOLD_MS))
memory_report("WIFI AP OFF")

ethernet.initialize(config["ethernet"])
memory_report("W5500")

# Prepare homing only AFTER the rest of the controller is alive. If one CLOSE
# limit is missing/noisy, the main loop remains responsive and retries homing.
prepare_boot_home_nonblocking()

if boot_home_complete:
    matrix.start_standby()
else:
    matrix.clear()

gc.collect()

configure_tap_client()
print("ACCESS DECISION    : SERVER ->", tap_client.url, "| gate", tap_client.gate_id,
      "| timeout", tap_client.decision_timeout_ms, "ms (fail-closed) |",
      "READY" if tap_client.ready_reason() == "" else "NOT READY: " + tap_client.ready_reason())

print()
print("========================================")
print("SYSTEM READY - CORE SERVICES ONLINE")
print("========================================")
print("GATE HOME READY:", boot_home_complete)
print(
    "GATE HOME STATE:",
    "READY" if boot_home_complete else "NON-BLOCKING HOMING / RFID BLOCKED",
)
print("TIME:", format_datetime(clock.current_datetime()))
print("RTC READY:", clock.ready)
print("VL53L0X READY:", tof.ready)
print("VL53L0X STATE:", tof.status()["global_state"])
print("I2C0 FREQUENCY     :", i2c_frequency_active, "Hz")
print("TASK POLICY        : MOTION -> ToF+RFID PAUSED, LED+WEB+SD/SYNC DEFERRED")
print("RFID RE-ARM RULE   : GATE LOCKED + NO_PRESENCE", _rfid_rearm_ms(), "ms CONTINUOUS")
print("ToF CLOSE GUARD    :", config["tof"].get("close_motion_guard", False))
print("CONFIG WEB         : LOCKED | HOLD GPIO{} {} ms -> AP {} + http://{}/".format(
    MANUAL_OPEN_BUTTON_PIN, CONFIG_MODE_HOLD_MS, config["ap"]["ssid"], config["ap"]["ip"]))
print("RFID PRESENCE RULE: NO_PRESENCE + BOTH CLOSE LIMITS")
print("PRESENCE RULE      : BLOCK NEW RFID / WARN IF NO ACTIVE GRANT")
print("NO_PRESENCE RULE   : CLOSE ARMED AFTER", PRESENCE_CLEAR_CLOSE_DELAY_MS, "ms CLEAR")
print("WARN CADENCE       :", TURNSTILE_WARN_BEEPS, "x", TURNSTILE_WARN_ON_MS, "ms ON /", TURNSTILE_WARN_OFF_MS, "ms OFF")
print("SD READY:", sd_ready)
print("MOTOR ENABLED:", config["gate"]["motor_enabled"])
print("MOTOR CLOSED ANGLE:", config["gate"]["closed_angle"])
print("MOTOR OPEN ANGLE:", config["gate"]["open_angle"])
print("MOTOR DIRECTION INVERTED:", config["gate"]["direction_inverted"])
print("LATCH SHAKE CYCLES:", config["gate"]["latch_release_shake_cycles"])
print("SOLENOID FIXED RULE: LOCK", SOLENOID_LOCK_AFTER_OPEN_DELAY_MS, "ms AFTER FULL OPEN")
print("M1 CLOSE LIMIT     :", gate_close_limit_active, "GPIO", GATE_MOTOR1_CLOSE_LIMIT_PIN)
print("M1 OPEN LIMIT      :", gate_open_limit_active, "GPIO", GATE_MOTOR1_OPEN_LIMIT_PIN)
print("M2 CLOSE LIMIT     :", gate_close_limit2_active, "GPIO", GATE_MOTOR2_CLOSE_LIMIT_PIN)
print("M2 OPEN LIMIT      :", gate_open_limit2_active, "GPIO", GATE_MOTOR2_OPEN_LIMIT_PIN)
print("BOTH CLOSE READY   :", _all_endpoint_limits_active("CLOSE"))
print("BOTH OPEN READY    :", _all_endpoint_limits_active("OPEN"))
print("LIMIT FAULT        :", gate_limits_conflict(), gate_limit_conflict_detail())
print("LIMITS REQUIRED    :", GATE_LIMIT_SWITCHES_REQUIRED)
print("LIMIT FIND MAX     :", GATE_LIMIT_FIND_MAX_MS, "ms FIXED PER ATTEMPT")
print("LIMIT RETRY        :", GATE_LIMIT_RETRY_INTERVAL_MS, "ms FIXED")
print("RFID CLOSE RECOVERY: ENABLED / FIXED")
print("OPEN LIMIT MAX WAIT:", GATE_OPEN_LIMIT_MAX_WAIT_MS, "ms FIXED")
print("CLOSE LIMIT MAX WAIT:", GATE_CLOSE_LIMIT_MAX_WAIT_MS, "ms FIXED")
print("MOTOR PULSE FALLBACK:", TB6600_MAX_PULSES_PER_UPDATE, "pulses max /", TB6600_MAX_BURST_US, "us max")
print("MOTOR RMT REQUESTED:", TB6600_USE_HARDWARE_RMT)
print("MOTOR RMT CHANNELS :", TB6600_RMT_CHANNEL_1, "/", TB6600_RMT_CHANNEL_2)
print("MOTOR RMT CHUNK    :", TB6600_RMT_CHUNK_PULSES, "pulses /", TB6600_RMT_CHUNK_MAX_US, "us max")
print("MOTOR RMT WATCHDOG :", TB6600_RMT_STALL_TIMEOUT_MS, "ms -> BITBANG fallback")
print("MOTOR PULSE ENGINE :", stepper.status().get("pulse_engine", "UNKNOWN"))
print("OPEN SOFT LAND     :", TB6600_OPEN_SOFT_LAND_ENABLED, "from", TB6600_OPEN_DECEL_START_PERCENT, "% ->", TB6600_OPEN_FINAL_DELAY_US, "us")
print("OPEN SEAT PRELOAD  :", GATE_OPEN_SEAT_ENABLED, GATE_OPEN_SEAT_DEGREES, "deg @", GATE_OPEN_SEAT_DELAY_US, "us")
print("CONFIG AP: OFF (machine configuration mode locked)")
try:
    print("W5500 IP:", ethernet.ifconfig()[0])
except Exception:
    print("W5500 IP: NOT READY")
print()
print("GPIO MAP")
print("----------------------------------------")
print("ULN IN1 / M1 STEP -> GPIO", TB6600_1_STEP_PIN)
print("ULN IN2 / M1 DIR  -> GPIO", TB6600_1_DIR_PIN)
print("ULN OUT1 -> TB6600 #1 PUL-")
print("ULN OUT2 -> TB6600 #1 DIR-")
print("ULN IN3 / M2 STEP -> GPIO", TB6600_2_STEP_PIN)
print("ULN IN4 / M2 DIR  -> GPIO", TB6600_2_DIR_PIN)
print("ULN OUT3 -> TB6600 #2 PUL-")
print("ULN OUT4 -> TB6600 #2 DIR-")
print("CLOSE LIMIT        -> GPIO", GATE_CLOSE_LIMIT_PIN)
print("OPEN LIMIT         -> GPIO", GATE_OPEN_LIMIT_PIN)
print("W5500 SCK        -> GPIO", W5500_SCK_PIN)
print("W5500 MOSI       -> GPIO", W5500_MOSI_PIN)
print("W5500 MISO       -> GPIO", W5500_MISO_PIN)
print("W5500 CS         -> GPIO", W5500_CS_PIN)
print("I2C SDA DS3231/TOF -> GPIO", RTC_SDA_PIN)
print("I2C SCL DS3231/TOF -> GPIO", RTC_SCL_PIN)
print("VL53 XSHUT #1      -> GPIO", TOF_XSHUT1_PIN)
print("VL53 XSHUT #2      -> GPIO", TOF_XSHUT2_PIN)
print("SD CS            -> GPIO", SD_CS_PIN)
print("SD MOSI          -> GPIO", SD_MOSI_PIN)
print("SD SCK           -> GPIO", SD_SCK_PIN)
print("SD MISO          -> GPIO", SD_MISO_PIN)
print("W5500 INT        -> GPIO", W5500_INT_PIN)
print("SOLENOID RELAY   -> GPIO", SOLENOID_RELAY_PIN)
print("RDM6300 TX -> RX  -> GPIO", RFID_RX_PIN)
print("BUZZER           -> GPIO", BUZZER_PIN)
print("WS2812B DIN      -> GPIO", LED_PIN)
print("W5500 RST        -> GPIO", W5500_RST_PIN)
print("RDM6300 RX-ONLY   -> UART TX NOT ASSIGNED")
print("========================================")
print("Waiting for RFID...")


# ============================================================
# MAIN LOOP
# ============================================================

# Serial-safe exception reporting. A persistent fault must not print at 50+ lines/s
# because USB/serial backpressure can make the VM look completely frozen.
if HARDWARE_WATCHDOG_ENABLED:
    try:
        wdt = machine.WDT(timeout=int(HARDWARE_WATCHDOG_TIMEOUT_MS))
        print("HARDWARE WATCHDOG  : ENABLED", HARDWARE_WATCHDOG_TIMEOUT_MS, "ms")
    except Exception as exc:
        wdt = None
        print("HARDWARE WATCHDOG  : FAILED", repr(exc))
else:
    print("HARDWARE WATCHDOG  : DISABLED (config HARDWARE_WATCHDOG_ENABLED)")
print("I2C DRIVER         :", "SoftI2C (timeout-bounded)" if (I2C_USE_SOFTWARE and SoftI2C is not None) else "hardware I2C")

_last_system_error_text = ""
_last_system_error_print_at = 0
_system_error_repeat_count = 0

while True:
    try:
        # ========================================================
        # v2.0.0 TASK POLICY - decided once per pass
        # ========================================================
        # Pauses/resumes VL53L0X, enables/disables RFID and maintains the
        # PRESENCE lockout + 1000 ms re-arm. Returns True while moving.
        service_loop_monitor()
        if wdt is not None:
            wdt.feed()
        moving = apply_task_policy()
        service_manual_button()
        # The button can start UNLOCKING; apply the current task policy now.
        moving = apply_task_policy()

        # v2.3.0 MACHINE CONFIGURATION MODE: AP + web only; ToF, RFID and the
        # motors are never serviced, so nothing can move the barrier.
        if config_mode_active:
            service_config_mode_pass()
            time.sleep_ms(MAIN_LOOP_SLEEP_MS)
            continue

        # ========================================================
        # COOPERATIVE BOOT-HOME (v1.9.8 behavior preserved)
        # ========================================================
        if not boot_home_complete:
            # ToF runs only while the gate is stationary (between attempts),
            # so NO_PRESENCE can be checked before each homing move.
            if not moving:
                service_tof_recovery()
                service_tof_runtime(run_presence_flow=False)

            service_boot_home_nonblocking()
            apply_task_policy()

            # Homing may have started during this pass. Use the current motor
            # state before doing any optional work or taking the idle sleep.
            if motion_active():
                _motion_gc_guard()
                time.sleep_us(MOTOR_LOOP_SLEEP_US)
                continue

            flush_motion_prints()
            matrix.update()
            if not moving:
                clock.update()

            service_ethernet_link()
            service_web_server(moving=motion_active())
            service_tap_decision()

            time.sleep_ms(MAIN_LOOP_SLEEP_MS)
            continue

        # ========================================================
        # MOTION PASS - TB6600 / limits highest priority
        # ========================================================
        # ToF and RFID are PAUSED (zero I2C / UART work). SD, sync processing
        # RTC, LED rendering and network polling are deferred. Hardware STEP
        # timing is exact within a batch, but optional Python work can delay
        # the next batch and make loaded motors repeatedly decelerate/restart.
        if moving:
            update_gate_limit_switches()
            update_gate()

            # No-op unless tof.close_motion_guard=True during CLOSE motion.
            service_tof_motion_safety()

            update_gate_limit_switches()
            if stepper is not None and stepper.moving:
                update_gate()


            _motion_gc_guard()
            time.sleep_us(MOTOR_LOOP_SLEEP_US)
            continue

        # ========================================================
        # STATIONARY PASS
        # ========================================================
        # PRIORITY 1: presence sensing + access/gate hardware.
        _t = time.ticks_us()
        service_tof_recovery()
        _t = _pass_mark('service_tof_recovery', _t)
        service_tof_runtime(moving=False)
        _t = _pass_mark('service_tof_runtime', _t)
        update_gate_limit_switches()
        _t = _pass_mark('update_gate_limit_switches', _t)
        service_rfid_runtime()
        _t = _pass_mark('service_rfid_runtime', _t)
        service_tap_decision()
        _t = _pass_mark('service_tap_decision', _t)
        update_gate()
        _t = _pass_mark('update_gate', _t)

        # If update_gate()/RFID just started motion, pause ToF/RFID now
        # instead of on the next pass.
        if apply_task_policy():
            time.sleep_us(MOTOR_LOOP_SLEEP_US)
            continue

        # v2.1.3: print what was queued while the motors ran.
        flush_motion_prints()
        _t = _pass_mark('flush_motion_prints', _t)

        # PRIORITY 2: local visual/clock services.
        matrix.update()
        _t = _pass_mark('matrix.update', _t)
        clock.update()
        _t = _pass_mark('clock.update', _t)

        # PRIORITY 3: deferred flash/SD writes (gate idle only).
        service_access_log_queue()
        _t = _pass_mark('service_access_log_queue', _t)
        service_tof_persistence()
        _t = _pass_mark('service_tof_persistence', _t)

        # PRIORITY 4: W5500 link + Web UI/API (AP + Ethernet).
        service_ethernet_link()
        _t = _pass_mark('service_ethernet_link', _t)
        service_web_server(moving=False)
        _t = _pass_mark('service_web_server', _t)


        service_i2c_health()
        _t = _pass_mark('service_i2c_health', _t)

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
        error_text = repr(e)
        now_ms = time.ticks_ms()
        _system_error_repeat_count += 1

        should_print = (
            error_text != _last_system_error_text
            or not _last_system_error_print_at
            or time.ticks_diff(now_ms, _last_system_error_print_at) >= 1000
        )

        if should_print:
            print()
            print(
                "SYSTEM ERROR:",
                error_text,
                "| occurrences since last report:",
                _system_error_repeat_count,
            )
            _last_system_error_text = error_text
            _last_system_error_print_at = now_ms
            _system_error_repeat_count = 0

        # Yield long enough for Ctrl+C/USB and cooperative firmware work instead
        # of entering a high-rate exception/print loop.
        time.sleep_ms(100)
