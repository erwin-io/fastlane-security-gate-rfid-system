# ============================================================
# FASTLANE v2.0.0 - DUAL VL53L0X CONTINUOUS PRESENCE COMPONENT
# ============================================================
#
# WHAT CHANGED FROM v1.9.8 (and why the old setup was slow)
# ------------------------------------------------------------
# v1.9.8 used ONE single-shot measurement at a time and alternated the two
# sensors:  S1 start -> wait ~33 ms -> S1 read -> S2 start -> wait -> S2 read.
# Every start re-wrote the 7-register "stop variable" preamble, results were
# polled on a 5 ms service grid, and presence needed a 3-sample average plus
# 2 confirmations. Each sensor therefore produced only ~10-13 samples/s and a
# PRESENCE/NO_PRESENCE change took ~150-250 ms to appear.
#
# v2.0.0 uses the continuous back-to-back mode exposed by the Pololu /
# kapetan MicroPython_VL53L0X driver:
#   - BOTH sensors range IN PARALLEL in hardware (no alternation).
#   - No per-sample start preamble. Reading a result is 1 status byte +
#     2 distance bytes + 1 interrupt-clear write.
#   - Predictive polling: after a sample arrives the next poll is scheduled
#     just before the next sample is due, so idle I2C traffic is ~2 polls
#     per sample instead of continuous polling.
#   - Preallocated I2C buffers (readfrom_mem_into) -> no heap allocation per
#     sample, so high-rate ranging does not provoke GC pauses.
#   - Median-of-3 spike rejection + 1/2 sample confirmation.
#
# NON-BLOCKING GUARANTEES
# ------------------------------------------------------------
#   - update() performs at most ONE short I2C poll per sensor and returns.
#   - Nothing in update()/pause()/resume() waits for a measurement.
#   - Boot-only init waits are bounded by io_timeout_ms (raises, never hangs).
#   - A sensor that stops producing samples is restarted by a throttled
#     watchdog instead of being waited on.
#
# FIXED CALIBRATED BACKGROUND (replaces boot-time clear-model learning)
# ------------------------------------------------------------
# Per sensor, the clear/background distance is resolved ONCE at boot in this
# order and is NOT relearned on every boot:
#   1) tof.sensorN_background_mm > 0 in config      (real distance, mm)
#   2) /presence_range_calibration.json            (tools/presence_calibrate.py
#      option 2, background_calibrated = true)
#   3) /tof_background.json                        (previous one-time capture)
#   4) one-time runtime capture (lane must be empty), then saved to (3)
#
# Offsets (sensorN_offset_mm) always come from config; the current values
# -43.375 / -42.0 are the dual_sensor_multi_point_constant_offset results.
#
# PRESENCE  : median distance <= background - presence_delta_mm
# NO_PRESENCE: median distance >= background - release_delta_mm (hysteresis)
# A "no target / out of range" return counts as CLEAR (beam reached past
# the far wall), never as presence.
#
# DRIVER PROFILE
# ------------------------------------------------------------
# "legacy" (DEFAULT): exactly the init sequence the offsets/background were
#     calibrated with. Keeps the existing calibration valid.
# "full": full Pololu/ST init (tuning table + SPAD + VHV/phase calibration).
#     Generally more accurate, BUT changes the raw distance bias, so the
#     offset + background calibration MUST be re-run after switching.
#
# Low-level register code is adapted from the MIT-licensed
# kapetan/MicroPython_VL53L0X driver (Tony DiCola / Adafruit, based on the
# Pololu vl53l0x-arduino library).
# ============================================================

from machine import Pin
import time

try:
    import ujson as json
except ImportError:
    import json

try:
    import os
except ImportError:
    os = None


# ------------------------------------------------------------
# Legacy constants kept for backward compatibility / tools.
# ------------------------------------------------------------
CLEAR = 0
POSSIBLE = 1
PRESENCE = 2
UNCERTAIN = 3


def state_name(state):
    if state == CLEAR:
        return "CLEAR"
    if state == POSSIBLE:
        return "POSSIBLE"
    if state == PRESENCE:
        return "PRESENCE"
    if state == UNCERTAIN:
        return "UNCERTAIN"
    return "UNKNOWN"


def _median(values):
    if not values:
        return None
    temp = sorted(values)
    count = len(temp)
    middle = count // 2
    if count % 2:
        return temp[middle]
    return (temp[middle - 1] + temp[middle]) / 2


def _percentile(values, fraction):
    values = sorted(float(v) for v in values if v is not None)
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * float(fraction)
    low_index = int(position)
    high_index = min(low_index + 1, len(values) - 1)
    weight = position - low_index
    return values[low_index] * (1.0 - weight) + values[high_index] * weight


# ============================================================
# LOW-LEVEL REGISTER MAP
# ============================================================

_SYSRANGE_START = 0x00
_SYSTEM_SEQUENCE_CONFIG = 0x01
_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
_SYSTEM_INTERRUPT_CLEAR = 0x0B
_RESULT_INTERRUPT_STATUS = 0x13
_RESULT_RANGE_STATUS = 0x14
_RESULT_RANGE_MM = 0x1E          # RESULT_RANGE_STATUS + 10
_MSRC_CONFIG_CONTROL = 0x60
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_MSRC_CONFIG_TIMEOUT_MACROP = 0x46
_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
_I2C_SLAVE_DEVICE_ADDRESS = 0x8A
_OSC_CALIBRATE_VAL = 0xF8

_VCSEL_PERIOD_PRE_RANGE = 0
_VCSEL_PERIOD_FINAL_RANGE = 1

# v2.0.3 RANGE STATUS (RESULT_RANGE_STATUS bits 6:3), ST API mapping.
# The distance register ALWAYS holds a number, even when the sensor found no
# target. Those "no target" numbers are junk (often 20-200 mm) and must
# never be treated as a person.
RANGE_STATUS_HW_FAIL = (1, 2, 3)              # VCSEL / watchdog failure
RANGE_STATUS_NO_TARGET = (4, 6, 8, 9, 10)     # signal / phase / min-range fail
# Everything else (0, 11 "range complete", ...) is a real measurement.

SAMPLE_VALID = 0
SAMPLE_NO_TARGET = 1
SAMPLE_HW_FAIL = 2


def classify_range_status(status):
    if status in RANGE_STATUS_HW_FAIL:
        return SAMPLE_HW_FAIL
    if status in RANGE_STATUS_NO_TARGET:
        return SAMPLE_NO_TARGET
    return SAMPLE_VALID


PROFILE_LEGACY = "legacy"
PROFILE_FULL = "full"

# Pololu/ST default tuning table used by the "full" profile only.
_FULL_TUNING = (
    (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00),
    (0x11, 0x00), (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01),
    (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09),
    (0x54, 0x00), (0x31, 0x04), (0x32, 0x03), (0x40, 0x83), (0x46, 0x25),
    (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00), (0x52, 0x96),
    (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
    (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14),
    (0x49, 0xFF), (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00),
    (0x78, 0x21), (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF),
    (0x45, 0x26), (0x46, 0x05), (0x40, 0x40), (0x0E, 0x06), (0x20, 0x1A),
    (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44), (0xFF, 0x01),
    (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
    (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00),
    (0x70, 0x04), (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00),
    (0xFF, 0x01), (0x0D, 0x01), (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8),
    (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00),
)


def _decode_timeout(val):
    return float(val & 0xFF) * (2 ** ((val & 0xFF00) >> 8)) + 1


def _encode_timeout(timeout_mclks):
    timeout_mclks = int(timeout_mclks) & 0xFFFF
    ls_byte = 0
    ms_byte = 0
    if timeout_mclks > 0:
        ls_byte = timeout_mclks - 1
        while ls_byte > 255:
            ls_byte >>= 1
            ms_byte += 1
        return ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF
    return 0


def _timeout_mclks_to_microseconds(timeout_period_mclks, vcsel_period_pclks):
    macro_period_ns = ((2304 * vcsel_period_pclks * 1655) + 500) // 1000
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns // 2)) // 1000


def _timeout_microseconds_to_mclks(timeout_period_us, vcsel_period_pclks):
    macro_period_ns = ((2304 * vcsel_period_pclks * 1655) + 500) // 1000
    return ((timeout_period_us * 1000) + (macro_period_ns // 2)) // macro_period_ns


# ============================================================
# LOW-LEVEL DRIVER
# ============================================================

class VL53L0X:
    """VL53L0X driver with continuous and cooperative single-shot ranging.

    Constructor signature stays compatible with the calibration tools:
        VL53L0X(i2c, address)  -> legacy profile (calibration-compatible)
    """

    DEFAULT_ADDRESS = 0x29

    # Backward-compatible attribute names used by older code/tools.
    SYSRANGE_START = _SYSRANGE_START
    SYSTEM_INTERRUPT_CONFIG_GPIO = _SYSTEM_INTERRUPT_CONFIG_GPIO
    SYSTEM_INTERRUPT_CLEAR = _SYSTEM_INTERRUPT_CLEAR
    RESULT_INTERRUPT_STATUS = _RESULT_INTERRUPT_STATUS
    RESULT_RANGE_STATUS = _RESULT_RANGE_STATUS
    I2C_SLAVE_DEVICE_ADDRESS = _I2C_SLAVE_DEVICE_ADDRESS

    _IDLE = 0
    _WAIT_START_CLEAR = 1
    _WAIT_RESULT = 2

    def __init__(
        self,
        i2c,
        address=0x29,
        io_timeout_ms=100,
        profile=PROFILE_LEGACY,
    ):
        self.i2c = i2c
        self.address = int(address)
        self.io_timeout_ms = max(10, int(io_timeout_ms))
        self.profile = PROFILE_FULL if str(profile).lower() == PROFILE_FULL else PROFILE_LEGACY

        # Preallocated I2C buffers: no allocation in the hot path.
        self._b1 = bytearray(1)
        self._b2 = bytearray(2)
        self._b4 = bytearray(4)
        self._b6 = bytearray(6)
        self._b12 = bytearray(12)
        self.last_range_status = None
        self.last_signal_mcps = None

        self._state = self._IDLE
        self._deadline_ms = 0
        self._timeout_ms = 80
        self.continuous = False
        self.stop_variable = 0
        self._timing_budget_us = 0

        if self.profile == PROFILE_FULL:
            self._init_full()
        else:
            self._init_legacy()

    # ---------------- register helpers ----------------

    def _write8(self, register, value):
        self._b1[0] = value & 0xFF
        self.i2c.writeto_mem(self.address, register, self._b1)

    def _write16(self, register, value):
        self._b2[0] = (value >> 8) & 0xFF
        self._b2[1] = value & 0xFF
        self.i2c.writeto_mem(self.address, register, self._b2)

    def _write32(self, register, value):
        self._b4[0] = (value >> 24) & 0xFF
        self._b4[1] = (value >> 16) & 0xFF
        self._b4[2] = (value >> 8) & 0xFF
        self._b4[3] = value & 0xFF
        self.i2c.writeto_mem(self.address, register, self._b4)

    def _read8(self, register):
        self.i2c.readfrom_mem_into(self.address, register, self._b1)
        return self._b1[0]

    def _read16(self, register):
        self.i2c.readfrom_mem_into(self.address, register, self._b2)
        return (self._b2[0] << 8) | self._b2[1]

    def _write_pairs(self, pairs):
        for register, value in pairs:
            self._write8(register, value)

    def _wait_bounded(self, check):
        """Boot-only bounded wait. Raises instead of hanging."""
        start = time.ticks_ms()
        while not check():
            if time.ticks_diff(time.ticks_ms(), start) >= self.io_timeout_ms:
                raise RuntimeError("VL53L0X I/O timeout")
            time.sleep_us(500)

    # ---------------- initialization ----------------

    def _init_legacy(self):
        # Identical to the v1.9.x sequence used during calibration.
        self._write8(0x88, 0x00)
        self._write8(0x80, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)
        self.stop_variable = self._read8(0x91)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._write8(0x80, 0x00)

        self._write8(_MSRC_CONFIG_CONTROL, self._read8(_MSRC_CONFIG_CONTROL) | 0x12)
        self._write8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)

        gpio = self._read8(_GPIO_HV_MUX_ACTIVE_HIGH)
        gpio &= ~0x10
        self._write8(_GPIO_HV_MUX_ACTIVE_HIGH, gpio)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

    def _init_full(self):
        if self._read8(0xC0) != 0xEE:
            raise RuntimeError("VL53L0X model ID mismatch")

        self._write_pairs(((0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)))
        self.stop_variable = self._read8(0x91)
        self._write_pairs(((0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)))

        self._write8(_MSRC_CONFIG_CONTROL, self._read8(_MSRC_CONFIG_CONTROL) | 0x12)
        self.set_signal_rate_limit(0.25)
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)

        spad_count, spad_is_aperture = self._get_spad_info()

        ref_spad_map = self._b6
        self.i2c.readfrom_mem_into(self.address, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map)

        self._write_pairs((
            (0xFF, 0x01),
            (_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00),
            (_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C),
            (0xFF, 0x00),
            (_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4),
        ))

        first_spad_to_enable = 12 if spad_is_aperture else 0
        spads_enabled = 0
        for i in range(48):
            byte_index = i // 8
            bit = 1 << (i % 8)
            if i < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[byte_index] &= ~bit
            elif ref_spad_map[byte_index] & bit:
                spads_enabled += 1

        self.i2c.writeto_mem(self.address, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map)

        self._write_pairs(_FULL_TUNING)

        self._write8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        gpio = self._read8(_GPIO_HV_MUX_ACTIVE_HIGH)
        self._write8(_GPIO_HV_MUX_ACTIVE_HIGH, gpio & ~0x10)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        budget = self.get_timing_budget_us()
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.set_timing_budget_us(budget)

        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._single_ref_calibration(0x40)
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._single_ref_calibration(0x00)
        self._write8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    def _get_spad_info(self):
        self._write_pairs(((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x06)))
        self._write8(0x83, self._read8(0x83) | 0x04)
        self._write_pairs(((0xFF, 0x07), (0x81, 0x01), (0x80, 0x01), (0x94, 0x6B), (0x83, 0x00)))
        self._wait_bounded(lambda: self._read8(0x83) != 0x00)
        self._write8(0x83, 0x01)
        tmp = self._read8(0x92)
        count = tmp & 0x7F
        is_aperture = ((tmp >> 7) & 0x01) == 1
        self._write_pairs(((0x81, 0x00), (0xFF, 0x06)))
        self._write8(0x83, self._read8(0x83) & ~0x04)
        self._write_pairs(((0xFF, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)))
        return count, is_aperture

    def _single_ref_calibration(self, vhv_init_byte):
        self._write8(_SYSRANGE_START, 0x01 | (vhv_init_byte & 0xFF))
        self._wait_bounded(lambda: (self._read8(_RESULT_INTERRUPT_STATUS) & 0x07) != 0)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write8(_SYSRANGE_START, 0x00)

    # ---------------- configuration ----------------

    def set_address(self, new_address):
        new_address = int(new_address) & 0x7F
        if new_address == self.address:
            return
        self._write8(_I2C_SLAVE_DEVICE_ADDRESS, new_address)
        self.address = new_address

    def set_signal_rate_limit(self, mcps):
        mcps = float(mcps)
        if mcps < 0.0 or mcps > 511.99:
            raise ValueError("signal rate limit out of range")
        self._write16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(mcps * (1 << 7)))

    def _get_vcsel_pulse_period(self, vcsel_period_type):
        if vcsel_period_type == _VCSEL_PERIOD_PRE_RANGE:
            val = self._read8(_PRE_RANGE_CONFIG_VCSEL_PERIOD)
        else:
            val = self._read8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD)
        return ((val + 1) & 0xFF) << 1

    def _get_sequence_step_enables(self):
        cfg = self._read8(_SYSTEM_SEQUENCE_CONFIG)
        return (
            ((cfg >> 4) & 0x1) > 0,   # tcc
            ((cfg >> 3) & 0x1) > 0,   # dss
            ((cfg >> 2) & 0x1) > 0,   # msrc
            ((cfg >> 6) & 0x1) > 0,   # pre_range
            ((cfg >> 7) & 0x1) > 0,   # final_range
        )

    def _get_sequence_step_timeouts(self, pre_range):
        pre_vcsel = self._get_vcsel_pulse_period(_VCSEL_PERIOD_PRE_RANGE)
        msrc_dss_tcc_mclks = (self._read8(_MSRC_CONFIG_TIMEOUT_MACROP) + 1) & 0xFF
        msrc_dss_tcc_us = _timeout_mclks_to_microseconds(msrc_dss_tcc_mclks, pre_vcsel)
        pre_range_mclks = _decode_timeout(self._read16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        pre_range_us = _timeout_mclks_to_microseconds(pre_range_mclks, pre_vcsel)
        final_vcsel = self._get_vcsel_pulse_period(_VCSEL_PERIOD_FINAL_RANGE)
        final_range_mclks = _decode_timeout(self._read16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if pre_range:
            final_range_mclks -= pre_range_mclks
        final_range_us = _timeout_mclks_to_microseconds(final_range_mclks, final_vcsel)
        return msrc_dss_tcc_us, pre_range_us, final_range_us, final_vcsel, pre_range_mclks

    def get_timing_budget_us(self):
        budget_us = 1910 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        msrc_dss_tcc_us, pre_range_us, final_range_us, _, _ = self._get_sequence_step_timeouts(pre_range)
        if tcc:
            budget_us += msrc_dss_tcc_us + 590
        if dss:
            budget_us += 2 * (msrc_dss_tcc_us + 690)
        elif msrc:
            budget_us += msrc_dss_tcc_us + 660
        if pre_range:
            budget_us += pre_range_us + 660
        if final_range:
            budget_us += final_range_us + 550
        self._timing_budget_us = int(budget_us)
        return int(budget_us)

    def set_timing_budget_us(self, budget_us):
        budget_us = int(budget_us)
        if budget_us < 20000:
            raise ValueError("timing budget must be >= 20000 us")
        used_budget_us = 1320 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        steps = self._get_sequence_step_timeouts(pre_range)
        msrc_dss_tcc_us, pre_range_us = steps[0], steps[1]
        final_vcsel, pre_range_mclks = steps[3], steps[4]
        if tcc:
            used_budget_us += msrc_dss_tcc_us + 590
        if dss:
            used_budget_us += 2 * (msrc_dss_tcc_us + 690)
        elif msrc:
            used_budget_us += msrc_dss_tcc_us + 660
        if pre_range:
            used_budget_us += pre_range_us + 660
        if final_range:
            used_budget_us += 550
            if used_budget_us > budget_us:
                raise ValueError("timing budget too small for sequence")
            final_timeout_mclks = _timeout_microseconds_to_mclks(
                budget_us - used_budget_us, final_vcsel
            )
            if pre_range:
                final_timeout_mclks += pre_range_mclks
            self._write16(
                _FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                _encode_timeout(final_timeout_mclks),
            )
            self._timing_budget_us = budget_us
        return self._timing_budget_us

    # Kapetan-compatible property names.
    @property
    def measurement_timing_budget(self):
        return self.get_timing_budget_us()

    @measurement_timing_budget.setter
    def measurement_timing_budget(self, budget_us):
        self.set_timing_budget_us(budget_us)

    # ---------------- continuous ranging ----------------

    def _restore_stop_variable(self):
        self._write8(0x80, 0x01)
        self._write8(0xFF, 0x01)
        self._write8(0x00, 0x00)
        self._write8(0x91, self.stop_variable)
        self._write8(0x00, 0x01)
        self._write8(0xFF, 0x00)
        self._write8(0x80, 0x00)

    def start_continuous(self, period_ms=0):
        """Start back-to-back (period_ms=0) or timed continuous ranging."""
        self._state = self._IDLE
        self._restore_stop_variable()
        period_ms = int(period_ms)
        if period_ms > 0:
            osc = self._read16(_OSC_CALIBRATE_VAL)
            if osc != 0:
                period_ms *= osc
            self._write32(_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms)
            self._write8(_SYSRANGE_START, 0x04)
        else:
            self._write8(_SYSRANGE_START, 0x02)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.continuous = True

    def stop_continuous(self):
        self._write_pairs((
            (_SYSRANGE_START, 0x01),
            (0xFF, 0x01),
            (0x00, 0x00),
            (0x91, 0x00),
            (0x00, 0x01),
            (0xFF, 0x00),
        ))
        self.continuous = False

    def clear_interrupt(self):
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

    def data_ready(self):
        """One 1-byte I2C read. Never waits."""
        return (self._read8(_RESULT_INTERRUPT_STATUS) & 0x07) != 0

    def read_result_mm(self):
        """Read the completed range and clear the interrupt. Never waits."""
        distance = self._read16(_RESULT_RANGE_MM)
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        return distance

    def poll_continuous(self):
        """Return a new distance in mm, or None if no new sample yet."""
        if not self.data_ready():
            return None
        return self.read_result_mm()

    def poll_continuous_status(self):
        """v2.0.3: return (distance_mm, range_status) or None. One 12-byte
        block read (status + distance) + interrupt clear."""
        if not self.data_ready():
            return None
        self.i2c.readfrom_mem_into(self.address, _RESULT_RANGE_STATUS, self._b12)
        status = (self._b12[0] & 0x78) >> 3
        distance = (self._b12[10] << 8) | self._b12[11]
        # Return signal rate, 9.7 fixed point MCPS (diagnostic only).
        self.last_signal_mcps = ((self._b12[6] << 8) | self._b12[7]) / 128.0
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.last_range_status = status
        return distance, status

    # ---------------- single-shot (tools / compatibility) ----------------

    @property
    def busy(self):
        return self._state != self._IDLE

    def cancel(self):
        self._state = self._IDLE
        try:
            self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        except Exception:
            pass

    def begin_single(self, timeout_ms=80):
        """Start one range measurement without waiting for completion."""
        if self.busy or self.continuous:
            return False
        self._timeout_ms = max(10, int(timeout_ms))
        self._restore_stop_variable()
        self._write8(_SYSRANGE_START, 0x01)
        self._state = self._WAIT_START_CLEAR
        self._deadline_ms = time.ticks_add(time.ticks_ms(), self._timeout_ms)
        return True

    def poll_single(self):
        """(False, None) running / (True, mm) done / (True, None) failed."""
        if self._state == self._IDLE:
            return True, None
        if time.ticks_diff(time.ticks_ms(), self._deadline_ms) >= 0:
            self.cancel()
            return True, None
        try:
            if self._state == self._WAIT_START_CLEAR:
                if self._read8(_SYSRANGE_START) & 0x01:
                    return False, None
                self._state = self._WAIT_RESULT
                return False, None
            if self._state == self._WAIT_RESULT:
                if (self._read8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
                    return False, None
                distance = self._read16(_RESULT_RANGE_MM)
                self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
                self._state = self._IDLE
                return True, distance
        except Exception:
            self.cancel()
            return True, None
        self.cancel()
        return True, None


# ============================================================
# FASTLANE PRESENCE LAYER
# ============================================================

STATUS_PRESENCE = "PRESENCE"
STATUS_NO_PRESENCE = "NO_PRESENCE"

MEDIAN_WINDOW = 3

# Boot-only XSHUT/address sequencing (unchanged from v1.9.8).
XSHUT_ALL_LOW_MS = 150
XSHUT_WAKE_MS = 150
ADDRESS_CHANGE_SETTLE_MS = 100
FINAL_ADDRESS_SETTLE_MS = 150
HARDWARE_INIT_RETRIES = 3
HARDWARE_INIT_RETRY_DELAY_MS = 250

# Predictive poll scheduler.
POLL_RETRY_MS = 2          # sample not ready yet -> poll again after 2 ms
POLL_LEAD_MS = 3           # wake slightly before the next expected sample
POLL_ERROR_BACKOFF_MS = 10
PERIOD_MIN_MS = 8
PERIOD_MAX_MS = 150
DEFAULT_PERIOD_MS = 33     # VL53L0X default timing budget is ~33 ms

RESTART_THROTTLE_MS = 1000
ERROR_PRINT_THROTTLE_MS = 2000
I2C_ERROR_WINDOW_MS = 5000

# v2.0.1 stall watchdog floor: never shorter than this, nor than 4 sample
# periods (an old saved config clamped it to 80 ms with a 41 ms period).
WATCHDOG_MIN_MS = 250
WATCHDOG_PERIODS = 4

# v2.0.1 address-loss recovery. A VL53L0X whose supply dips (motor current /
# EMI) silently resets to 0x29, so its configured address answers ENODEV(19)
# forever. After this many consecutive errors (or this long without any
# sample) the pair is re-addressed with a short cooperative XSHUT sequence.
ENODEV = 19
RECOVERY_ERROR_THRESHOLD = 8
RECOVERY_NO_SAMPLE_MS = 1500
RECOVERY_STEP_WAIT_MS = 12        # XSHUT low / tBOOT (1.2 ms) with margin
RECOVERY_RETRY_BACKOFF_MS = 2000

# Diagnostics.
HEALTH_LOG_THROTTLE_MS = 5000
STUCK_PRESENCE_WARN_MS = 15000
STUCK_PRESENCE_REPEAT_MS = 30000

DEFAULT_CALIBRATION_FILE = "/presence_range_calibration.json"
DEFAULT_BACKGROUND_FILE = "/tof_background.json"
CALIBRATION_SCHEMA = "fastlane_presence_range_calibration"


def _load_json_file(path):
    if not path:
        return None
    try:
        with open(path, "r") as f:
            raw = f.read()
        if not raw:
            return None
        data = json.loads(raw)
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return None


def _positive_float(value):
    try:
        value = float(value)
    except Exception:
        return None
    if value > 0:
        return value
    return None


class _PresenceSensor:
    """Per-sensor fixed-background presence detector + poll scheduler."""

    def __init__(self, sensor_no, name, angle, hw, offset_mm=0.0):
        self.sensor_no = int(sensor_no)
        self.name = str(name)
        self.angle = int(angle)
        self.hw = hw

        # Calibration / thresholds.
        self.offset_mm = float(offset_mm)
        self.background_mm = None          # corrected (real) distance
        self.background_raw_mm = None      # raw value if source stored raw
        self.background_source = "NONE"
        self.presence_delta_mm = 120.0
        self.release_delta_mm = 70.0
        self.min_valid_mm = 50
        self.max_valid_mm = 3000
        self.presence_confirm = 1
        self.clear_confirm = 2
        self.sensor_stale_ms = 350
        self.capture_samples = 30
        self.capture_max_spread_mm = 80.0

        # Live measurement state.
        self.raw_mm = None
        self.corrected_mm = None
        self.median_mm = None
        self.window = []
        self.present = False
        self.presence_hits = 0
        self.clear_hits = 0
        self.last_valid_ms = None
        self.valid_samples = 0
        self.invalid_count = 0
        self.far_count = 0
        self.capture = []
        self.capture_rejects = 0

        # Scheduler state.
        self.next_poll_ms = 0
        self.period_ms = float(DEFAULT_PERIOD_MS)
        self.last_sample_t = None
        self.discard_next = 0
        self.alive_since_ms = 0
        self.rate_window_start = 0
        self.rate_window_count = 0
        self.rate_hz = 0.0
        self.restarts = 0
        self.last_restart_ms = 0
        self.io_errors = 0
        self.consec_errors = 0
        self.nodev_errors = 0
        self.no_target_count = 0
        self.hw_fail_count = 0
        self.near_blank_mm = 150.0
        self.near_blank_count = 0
        # v2.1.12: a STRONG echo inside the blank zone is a real body close to
        # the sensor (live: 154-201 mm raw @ 8-13 MCPS was blanked -> CLEAR).
        self.near_blank_strong_mcps = 2.5
        self.near_strong_count = 0
        # v2.1.1: weak-echo rejection. A VALID return whose signal rate is
        # below this is not a person (S1 empty-lane ghost: 0.58-0.78 MCPS).
        self.min_presence_signal_mcps = 0.0
        self.weak_echo_count = 0

        # v2.0.7 role: "entry" (always counted) or "exit" (angled through
        # the barrier; counted only while the gate is stationary OPEN because
        # its beam hits the closed arm when the gate is LOCKED).
        self.role = "entry"
        self.context_active = True
        self.last_signal_mcps = None
        self.last_status = None
        self.status_counts = {}

    # ---------------- thresholds ----------------

    def enter_threshold(self):
        if self.background_mm is None:
            return None
        return max(float(self.min_valid_mm) + 10.0, self.background_mm - self.presence_delta_mm)

    def release_threshold(self):
        if self.background_mm is None:
            return None
        release = self.background_mm - self.release_delta_mm
        enter = self.enter_threshold()
        if release < enter + 20.0:
            release = enter + 20.0
        return release

    def ready(self):
        return self.background_mm is not None

    def is_fresh(self, now_ms):
        if self.last_valid_ms is None:
            return False
        return time.ticks_diff(now_ms, self.last_valid_ms) <= self.sensor_stale_ms

    def effective_present(self, now_ms):
        return bool(
            self.context_active
            and self.ready()
            and self.is_fresh(now_ms)
            and self.present
        )

    def reset_live(self, keep_present=True):
        self.window = []
        self.presence_hits = 0
        self.clear_hits = 0
        if not keep_present:
            self.present = False

    def set_background(self, corrected_mm, source, raw_mm=None):
        self.background_mm = float(corrected_mm) if corrected_mm is not None else None
        self.background_raw_mm = float(raw_mm) if raw_mm is not None else None
        self.background_source = str(source)
        self.capture = []
        self.reset_live(keep_present=False)

    # ---------------- sample processing ----------------

    def _note_rate(self, now_ms):
        if not self.rate_window_start:
            self.rate_window_start = now_ms
            self.rate_window_count = 0
        self.rate_window_count += 1
        elapsed = time.ticks_diff(now_ms, self.rate_window_start)
        if elapsed >= 1000:
            self.rate_hz = round(self.rate_window_count * 1000.0 / elapsed, 1)
            self.rate_window_start = now_ms
            self.rate_window_count = 0

    def process(self, raw_mm, now_ms, range_status=None):
        """Handle one completed sample. Returns 'CAPTURED' when a background
        capture just completed, else None."""
        if raw_mm is None:
            self.invalid_count += 1
            return None

        raw = float(raw_mm)

        no_target = False
        if range_status is not None:
            self.last_status = range_status
            self.status_counts[range_status] = self.status_counts.get(range_status, 0) + 1
            kind = classify_range_status(range_status)
            if kind == SAMPLE_HW_FAIL:
                self.hw_fail_count += 1
                self.invalid_count += 1
                return None
            if kind == SAMPLE_NO_TARGET:
                # Nothing reflected inside range: the beam crossed an empty
                # lane (far wall beyond reach). This is CLEAR, never presence.
                self.no_target_count += 1
                no_target = True

        # v2.0.4 near-field blanking: a "valid" return within near_blank_mm
        # of the sensor face is crosstalk from the mounting hole / cover /
        # pillar edge (seen on S2 as 62-127 mm flicker on an empty lane). A
        # person crossing the lane is never that close to the glass.
        if (
            not no_target
            and self.near_blank_mm > 0
            and (raw + self.offset_mm) < self.near_blank_mm
        ):
            if (
                self.near_blank_strong_mcps > 0
                and self.last_signal_mcps is not None
                and self.last_signal_mcps >= self.near_blank_strong_mcps
            ):
                self.near_strong_count += 1      # person right at the sensor
            else:
                self.near_blank_count += 1
                no_target = True

        # v2.1.1 weak-echo rejection. Live capture on the empty lane (gate
        # locked, 03 Oct 2026): S1 returned "valid" status-11 ranges at
        # 315-403 mm raw with only 0.58-0.78 MCPS, interleaved with status-8
        # failures, which flipped PRESENCE several times per second, blocked
        # RFID re-arm, boot homing and the close timer. A body in the lane
        # reflects far more signal, so a weak valid echo counts as CLEAR.
        if (
            not no_target
            and self.min_presence_signal_mcps > 0
            and self.last_signal_mcps is not None
            and self.last_signal_mcps < self.min_presence_signal_mcps
        ):
            self.weak_echo_count += 1
            no_target = True

        # Too close / sensor-face noise: reject (as in v1.9.x logs, ~20 mm).
        if not no_target and raw < self.min_valid_mm:
            self.invalid_count += 1
            return None

        if no_target or raw > self.max_valid_mm:
            # No target / out of range = beam passed the lane = CLEAR.
            self.far_count += 1
            corrected = float(self.max_valid_mm)
        else:
            corrected = raw + self.offset_mm
            if corrected < self.min_valid_mm:
                corrected = float(self.min_valid_mm)

        self.raw_mm = raw
        self.corrected_mm = corrected
        self.last_valid_ms = now_ms
        self.valid_samples += 1
        self._note_rate(now_ms)

        self.window.append(corrected)
        if len(self.window) > MEDIAN_WINDOW:
            self.window.pop(0)
        self.median_mm = float(_median(self.window))

        if not self.context_active:
            # Exit sensor while the gate is closed: it sees the closed arm.
            # Keep it fresh for diagnostics, never report presence.
            self.present = False
            self.presence_hits = 0
            self.clear_hits = 0
            return None

        if self.background_mm is None:
            return self._capture_step(raw, corrected)

        distance = self.median_mm
        if self.present:
            candidate = distance < self.release_threshold()
        else:
            candidate = distance <= self.enter_threshold()

        if candidate:
            self.presence_hits += 1
            self.clear_hits = 0
            if self.presence_hits >= self.presence_confirm:
                self.present = True
        else:
            self.clear_hits += 1
            self.presence_hits = 0
            if self.clear_hits >= self.clear_confirm:
                self.present = False
        return None

    def _capture_step(self, raw, corrected):
        self.present = False
        self.capture.append(corrected)
        if len(self.capture) < self.capture_samples:
            return None

        low = _percentile(self.capture, 0.10)
        high = _percentile(self.capture, 0.90)
        spread = high - low
        if spread <= self.capture_max_spread_mm:
            background = float(_median(self.capture))
            raw_background = background - self.offset_mm
            if background >= float(self.max_valid_mm):
                raw_background = None
            self.set_background(background, "AUTO_CAPTURE", raw_background)
            return "CAPTURED"

        # Lane not empty / unstable: drop the oldest half and keep sampling.
        self.capture_rejects += 1
        self.capture = self.capture[len(self.capture) // 2:]
        return "UNSTABLE"

    # ---------------- status ----------------

    def status(self, now_ms, paused=False):
        fresh = self.is_fresh(now_ms)
        if not self.ready():
            state = "CAPTURING_BACKGROUND"
        elif not self.context_active:
            state = "IGNORED_GATE_CLOSED"
        elif paused:
            state = "PAUSED"
        elif not fresh:
            state = "STALE"
        elif self.present:
            state = STATUS_PRESENCE
        else:
            state = STATUS_NO_PRESENCE

        age_ms = None
        if self.last_valid_ms is not None:
            age_ms = max(0, time.ticks_diff(now_ms, self.last_valid_ms))

        enter = self.enter_threshold()
        release = self.release_threshold()
        return {
            "name": self.name,
            "angle": self.angle,
            "raw_mm": self.raw_mm,
            "corrected_mm": self.corrected_mm,
            "median_mm": self.median_mm,
            "avg3_mm": self.median_mm,          # web UI compatibility
            "state": state,
            "present": bool(self.effective_present(now_ms)),
            "fresh": bool(fresh),
            "age_ms": age_ms,

            "background_mm": self.background_mm,
            "background_raw_mm": self.background_raw_mm,
            "background_source": self.background_source,
            "clear_center_mm": self.background_mm,   # web UI compatibility
            "enter_threshold_mm": enter,
            "clear_threshold_mm": release,
            "presence_delta_mm": self.presence_delta_mm,
            "release_delta_mm": self.release_delta_mm,
            "capture_samples": len(self.capture),
            "capture_required": self.capture_samples,
            "capture_rejects": self.capture_rejects,

            "offset_mm": self.offset_mm,
            "invalid_count": self.invalid_count,
            "far_count": self.far_count,
            "valid_samples": self.valid_samples,
            "rate_hz": self.rate_hz,
            "period_ms": round(self.period_ms, 1),
            "restarts": self.restarts,
            "io_errors": self.io_errors,
            "nodev_errors": self.nodev_errors,
            "consec_errors": self.consec_errors,
            "last_range_status": self.last_status,
            "role": self.role,
            "context_active": bool(self.context_active),
            "near_blank_mm": self.near_blank_mm,
            "near_blank_count": self.near_blank_count,
            "near_strong_count": self.near_strong_count,
            "near_blank_strong_mcps": self.near_blank_strong_mcps,
            "weak_echo_count": self.weak_echo_count,
            "min_presence_signal_mcps": self.min_presence_signal_mcps,
            "last_signal_mcps": self.last_signal_mcps,
            "no_target_count": self.no_target_count,
            "hw_fail_count": self.hw_fail_count,
            "status_counts": dict(self.status_counts),
        }


class DualVL53L0XSafety:
    """Two-sensor continuous PRESENCE / NO_PRESENCE component.

    PRESENCE    : one or both fresh sensors confirm an obstruction/person.
    NO_PRESENCE : both sensors are fresh and neither confirms presence.
    During a temporary stale gap (or while paused for motor motion) the last
    real logical result is retained.

    Scheduler contract with main.py:
      pause()  - stop all I2C traffic (motor moving). Instant, no I2C.
      resume() - continue; first post-pause sample is discarded.
      update() - at most one short poll per sensor, never waits.
    """

    def __init__(
        self,
        i2c,
        xshut1_pin,
        xshut2_pin,
        sensor1_address=0x30,
        sensor2_address=0x31,
        default_address=0x29,
        sensor1_angle=90,
        sensor2_angle=90,
        io_timeout_ms=60,
        calibration_file=DEFAULT_CALIBRATION_FILE,
        background_file=DEFAULT_BACKGROUND_FILE,
    ):
        self.i2c = i2c
        self.xshut1_pin_num = int(xshut1_pin)
        self.xshut2_pin_num = int(xshut2_pin)
        self.sensor1_address = int(sensor1_address)
        self.sensor2_address = int(sensor2_address)
        self.default_address = int(default_address)
        self.sensor1_angle = int(sensor1_angle)
        self.sensor2_angle = int(sensor2_angle)
        self.io_timeout_ms = max(10, int(io_timeout_ms))
        self.calibration_file = calibration_file
        self.background_file = background_file

        self.xshut1 = None
        self.xshut2 = None
        self.sensor1_hw = None
        self.sensor2_hw = None
        self.sensor1 = None
        self.sensor2 = None

        self.enabled = True
        self.require_clear_to_close = True
        self.reopen_on_obstruction = True
        self.measurement_timeout_ms = 250     # stall watchdog
        self.sensor_stale_ms = 350
        self.close_clear_grace_ms = 2000
        self.close_allow_stale_no_presence = True

        self.driver_profile = PROFILE_LEGACY
        self.timing_budget_us = 0
        self.signal_rate_limit_mcps = 0.0
        self.use_calibration_file = True

        self.hardware_ready = False
        self.ready = False
        self.last_error = ""

        self.global_state = STATUS_NO_PRESENCE
        self.presence_detected = False

        self.paused = True                 # starts paused until initialize()
        self.pause_reason = "INIT"
        self.paused_since_ms = time.ticks_ms()
        self.resumed_at_ms = 0
        self.pause_count = 0

        self._hardware = []
        self._software = []
        self._cfg = {}
        self._calibration = None
        self._background_saved = None
        self.background_save_pending = False

        self._i2c_error_times = []
        self._last_error_print_ms = 0
        self._last_logged_state = None
        self._last_logged_data_valid = None
        self._last_health_log_ms = 0

        # v2.0.7: gate context for exit-role sensors (set by main.py).
        self.gate_open_context = False

        # v2.0.5: main.py sets this to a bus-clear + rebuild function.
        self.bus_recover_cb = None

        # v2.0.1 address-loss recovery state machine.
        self.recovery_needed = False
        self.recovery_reason = ""
        self.recovery_step = 0
        self.recovery_at_ms = 0
        self.recovery_count = 0
        self.recovery_failures = 0
        self._recovery_hw = [None, None]

        # v2.0.1 stuck-presence diagnostic.
        self._presence_since_ms = 0
        self._stuck_warn_at_ms = 0

    # ========================================================
    # BOOT-ONLY HARDWARE SEQUENCE
    # ========================================================

    def _scan_addresses(self):
        try:
            return self.i2c.scan()
        except Exception as exc:
            self.last_error = "I2C scan failed: {}".format(repr(exc))
            return []

    def _scan_text(self):
        return "[{}]".format(
            ", ".join("0x{:02X}".format(int(a)) for a in self._scan_addresses())
        )

    def _shutdown_both_sensors(self):
        if self.xshut1 is None:
            self.xshut1 = Pin(self.xshut1_pin_num, Pin.OUT, value=0)
        else:
            self.xshut1.value(0)
        if self.xshut2 is None:
            self.xshut2 = Pin(self.xshut2_pin_num, Pin.OUT, value=0)
        else:
            self.xshut2.value(0)
        time.sleep_ms(XSHUT_ALL_LOW_MS)

    def _make_hw(self):
        return VL53L0X(
            self.i2c,
            self.default_address,
            io_timeout_ms=self.io_timeout_ms,
            profile=self.driver_profile,
        )

    def _initialize_hardware_pair(self):
        self.sensor1_hw = None
        self.sensor2_hw = None
        last_error = None
        last_stage = "START"

        for attempt in range(1, HARDWARE_INIT_RETRIES + 1):
            try:
                print()
                print("VL53L0X HARDWARE INIT ATTEMPT {}/{} | PROFILE={}".format(
                    attempt, HARDWARE_INIT_RETRIES, self.driver_profile.upper()))

                last_stage = "BOTH_XSHUT_LOW"
                self._shutdown_both_sensors()
                print("I2C WITH VL53L0X OFF:", self._scan_text())

                last_stage = "S1_WAKE"
                self.xshut1.value(1)
                time.sleep_ms(XSHUT_WAKE_MS)
                if self.default_address not in self._scan_addresses():
                    raise RuntimeError("S1 did not appear at 0x{:02X}; I2C={}".format(
                        self.default_address, self._scan_text()))
                last_stage = "S1_INIT"
                self.sensor1_hw = self._make_hw()
                last_stage = "S1_SET_ADDRESS"
                self.sensor1_hw.set_address(self.sensor1_address)
                time.sleep_ms(ADDRESS_CHANGE_SETTLE_MS)
                if self.sensor1_address not in self._scan_addresses():
                    raise RuntimeError("S1 failed to move to 0x{:02X}; I2C={}".format(
                        self.sensor1_address, self._scan_text()))
                print("S1 ADDRESS READY: 0x{:02X}".format(self.sensor1_address))

                last_stage = "S2_WAKE"
                self.xshut2.value(1)
                time.sleep_ms(XSHUT_WAKE_MS)
                if self.default_address not in self._scan_addresses():
                    raise RuntimeError("S2 did not appear at 0x{:02X}; I2C={}".format(
                        self.default_address, self._scan_text()))
                last_stage = "S2_INIT"
                self.sensor2_hw = self._make_hw()
                last_stage = "S2_SET_ADDRESS"
                self.sensor2_hw.set_address(self.sensor2_address)
                time.sleep_ms(FINAL_ADDRESS_SETTLE_MS)

                last_stage = "FINAL_VERIFY"
                devices = self._scan_addresses()
                if self.sensor1_address not in devices:
                    raise RuntimeError("S1 missing at 0x{:02X}; I2C={}".format(
                        self.sensor1_address, self._scan_text()))
                if self.sensor2_address not in devices:
                    raise RuntimeError("S2 missing at 0x{:02X}; I2C={}".format(
                        self.sensor2_address, self._scan_text()))

                print("VL53L0X I2C FINAL:", self._scan_text())
                return True

            except Exception as exc:
                last_error = exc
                print("VL53L0X INIT ATTEMPT FAILED | STAGE={} | ERROR={} | I2C={}".format(
                    last_stage, repr(exc), self._scan_text()))
                try:
                    self._shutdown_both_sensors()
                except Exception:
                    pass
                self.sensor1_hw = None
                self.sensor2_hw = None
                if attempt < HARDWARE_INIT_RETRIES:
                    time.sleep_ms(HARDWARE_INIT_RETRY_DELAY_MS)

        self.last_error = "VL53L0X hardware init failed at {}: {}".format(
            last_stage, repr(last_error))
        return False

    def _apply_measurement_profile(self, hw, label):
        """Optional timing budget / signal-rate overrides. Boot/config only."""
        if self.signal_rate_limit_mcps and self.signal_rate_limit_mcps > 0:
            try:
                hw.set_signal_rate_limit(self.signal_rate_limit_mcps)
            except Exception as exc:
                print("{} SIGNAL RATE LIMIT NOT APPLIED: {}".format(label, repr(exc)))

        if self.timing_budget_us and self.timing_budget_us >= 20000:
            try:
                hw.set_timing_budget_us(self.timing_budget_us)
            except Exception as exc:
                print("{} TIMING BUDGET NOT APPLIED: {}".format(label, repr(exc)))

        budget = 0
        try:
            budget = hw.get_timing_budget_us()
        except Exception:
            budget = 0
        return budget

    def _start_sensor(self, index, now_ms):
        hw = self._hardware[index]
        sw = self._software[index]
        hw.start_continuous(0)
        sw.next_poll_ms = now_ms
        sw.last_sample_t = None
        sw.discard_next = 1
        sw.alive_since_ms = now_ms
        sw.reset_live(keep_present=True)

    # ========================================================
    # CALIBRATION / BACKGROUND RESOLUTION
    # ========================================================

    def _load_calibration_file(self):
        data = _load_json_file(self.calibration_file)
        if data is None:
            return None
        schema = str(data.get("schema", ""))
        if schema and schema != CALIBRATION_SCHEMA:
            print("VL53L0X CALIBRATION FILE IGNORED: schema", schema)
            return None
        return data

    def _resolve_background(self, sensor, cfg):
        n = sensor.sensor_no

        configured = _positive_float(cfg.get("sensor{}_background_mm".format(n), 0))
        if configured is not None:
            sensor.set_background(configured, "CONFIG", configured - sensor.offset_mm)
            return

        cal = self._calibration if self.use_calibration_file else None
        if cal and cal.get("background_calibrated", False):
            raw = _positive_float(cal.get("sensor{}_background_mm".format(n)))
            if raw is not None:
                sensor.set_background(raw + sensor.offset_mm, "CALIBRATION_FILE", raw)
                return
            corrected = _positive_float(cal.get("sensor{}_background_corrected_mm".format(n)))
            if corrected is not None:
                sensor.set_background(corrected, "CALIBRATION_FILE", None)
                return

        saved = self._background_saved
        if saved and str(saved.get("driver_profile", PROFILE_LEGACY)) == self.driver_profile:
            raw = _positive_float(saved.get("sensor{}_background_raw_mm".format(n)))
            if raw is not None:
                sensor.set_background(raw + sensor.offset_mm, "AUTO_CAPTURE_FILE", raw)
                return
            far = saved.get("sensor{}_background_far".format(n), False)
            if far:
                sensor.set_background(float(sensor.max_valid_mm), "AUTO_CAPTURE_FILE", None)
                return

        if sensor.role == "exit":
            # Exit beam must not be captured with the gate closed (it sees the
            # arm). Without a configured value, only targets closer than
            # max_valid - delta count; "no target" is clear.
            sensor.set_background(float(sensor.max_valid_mm), "EXIT_DEFAULT_FAR", None)
            return
        sensor.set_background(None, "CAPTURING", None)

    def _report_calibration(self):
        cal = self._calibration
        if cal is None:
            print("VL53L0X CALIBRATION FILE :", self.calibration_file, "(not present)")
            return
        print("VL53L0X CALIBRATION FILE :", self.calibration_file)
        print("  METHOD                 :", cal.get("calibration_method", "?"))
        print("  BACKGROUND CALIBRATED  :", bool(cal.get("background_calibrated", False)))
        for sensor in (self.sensor1, self.sensor2):
            key = "sensor{}_offset_mm".format(sensor.sensor_no)
            if key in cal:
                try:
                    if abs(float(cal[key]) - sensor.offset_mm) > 0.5:
                        print("  WARNING: {} file={} config={} (config is used)".format(
                            key, cal[key], sensor.offset_mm))
                except Exception:
                    pass

    def save_background_now(self):
        """Write /tof_background.json. Call only while the gate is idle."""
        if not self.background_file or self.sensor1 is None:
            self.background_save_pending = False
            return False
        data = {
            "schema": "fastlane_tof_background",
            "schema_version": 1,
            "driver_profile": self.driver_profile,
            "timing_budget_us": self.timing_budget_us,
        }
        for sensor in (self.sensor1, self.sensor2):
            n = sensor.sensor_no
            data["sensor{}_offset_mm".format(n)] = sensor.offset_mm
            data["sensor{}_background_corrected_mm".format(n)] = sensor.background_mm
            data["sensor{}_background_raw_mm".format(n)] = sensor.background_raw_mm
            data["sensor{}_background_far".format(n)] = bool(
                sensor.background_mm is not None
                and sensor.background_raw_mm is None
            )
        temp = self.background_file + ".tmp"
        try:
            with open(temp, "w") as f:
                f.write(json.dumps(data))
            try:
                os.remove(self.background_file)
            except Exception:
                pass
            os.rename(temp, self.background_file)
            self._background_saved = data
            self.background_save_pending = False
            print("VL53L0X BACKGROUND SAVED:", self.background_file)
            return True
        except Exception as exc:
            self.background_save_pending = False
            print("VL53L0X BACKGROUND SAVE ERROR:", repr(exc))
            return False

    def request_background_capture(self):
        """Forget the background and capture a new one (lane must be empty)."""
        if self.sensor1 is None or self.sensor2 is None:
            return False
        for sensor in (self.sensor1, self.sensor2):
            if sensor.role == "exit":
                continue      # exit background is configured, never captured closed
            sensor.set_background(None, "CAPTURING", None)
            sensor.capture_rejects = 0
        self.ready = False
        self.presence_detected = False
        self.global_state = STATUS_NO_PRESENCE
        print("VL53L0X BACKGROUND CAPTURE REQUESTED - KEEP LANE EMPTY")
        return True

    # ========================================================
    # CONFIGURATION
    # ========================================================

    def _apply_sensor_cfg(self, sensor, cfg):
        sensor.offset_mm = float(cfg.get("sensor{}_offset_mm".format(sensor.sensor_no), sensor.offset_mm))
        sensor.min_valid_mm = max(20, int(cfg.get("min_valid_mm", 50)))
        sensor.max_valid_mm = max(sensor.min_valid_mm + 100, int(cfg.get("max_valid_mm", 3000)))
        sensor.sensor_stale_ms = self.sensor_stale_ms
        sensor.presence_delta_mm = max(20.0, float(cfg.get("presence_delta_mm", 150)))
        sensor.release_delta_mm = max(5.0, float(cfg.get("release_delta_mm", 90)))
        if sensor.release_delta_mm >= sensor.presence_delta_mm:
            sensor.release_delta_mm = max(5.0, sensor.presence_delta_mm - 20.0)
        sensor.presence_confirm = max(1, int(cfg.get("presence_confirm_samples", 1)))
        sensor.clear_confirm = max(1, int(cfg.get("clear_confirm_samples", 2)))
        sensor.capture_samples = max(10, int(cfg.get("background_capture_samples", 30)))
        role = str(cfg.get("sensor{}_role".format(sensor.sensor_no),
                           "entry" if sensor.sensor_no == 1 else "exit")).lower()
        if role not in ("entry", "exit"):
            role = "entry"
        if sensor.sensor_no == 1:
            role = "entry"          # S1 is always the entry / RFID-clear sensor
        if role != sensor.role:
            sensor.role = role
            sensor.context_active = (role == "entry") or self.gate_open_context
            sensor.reset_live(keep_present=False)
        sensor.near_blank_mm = max(0.0, float(cfg.get(
            "sensor{}_near_blank_mm".format(sensor.sensor_no),
            cfg.get("near_blank_mm", 150))))
        sensor.capture_max_spread_mm = max(10.0, float(cfg.get("background_capture_max_spread_mm", 80)))
        sensor.near_blank_strong_mcps = max(0.0, float(cfg.get("near_blank_strong_mcps", 2.5)))
        sensor.min_presence_signal_mcps = max(0.0, float(cfg.get(
            "sensor{}_min_signal_mcps".format(sensor.sensor_no),
            cfg.get("min_presence_signal_mcps", 1.0))))

    def configure(self, cfg):
        self._cfg = dict(cfg)
        self.enabled = bool(cfg.get("enabled", True))
        self.require_clear_to_close = bool(cfg.get("require_clear_to_close", True))
        self.reopen_on_obstruction = bool(cfg.get("reopen_on_obstruction", True))
        self.measurement_timeout_ms = max(80, int(cfg.get("measurement_timeout_ms", 250)))
        self.sensor_stale_ms = max(100, int(cfg.get("sensor_stale_ms", 350)))
        self.close_clear_grace_ms = max(self.sensor_stale_ms, int(cfg.get("close_clear_grace_ms", 2000)))
        self.close_allow_stale_no_presence = bool(cfg.get("close_allow_stale_no_presence", True))
        self.use_calibration_file = bool(cfg.get("use_calibration_file", True))
        self.signal_rate_limit_mcps = float(cfg.get("signal_rate_limit_mcps", 0) or 0)

        profile = str(cfg.get("driver_profile", PROFILE_LEGACY)).lower()
        if profile not in (PROFILE_LEGACY, PROFILE_FULL):
            profile = PROFILE_LEGACY
        if self.hardware_ready and profile != self.driver_profile:
            print("VL53L0X DRIVER PROFILE CHANGE TO", profile.upper(), "APPLIES AFTER REBOOT")
        if not self.hardware_ready:
            self.driver_profile = profile

        budget = int(cfg.get("timing_budget_us", 0) or 0)
        if budget and budget < 20000:
            budget = 20000
        budget_changed = budget != self.timing_budget_us
        self.timing_budget_us = budget

        if self.sensor1 is None or self.sensor2 is None:
            return

        old_backgrounds = (self.sensor1.background_mm, self.sensor2.background_mm)
        for sensor in (self.sensor1, self.sensor2):
            old_offset = sensor.offset_mm
            self._apply_sensor_cfg(sensor, cfg)
            # Re-resolve only if the configured background or offset changed.
            configured = _positive_float(cfg.get("sensor{}_background_mm".format(sensor.sensor_no), 0))
            if configured is not None and configured != sensor.background_mm:
                sensor.set_background(configured, "CONFIG", configured - sensor.offset_mm)
            elif configured is None and sensor.background_source == "CONFIG":
                self._resolve_background(sensor, cfg)
            elif old_offset != sensor.offset_mm and sensor.background_raw_mm is not None:
                if sensor.background_source != "CONFIG":
                    sensor.set_background(
                        sensor.background_raw_mm + sensor.offset_mm,
                        sensor.background_source,
                        sensor.background_raw_mm,
                    )

        if budget_changed and self.hardware_ready and budget >= 20000:
            for index, hw in enumerate(self._hardware):
                try:
                    hw.stop_continuous()
                    self._apply_measurement_profile(hw, "S{}".format(index + 1))
                    if not self.paused:
                        self._start_sensor(index, time.ticks_ms())
                except Exception as exc:
                    self._note_io_error(index, exc, time.ticks_ms())

        if old_backgrounds != (self.sensor1.background_mm, self.sensor2.background_mm):
            print("VL53L0X BACKGROUND:", self._background_text())

        self.ready = self._models_ready()

    def _background_sanity(self):
        """v2.0.2: warn when a background is too short to ever detect a person
        (e.g. the beam hits the gate frame ~10 cm away)."""
        for sensor in (self.sensor1, self.sensor2):
            if sensor is None or sensor.background_mm is None:
                continue
            if sensor.role == "exit" and sensor.background_source == "EXIT_DEFAULT_FAR":
                continue
            usable = sensor.background_mm - sensor.presence_delta_mm
            if usable < 150:
                print("VL53L0X WARNING: S{} background {:.0f} mm is too short - the beam".format(
                    sensor.sensor_no, sensor.background_mm))
                print("  is blocked near the sensor (frame/leaf?). This sensor cannot detect")
                print("  a person. Re-aim it across the open lane, then capture again.")

    def _background_text(self):
        parts = []
        for sensor in (self.sensor1, self.sensor2):
            if sensor is None:
                continue
            parts.append("S{}={} ({})".format(
                sensor.sensor_no,
                "{:.0f}mm".format(sensor.background_mm) if sensor.background_mm is not None else "-",
                sensor.background_source,
            ))
        return " | ".join(parts)

    def initialize(self, cfg):
        self.last_error = ""
        self.hardware_ready = False
        self.ready = False
        self.presence_detected = False
        self.global_state = STATUS_NO_PRESENCE
        self._hardware = []
        self._software = []
        self._last_logged_state = None
        self._last_logged_data_valid = None

        self.configure(cfg)

        if not self.enabled:
            self.ready = True
            self.paused = True
            self.pause_reason = "DISABLED"
            print("VL53L0X PRESENCE DISABLED BY CONFIG")
            return True

        try:
            if not self._initialize_hardware_pair():
                raise RuntimeError(self.last_error or "VL53L0X hardware pair initialization failed")

            self.sensor1 = _PresenceSensor(1, "SENSOR #1", self.sensor1_angle, self.sensor1_hw,
                                           float(cfg.get("sensor1_offset_mm", -43.375)))
            self.sensor2 = _PresenceSensor(2, "SENSOR #2", self.sensor2_angle, self.sensor2_hw,
                                           float(cfg.get("sensor2_offset_mm", -42.0)))
            self._hardware = [self.sensor1_hw, self.sensor2_hw]
            self._software = [self.sensor1, self.sensor2]

            for sensor in self._software:
                self._apply_sensor_cfg(sensor, cfg)

            budgets = []
            for index, hw in enumerate(self._hardware):
                budgets.append(self._apply_measurement_profile(hw, "S{}".format(index + 1)))

            for index, budget in enumerate(budgets):
                if budget and budget >= 20000:
                    period = budget / 1000.0 + 1.0
                    if PERIOD_MIN_MS <= period <= PERIOD_MAX_MS:
                        self._software[index].period_ms = period

            self._calibration = self._load_calibration_file()
            self._background_saved = _load_json_file(self.background_file)
            for sensor in self._software:
                self._resolve_background(sensor, cfg)

            self.hardware_ready = True

            now = time.ticks_ms()
            for index in (0, 1):
                self._start_sensor(index, now)
            self.paused = False
            self.pause_reason = ""
            self.resumed_at_ms = now

            self.ready = self._models_ready()

            print()
            print("========================================")
            print("DUAL VL53L0X CONTINUOUS PRESENCE v2.1.1")
            print("========================================")
            print("SENSOR ADDRESSES   :", hex(self.sensor1_address), "/", hex(self.sensor2_address))
            print("XSHUT              : GPIO", self.xshut1_pin_num, "/ GPIO", self.xshut2_pin_num)
            print("DRIVER PROFILE     :", self.driver_profile.upper())
            print("RANGING MODE       : CONTINUOUS BACK-TO-BACK, BOTH SENSORS IN PARALLEL")
            print("TIMING BUDGET      : S1={} us S2={} us{}".format(
                budgets[0], budgets[1],
                "" if self.timing_budget_us else " (device default / calibrated)"))
            print("OFFSETS            : S1={:+.3f} mm S2={:+.3f} mm".format(
                self.sensor1.offset_mm, self.sensor2.offset_mm))
            print("BACKGROUND         :", self._background_text())
            print("SENSOR ROLES       : S1={} | S2={} (exit counts only while gate OPEN)".format(
                self.sensor1.role.upper(), self.sensor2.role.upper()))
            print("PRESENCE / RELEASE : bg -{} mm / bg -{} mm".format(
                self.sensor1.presence_delta_mm, self.sensor1.release_delta_mm))
            print("STALL WATCHDOG     : {} ms (configured {} ms, floor {} ms / {} periods)".format(
                self._watchdog_ms(self.sensor1), self.measurement_timeout_ms,
                WATCHDOG_MIN_MS, WATCHDOG_PERIODS))
            print("ADDRESS RECOVERY   : AUTO after {} consecutive I2C errors".format(
                RECOVERY_ERROR_THRESHOLD))
            print("STALE TIMEOUT      :", self.sensor_stale_ms, "ms")
            print("RANGE STATUS FILTER: ON (no-target returns = CLEAR, HW fail = ignored)")
            print("NEAR-FIELD BLANK   : S1 <{:.0f} mm, S2 <{:.0f} mm = sensor-face echo -> CLEAR".format(
                self.sensor1.near_blank_mm, self.sensor2.near_blank_mm))
            print("NEAR STRONG ECHO   : >= {:.1f} MCPS inside blank zone = PERSON (v2.1.12)".format(
                self.sensor1.near_blank_strong_mcps))
            print("WEAK ECHO REJECT   : S1 <{:.2f} MCPS, S2 <{:.2f} MCPS = not a person -> CLEAR".format(
                self.sensor1.min_presence_signal_mcps, self.sensor2.min_presence_signal_mcps))
            print("I/O TIMEOUT        :", self.io_timeout_ms, "ms (boot only)")
            self._report_calibration()
            self._background_sanity()
            if not self.ready:
                print("BACKGROUND CAPTURE : RUNNING (KEEP LANE EMPTY) - RFID BLOCKED UNTIL DONE")
            print("========================================")
            return self.ready

        except Exception as exc:
            self.last_error = repr(exc)
            self.hardware_ready = False
            self.ready = False
            self.presence_detected = False
            self.paused = True
            self.pause_reason = "INIT_FAILED"
            print("VL53L0X INITIALIZATION ERROR:", self.last_error, "| I2C:", self._scan_text())
            return False

    def rebind_i2c(self, i2c):
        """Swap the shared I2C object (e.g. 400 kHz -> 100 kHz fallback)."""
        self.i2c = i2c
        for hw in (self.sensor1_hw, self.sensor2_hw):
            if hw is not None:
                hw.i2c = i2c
        self._i2c_error_times = []

    # ========================================================
    # SCHEDULER CONTRACT
    # ========================================================

    @property
    def running(self):
        return bool(self.enabled and self.hardware_ready and not self.paused)

    def pause(self, reason="MOTION"):
        """Stop all I2C traffic immediately. No I2C transaction is issued."""
        if self.paused:
            return False
        self.paused = True
        self.pause_reason = str(reason)
        self.paused_since_ms = time.ticks_ms()
        self.pause_count += 1
        return True

    def resume(self, reason=""):
        """Resume polling. The sensors kept ranging in hardware while paused;
        the first post-pause sample is discarded so it cannot be stale."""
        if not self.paused or not self.enabled or not self.hardware_ready:
            return False
        now = time.ticks_ms()
        self.paused = False
        self.pause_reason = ""
        self.resumed_at_ms = now
        for index, sw in enumerate(self._software):
            sw.next_poll_ms = now
            sw.last_sample_t = None
            sw.discard_next = 1
            sw.alive_since_ms = now
            sw.reset_live(keep_present=True)
            try:
                self._hardware[index].clear_interrupt()
            except Exception as exc:
                self._note_io_error(index, exc, now)
        return True

    @staticmethod
    def _errno(exc):
        try:
            return int(exc.args[0])
        except Exception:
            return None

    def _note_io_error(self, index, exc, now_ms):
        sw = self._software[index] if index < len(self._software) else None
        nodev = self._errno(exc) == ENODEV
        if sw is not None:
            sw.io_errors += 1
            sw.consec_errors += 1
            if nodev:
                sw.nodev_errors += 1
            sw.next_poll_ms = time.ticks_add(now_ms, POLL_ERROR_BACKOFF_MS)
            if sw.consec_errors >= RECOVERY_ERROR_THRESHOLD:
                self._request_recovery(
                    "S{} {} consecutive I2C errors ({})".format(
                        index + 1, sw.consec_errors,
                        "address lost / sensor reset" if nodev else repr(exc),
                    ),
                    now_ms,
                )
        self.last_error = "S{}: {}".format(index + 1, repr(exc))

        # ENODEV = nobody at that address (sensor reset to 0x29). That is
        # NOT a bus-speed problem, so it must not trigger the 400 -> 100 kHz
        # fallback. Only NACK-mid-transfer / timeout errors count for that.
        if not nodev:
            self._i2c_error_times.append(now_ms)
            if len(self._i2c_error_times) > 40:
                self._i2c_error_times.pop(0)

        if (
            not self._last_error_print_ms
            or time.ticks_diff(now_ms, self._last_error_print_ms) >= ERROR_PRINT_THROTTLE_MS
        ):
            self._last_error_print_ms = now_ms
            print("VL53L0X I2C ERROR:", self.last_error, "| SYSTEM CONTINUES")

    # ========================================================
    # v2.0.1 ADDRESS-LOSS RECOVERY (cooperative, no long sleeps)
    # ========================================================

    def _request_recovery(self, reason, now_ms):
        if self.recovery_needed:
            return
        self.recovery_needed = True
        self.recovery_reason = str(reason)
        self.recovery_step = 0
        if not self.recovery_at_ms or time.ticks_diff(now_ms, self.recovery_at_ms) >= 0:
            self.recovery_at_ms = now_ms
        print("VL53L0X RECOVERY REQUESTED:", self.recovery_reason)
        print("  LIKELY CAUSE: sensor supply dip/EMI while motors run reset it to 0x29")

    def _fail_recovery(self, stage, exc):
        self.recovery_failures += 1
        self.recovery_step = 0
        self.recovery_at_ms = time.ticks_add(time.ticks_ms(), RECOVERY_RETRY_BACKOFF_MS)
        self.last_error = "RECOVERY {} failed: {}".format(stage, repr(exc))
        if self.recovery_failures <= 3 or self.recovery_failures % 10 == 0:
            print("VL53L0X RECOVERY FAILED at", stage, repr(exc),
                  "| I2C:", self._scan_text(), "| retry in", RECOVERY_RETRY_BACKOFF_MS, "ms")

    def service_recovery(self):
        """Advance the re-address sequence by one short step.

        Call only while the gate is stationary. Each step is a few I2C
        transactions (< ~5 ms); waits between steps are timestamps, never
        sleeps. Returns True while recovery is still in progress.
        """
        if not self.recovery_needed or not self.enabled:
            return False
        now = time.ticks_ms()
        if time.ticks_diff(now, self.recovery_at_ms) < 0:
            return True

        step = self.recovery_step
        try:
            if step == 0:
                if self.bus_recover_cb is not None:
                    try:
                        self.bus_recover_cb()
                    except Exception as exc:
                        print("VL53L0X BUS RECOVER CALLBACK ERROR:", repr(exc))
                if self.xshut1 is None:
                    self.xshut1 = Pin(self.xshut1_pin_num, Pin.OUT, value=0)
                if self.xshut2 is None:
                    self.xshut2 = Pin(self.xshut2_pin_num, Pin.OUT, value=0)
                self.xshut1.value(0)
                self.xshut2.value(0)
                self._recovery_hw = [None, None]
                self.recovery_step = 1
            elif step == 1:
                self.xshut1.value(1)
                self.recovery_step = 2
            elif step == 2:
                hw = self._make_hw()
                hw.set_address(self.sensor1_address)
                self._apply_measurement_profile(hw, "S1")
                self._recovery_hw[0] = hw
                self.xshut2.value(1)
                self.recovery_step = 3
            elif step == 3:
                hw = self._make_hw()
                hw.set_address(self.sensor2_address)
                self._apply_measurement_profile(hw, "S2")
                self._recovery_hw[1] = hw
                self.recovery_step = 4
            elif step == 4:
                self.sensor1_hw, self.sensor2_hw = self._recovery_hw
                self._hardware = [self.sensor1_hw, self.sensor2_hw]
                for index, sw in enumerate(self._software):
                    sw.hw = self._hardware[index]
                    sw.consec_errors = 0
                    self._start_sensor(index, now)
                self.recovery_needed = False
                self.recovery_step = 0
                self.recovery_failures = 0
                self.recovery_count += 1
                self.last_error = ""
                print("VL53L0X RECOVERED (#{}): S1=0x{:02X} S2=0x{:02X} | {}".format(
                    self.recovery_count, self.sensor1_address, self.sensor2_address,
                    self.recovery_reason))
                return False
        except Exception as exc:
            self._fail_recovery("STEP{}".format(step), exc)
            return True

        self.recovery_at_ms = time.ticks_add(now, RECOVERY_STEP_WAIT_MS)
        return True

    def recent_i2c_errors(self, window_ms=I2C_ERROR_WINDOW_MS):
        now = time.ticks_ms()
        count = 0
        for stamp in self._i2c_error_times:
            if time.ticks_diff(now, stamp) <= window_ms:
                count += 1
        return count

    def _watchdog_ms(self, sw):
        return max(
            int(self.measurement_timeout_ms),
            int(sw.period_ms * WATCHDOG_PERIODS),
            WATCHDOG_MIN_MS,
        )

    def _watchdog(self, index, now_ms):
        sw = self._software[index]
        reference = sw.last_sample_t if sw.last_sample_t is not None else sw.alive_since_ms
        silent_ms = time.ticks_diff(now_ms, reference)
        if silent_ms >= RECOVERY_NO_SAMPLE_MS and sw.restarts and sw.last_restart_ms:
            self._request_recovery(
                "S{} no sample for {} ms after restart".format(index + 1, silent_ms), now_ms)
            return
        if silent_ms < self._watchdog_ms(sw):
            return
        if sw.last_restart_ms and time.ticks_diff(now_ms, sw.last_restart_ms) < RESTART_THROTTLE_MS:
            return
        sw.last_restart_ms = now_ms
        sw.restarts += 1
        try:
            hw = self._hardware[index]
            hw.stop_continuous()
            self._start_sensor(index, now_ms)
            if sw.restarts <= 3 or sw.restarts % 20 == 0:
                print("VL53L0X S{} STALLED - CONTINUOUS RANGING RESTARTED (#{})".format(
                    index + 1, sw.restarts))
        except Exception as exc:
            sw.alive_since_ms = now_ms
            self._note_io_error(index, exc, now_ms)

    def _poll_sensor(self, index, now_ms):
        sw = self._software[index]
        if time.ticks_diff(now_ms, sw.next_poll_ms) < 0:
            return False

        hw = self._hardware[index]
        try:
            value = hw.poll_continuous_status()
        except Exception as exc:
            self._note_io_error(index, exc, now_ms)
            self._watchdog(index, now_ms)
            return False

        sw.consec_errors = 0

        if value is None:
            sw.next_poll_ms = time.ticks_add(now_ms, POLL_RETRY_MS)
            self._watchdog(index, now_ms)
            return False

        # Learn the real sample period for predictive polling.
        if sw.last_sample_t is not None:
            dt = time.ticks_diff(now_ms, sw.last_sample_t)
            if PERIOD_MIN_MS <= dt <= PERIOD_MAX_MS:
                sw.period_ms = sw.period_ms * 0.8 + dt * 0.2
        sw.last_sample_t = now_ms
        lead = int(sw.period_ms) - POLL_LEAD_MS
        if lead < 1:
            lead = 1
        sw.next_poll_ms = time.ticks_add(now_ms, lead)

        if sw.discard_next > 0:
            sw.discard_next -= 1
            return False

        sw.last_signal_mcps = hw.last_signal_mcps
        result = sw.process(value[0], now_ms, value[1])
        if result == "CAPTURED":
            print("VL53L0X S{} BACKGROUND CAPTURED: {:.0f} mm (corrected)".format(
                sw.sensor_no, sw.background_mm))
            if self._models_ready():
                self.background_save_pending = True
                print("VL53L0X BACKGROUND READY:", self._background_text())
                self._background_sanity()
        elif result == "UNSTABLE" and (sw.capture_rejects <= 3 or sw.capture_rejects % 10 == 0):
            print("VL53L0X S{} BACKGROUND CAPTURE UNSTABLE - LANE NOT EMPTY? retry #{}".format(
                sw.sensor_no, sw.capture_rejects))
        return True

    def update(self):
        """One cooperative step: at most one short poll per sensor."""
        if not self.enabled:
            self.global_state = STATUS_NO_PRESENCE
            self.presence_detected = False
            return
        if not self.hardware_ready or not self._hardware or self.paused:
            return
        if self.recovery_needed:
            return  # main.py drives service_recovery() while stationary

        now = time.ticks_ms()
        changed = self._poll_sensor(0, now)
        if self._poll_sensor(1, time.ticks_ms()):
            changed = True
        if changed:
            self._refresh_global_state(time.ticks_ms())

    # ========================================================
    # LOGICAL STATE
    # ========================================================

    def _models_ready(self):
        return bool(
            self.sensor1 is not None and self.sensor2 is not None
            and self.sensor1.ready() and self.sensor2.ready()
        )

    def _fresh_pair(self, now_ms):
        """All CONTEXT-ACTIVE sensors fresh (exit sensor ignored when closed)."""
        if self.sensor1 is None or self.sensor2 is None:
            return False
        for sensor in (self.sensor1, self.sensor2):
            if sensor.context_active and not sensor.is_fresh(now_ms):
                return False
        return True

    def set_gate_open(self, is_open):
        """v2.0.7: called by main.py. Exit-role sensors count only while the
        gate is stationary OPEN."""
        is_open = bool(is_open)
        if is_open == self.gate_open_context:
            return False
        self.gate_open_context = is_open
        for sensor in (self.sensor1, self.sensor2):
            if sensor is None or sensor.role != "exit":
                continue
            sensor.context_active = is_open
            sensor.reset_live(keep_present=False)
        return True

    def _presence_mask(self, now_ms):
        if self.sensor1 is None or self.sensor2 is None:
            return 0
        return (
            (1 if self.sensor1.effective_present(now_ms) else 0)
            | (2 if self.sensor2.effective_present(now_ms) else 0)
        )

    def _refresh_global_state(self, now_ms):
        if not self.enabled:
            self.global_state = STATUS_NO_PRESENCE
            self.presence_detected = False
            return
        if not self._models_ready():
            self.ready = False
            return

        if not self.ready:
            self.ready = True
            self.presence_detected = False
            self.global_state = STATUS_NO_PRESENCE

        mask = self._presence_mask(now_ms)
        fresh_pair = self._fresh_pair(now_ms)
        if mask != 0:
            self.presence_detected = True
            self.global_state = STATUS_PRESENCE
        elif fresh_pair:
            self.presence_detected = False
            self.global_state = STATUS_NO_PRESENCE

        data_valid = bool(self.ready and fresh_pair)

        # v2.0.1: print every PRESENCE/NO_PRESENCE change, but data-health
        # (OK/STALE) flips at most once per 5 s. v2.0.0 printed every flip,
        # which flooded the USB serial whenever one sensor dropped samples.
        state_changed = self.global_state != self._last_logged_state
        health_changed = data_valid != self._last_logged_data_valid
        if state_changed or (
            health_changed and (
                not self._last_health_log_ms
                or time.ticks_diff(now_ms, self._last_health_log_ms) >= HEALTH_LOG_THROTTLE_MS
            )
        ):
            print(
                "VL53L0X PRESENCE:", self.global_state,
                "| DATA={}".format("OK" if data_valid else "STALE"),
                "| S1={}".format(self._sensor_log_text(self.sensor1)),
                "| S2={}".format(self._sensor_log_text(self.sensor2)),
            )
            self._last_logged_state = self.global_state
            self._last_logged_data_valid = data_valid
            if health_changed:
                self._last_health_log_ms = now_ms

        # v2.0.1: PRESENCE that never clears blocks RFID forever and looks
        # like a frozen gate. Explain it on the serial log.
        if self.global_state == STATUS_PRESENCE:
            if not self._presence_since_ms:
                self._presence_since_ms = now_ms
            held = time.ticks_diff(now_ms, self._presence_since_ms)
            if held >= STUCK_PRESENCE_WARN_MS and (
                not self._stuck_warn_at_ms
                or time.ticks_diff(now_ms, self._stuck_warn_at_ms) >= STUCK_PRESENCE_REPEAT_MS
            ):
                self._stuck_warn_at_ms = now_ms
                print("VL53L0X WARNING: PRESENCE HELD {} s - RFID STAYS LOCKED".format(held // 1000))
                for sensor in (self.sensor1, self.sensor2):
                    if sensor is None or not sensor.present or not sensor.context_active:
                        continue
                    print("  S{} median={} mm | background={} mm ({}) | presence below {} mm".format(
                        sensor.sensor_no,
                        "-" if sensor.median_mm is None else int(sensor.median_mm),
                        "-" if sensor.background_mm is None else int(sensor.background_mm),
                        sensor.background_source,
                        "-" if sensor.enter_threshold() is None else int(sensor.enter_threshold()),
                    ))
                    print("  S{} range status last={} counts={} (11/0=real target, 4/6/8/9/10=no target)".format(
                        sensor.sensor_no, sensor.last_status, sensor.status_counts))
                    print("  S{} signal={} MCPS | near-blank <{} mm hits={}".format(
                        sensor.sensor_no, sensor.last_signal_mcps,
                        int(sensor.near_blank_mm), sensor.near_blank_count))
                print("  If the lane is EMPTY: something sits in that beam (gate leaf/frame?)")
                print("  or the background is wrong. Re-aim the sensor, or set its background")
                print("  to 0 and use 'Capture Empty-Lane Background' in the Web UI.")
        else:
            self._presence_since_ms = 0
            self._stuck_warn_at_ms = 0

    @staticmethod
    def _sensor_log_text(sensor):
        if sensor is None or sensor.median_mm is None:
            return "-"
        tag = ""
        if sensor.last_status is not None and classify_range_status(sensor.last_status) == SAMPLE_NO_TARGET:
            tag = "(no-target)"
        return "{:.0f}mm{}{}".format(sensor.median_mm, tag, "*" if sensor.present else "")

    def fresh_clear_now(self):
        """BOTH sensors fresh right now and neither present."""
        if not self.enabled:
            return True
        now = time.ticks_ms()
        return bool(
            self.ready and not self.paused
            and self._fresh_pair(now)
            and self._presence_mask(now) == 0
            and self.global_state == STATUS_NO_PRESENCE
        )

    def can_tap(self):
        return self.fresh_clear_now()

    def _recent_clear_pair(self, now_ms):
        if not self.ready or self.sensor1 is None or self.sensor2 is None:
            return False
        if self.global_state != STATUS_NO_PRESENCE:
            return False
        if self.sensor1.present or self.sensor2.present:
            return False
        if self.sensor1.last_valid_ms is None or self.sensor2.last_valid_ms is None:
            return False
        age1 = time.ticks_diff(now_ms, self.sensor1.last_valid_ms)
        age2 = time.ticks_diff(now_ms, self.sensor2.last_valid_ms)
        if age1 < 0 or age2 < 0:
            return False
        return bool(age1 <= self.close_clear_grace_ms and age2 <= self.close_clear_grace_ms)

    def safe_to_close(self):
        if not self.enabled or not self.require_clear_to_close:
            return True
        now = time.ticks_ms()
        if not self.ready:
            return False
        if self.global_state != STATUS_NO_PRESENCE:
            return False
        if self._presence_mask(now) != 0:
            return False
        if self._fresh_pair(now):
            return True
        if self.close_allow_stale_no_presence:
            return True
        return self._recent_clear_pair(now)

    def status(self):
        now = time.ticks_ms()

        if not self.enabled:
            return {
                "enabled": False,
                "hardware_ready": self.hardware_ready,
                "ready": True,
                "error": self.last_error,
                "global_state": STATUS_NO_PRESENCE,
                "presence_detected": False,
                "presence": False,
                "no_presence": True,
                "data_valid": True,
                "fresh_pair": True,
                "safe_to_tap": True,
                "safe_to_close": True,
                "paused": True,
                "pause_reason": "DISABLED",
                "require_presence_for_tap": False,
                "require_clear_to_close": self.require_clear_to_close,
                "reopen_on_obstruction": self.reopen_on_obstruction,
                "measurement_timeout_ms": self.measurement_timeout_ms,
                "sensor_stale_ms": self.sensor_stale_ms,
                "close_clear_grace_ms": self.close_clear_grace_ms,
                "close_allow_stale_no_presence": self.close_allow_stale_no_presence,
                "fast_mask": 0,
                "fast_state": "NONE",
            }

        fresh_pair = self._fresh_pair(now)
        mask = self._presence_mask(now)
        presence_now = bool(self.ready and self.global_state == STATUS_PRESENCE)

        def _sensor_status(sensor, number, angle):
            if sensor is None:
                return {
                    "name": "SENSOR #{}".format(number),
                    "angle": angle,
                    "state": "UNAVAILABLE",
                    "fresh": False,
                    "present": False,
                }
            return sensor.status(now, self.paused)

        return {
            "enabled": True,
            "hardware_ready": self.hardware_ready,
            "ready": self.ready,
            "error": self.last_error,
            "global_state": self.global_state,
            "presence_detected": presence_now,
            "presence": presence_now,
            "no_presence": bool(self.ready and self.global_state == STATUS_NO_PRESENCE),
            "data_valid": bool(self.ready and fresh_pair),
            "fresh_pair": fresh_pair,
            "safe_to_tap": self.can_tap(),
            "safe_to_close": self.safe_to_close(),

            "paused": bool(self.paused),
            "pause_reason": self.pause_reason,
            "pause_count": self.pause_count,
            "driver_profile": self.driver_profile,
            "timing_budget_us": self.timing_budget_us,
            "ranging_mode": "CONTINUOUS_BACK_TO_BACK",
            "background_save_pending": bool(self.background_save_pending),
            "recent_i2c_errors": self.recent_i2c_errors(),
            "recovery_needed": bool(self.recovery_needed),
            "recovery_reason": self.recovery_reason,
            "recovery_count": self.recovery_count,
            "recovery_failures": self.recovery_failures,

            "require_presence_for_tap": False,
            "require_clear_to_close": self.require_clear_to_close,
            "reopen_on_obstruction": self.reopen_on_obstruction,
            "measurement_timeout_ms": self.measurement_timeout_ms,
            "sensor_stale_ms": self.sensor_stale_ms,
            "close_clear_grace_ms": self.close_clear_grace_ms,
            "close_allow_stale_no_presence": self.close_allow_stale_no_presence,
            "recent_clear_for_close": self._recent_clear_pair(now),

            "fast_mask": mask,
            "fast_state": (
                "NONE" if mask == 0 else
                "S1" if mask == 1 else
                "S2" if mask == 2 else
                "S1+S2"
            ),
            "sensor1": _sensor_status(self.sensor1, 1, self.sensor1_angle),
            "sensor2": _sensor_status(self.sensor2, 2, self.sensor2_angle),
        }
