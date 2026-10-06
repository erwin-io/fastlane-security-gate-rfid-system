# FASTLANE standalone dual-VL53L0X helper
#
# Compatible with the CURRENT FASTLANE components/vl53l0x.py driver,
# which uses cooperative:
#
#     begin_single()
#     poll_single()
#
# instead of a blocking read() method.
#
# Used by:
#   /tools/presence_calibrate.py
#   /tools/presence_collect.py
#
# IMPORTANT:
# - Run only while FASTLANE main.py is stopped.
# - No SD card is required.
# - The production /components/vl53l0x.py is NOT modified.

import time
from machine import Pin, I2C

from components.vl53l0x import VL53L0X


# ============================================================
# HARDWARE - SAME AS PRODUCTION FASTLANE
# ============================================================

I2C_ID = 0
SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 100000

XSHUT1_PIN = 47
XSHUT2_PIN = 48

DEFAULT_ADDRESS = 0x29
SENSOR1_ADDRESS = 0x30
SENSOR2_ADDRESS = 0x31

# Calibration can wait longer than the production cooperative loop.
DEFAULT_TIMEOUT_MS = 300

# Reject 20 mm near-limit/noise returns seen in real FASTLANE logs.
MIN_VALID_MM = 50
MAX_VALID_MM = 4000


class PresenceSensorPair:
    def __init__(self):
        self.i2c = None
        self.sensor1 = None
        self.sensor2 = None
        self.x1 = None
        self.x2 = None

    # ========================================================
    # INITIALIZATION
    # ========================================================

    def initialize(self):
        # Both VL53L0X modules power up at 0x29, therefore both must
        # first be forced into XSHUT before assigning separate addresses.
        self.x1 = Pin(XSHUT1_PIN, Pin.OUT, value=0)
        self.x2 = Pin(XSHUT2_PIN, Pin.OUT, value=0)
        time.sleep_ms(150)

        self.i2c = I2C(
            I2C_ID,
            sda=Pin(SDA_PIN),
            scl=Pin(SCL_PIN),
            freq=I2C_FREQ,
        )

        # Sensor #1: 0x29 -> 0x30
        self.x1.value(1)
        time.sleep_ms(150)

        self.sensor1 = VL53L0X(
            self.i2c,
            DEFAULT_ADDRESS,
        )
        self.sensor1.set_address(
            SENSOR1_ADDRESS
        )

        time.sleep_ms(100)

        # Sensor #2: 0x29 -> 0x31
        self.x2.value(1)
        time.sleep_ms(150)

        self.sensor2 = VL53L0X(
            self.i2c,
            DEFAULT_ADDRESS,
        )
        self.sensor2.set_address(
            SENSOR2_ADDRESS
        )

        time.sleep_ms(150)

        devices = self.i2c.scan()

        if SENSOR1_ADDRESS not in devices:
            raise RuntimeError(
                "VL53L0X Sensor #1 not detected at 0x30; I2C={}".format(
                    [hex(x) for x in devices]
                )
            )

        if SENSOR2_ADDRESS not in devices:
            raise RuntimeError(
                "VL53L0X Sensor #2 not detected at 0x31; I2C={}".format(
                    [hex(x) for x in devices]
                )
            )

        print("VL53L0X PAIR READY")
        print(
            "I2C:",
            [hex(x) for x in devices],
        )
        print(
            "I2C SPEED:",
            I2C_FREQ,
            "Hz",
        )
        print(
            "S1: 0x30 | XSHUT GPIO47"
        )
        print(
            "S2: 0x31 | XSHUT GPIO48"
        )
        print(
            "DRIVER API: begin_single() + poll_single()"
        )

        self._warmup_sensor(1)
        self._warmup_sensor(2)

        return True

    def _sensor_for(self, sensor_no):
        if int(sensor_no) == 1:
            return self.sensor1
        return self.sensor2

    # ========================================================
    # CURRENT FASTLANE DRIVER ADAPTER
    # ========================================================

    def _blocking_cooperative_read(
        self,
        sensor,
        timeout_ms,
    ):
        """Use the production driver's cooperative API as one blocking
        measurement for standalone calibration/data-collection tools.

        The production FASTLANE main loop itself remains non-blocking.
        """

        timeout_ms = max(
            10,
            int(timeout_ms),
        )

        # A previous interrupted standalone run may have left the software
        # state busy. Reset it before starting a new calibration sample.
        try:
            if sensor.busy:
                sensor.cancel()
                time.sleep_ms(2)
        except Exception:
            pass

        started = sensor.begin_single(
            timeout_ms
        )

        if not started:
            try:
                sensor.cancel()
            except Exception:
                pass

            time.sleep_ms(2)

            started = sensor.begin_single(
                timeout_ms
            )

            if not started:
                return None

        # poll_single() has its own timeout/deadline. This extra deadline
        # prevents the standalone tool from ever becoming stuck if something
        # unexpected happens.
        hard_deadline = time.ticks_add(
            time.ticks_ms(),
            timeout_ms + 100,
        )

        while True:
            done, value = sensor.poll_single()

            if done:
                return value

            if (
                time.ticks_diff(
                    time.ticks_ms(),
                    hard_deadline,
                )
                >= 0
            ):
                try:
                    sensor.cancel()
                except Exception:
                    pass
                return None

            # The production code polls cooperatively from its main loop.
            # The standalone calibration tool can simply wait a little
            # between polls.
            time.sleep_ms(2)

    def _read_driver(
        self,
        sensor,
        timeout_ms,
    ):
        # CURRENT FASTLANE DRIVER
        if (
            hasattr(sensor, "begin_single")
            and
            hasattr(sensor, "poll_single")
        ):
            return self._blocking_cooperative_read(
                sensor,
                timeout_ms,
            )

        # Compatibility fallback only for older FASTLANE drivers that had
        # a blocking read() method.
        if hasattr(sensor, "read"):
            try:
                return sensor.read(
                    timeout_ms=timeout_ms
                )
            except TypeError:
                try:
                    return sensor.read(
                        timeout_ms
                    )
                except TypeError:
                    return sensor.read()

        raise AttributeError(
            "Unsupported VL53L0X driver: expected "
            "begin_single()/poll_single() or read()"
        )

    # ========================================================
    # READ HELPERS
    # ========================================================

    def _warmup_sensor(
        self,
        sensor_no,
    ):
        sensor = self._sensor_for(
            sensor_no
        )

        if sensor is None:
            return

        for _ in range(3):
            try:
                value = self._read_driver(
                    sensor,
                    DEFAULT_TIMEOUT_MS,
                )

                if value is not None:
                    value = int(value)

                    if (
                        MIN_VALID_MM
                        <= value
                        <= MAX_VALID_MM
                    ):
                        print(
                            "S{} WARMUP: {} mm".format(
                                int(sensor_no),
                                value,
                            )
                        )
                        return

            except Exception as exc:
                print(
                    "S{} WARMUP ERROR: {}".format(
                        int(sensor_no),
                        repr(exc),
                    )
                )

            time.sleep_ms(80)

        print(
            "S{} WARMUP: no valid reading yet - "
            "calibration will retry".format(
                int(sensor_no)
            )
        )

    def read_one_raw(
        self,
        sensor_no,
        timeout_ms=DEFAULT_TIMEOUT_MS,
    ):
        """Return one unfiltered driver result.

        Returns:
            (value, error_text)
        """

        sensor = self._sensor_for(
            sensor_no
        )

        if sensor is None:
            return (
                None,
                "sensor not initialized",
            )

        try:
            value = self._read_driver(
                sensor,
                int(timeout_ms),
            )
            return value, ""

        except Exception as exc:
            return None, repr(exc)

    def read_one(
        self,
        sensor_no,
        timeout_ms=DEFAULT_TIMEOUT_MS,
    ):
        """Return one valid distance in millimeters."""

        for attempt in range(2):
            value, error = self.read_one_raw(
                sensor_no,
                timeout_ms,
            )

            if error:
                value = None

            try:
                value = int(value)
            except Exception:
                value = None

            if (
                value is not None
                and
                MIN_VALID_MM
                <= value
                <= MAX_VALID_MM
            ):
                return value

            if attempt == 0:
                time.sleep_ms(20)

        return None

    def read_pair(
        self,
        timeout_ms=DEFAULT_TIMEOUT_MS,
    ):
        # Keep the two sensors sequential.
        s1 = self.read_one(
            1,
            timeout_ms,
        )

        time.sleep_ms(3)

        s2 = self.read_one(
            2,
            timeout_ms,
        )

        return s1, s2

    # ========================================================
    # TROUBLESHOOTING
    # ========================================================

    def diagnostic_read(
        self,
        sensor_no,
        count=10,
    ):
        print()
        print(
            "SENSOR #{} RAW RANGE DIAGNOSTIC".format(
                int(sensor_no)
            )
        )

        valid = 0

        for index in range(
            int(count)
        ):
            value, error = self.read_one_raw(
                sensor_no,
                DEFAULT_TIMEOUT_MS,
            )

            if error:
                print(
                    "#{:02d}: ERROR {}".format(
                        index + 1,
                        error,
                    )
                )

            else:
                print(
                    "#{:02d}: {}".format(
                        index + 1,
                        value,
                    )
                )

                try:
                    ivalue = int(value)

                    if (
                        MIN_VALID_MM
                        <= ivalue
                        <= MAX_VALID_MM
                    ):
                        valid += 1

                except Exception:
                    pass

            time.sleep_ms(80)

        print(
            "VALID {}/{}".format(
                valid,
                int(count),
            )
        )

        return valid


# ============================================================
# CALIBRATION/DATA COLLECTION HELPERS
# ============================================================

def median(values):
    values = sorted(
        int(v)
        for v in values
        if v is not None
    )

    if not values:
        return None

    count = len(values)
    middle = count // 2

    if count % 2:
        return values[middle]

    return (
        values[middle - 1]
        + values[middle]
    ) / 2.0


def sample_sensor(
    pair,
    sensor_no,
    count=40,
    interval_ms=50,
    timeout_ms=DEFAULT_TIMEOUT_MS,
):
    values = []

    for _ in range(
        int(count)
    ):
        value = pair.read_one(
            sensor_no,
            timeout_ms,
        )

        if value is not None:
            values.append(
                value
            )

        time.sleep_ms(
            int(interval_ms)
        )

    return values
