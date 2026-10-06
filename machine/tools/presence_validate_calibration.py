# ============================================================
# FASTLANE PRESENCE CALIBRATION VALIDATOR
# File: /tools/presence_validate_calibration.py
# ============================================================
#
# PURPOSE
# - Validate BOTH VL53L0X sensors together at each distance.
# - Position one large flat target ONCE at each distance.
# - Sensor #1 and Sensor #2 are measured before moving target.
#
# TEST DISTANCES
#   5 cm, 10 cm, 15 cm, 20 cm
#
# USES EXISTING
#   /tools/presence_sensor_pair.py
#   /presence_range_calibration.json
#
# DOES NOT MODIFY CALIBRATION.
#
# RESULT GUIDE
#   <= +/-2.0 cm : PASS
#   <= +/-3.0 cm : USABLE
#   >  +/-3.0 cm : CHECK
# ============================================================

try:
    import ujson as json
except ImportError:
    import json

from tools.presence_sensor_pair import PresenceSensorPair, sample_sensor, median


CALIBRATION_FILE = "/presence_range_calibration.json"

TEST_DISTANCES_CM = (5, 10, 15, 20)

SAMPLES_PER_SENSOR = 30
SAMPLE_INTERVAL_MS = 40

PASS_ERROR_CM = 2.0
USABLE_ERROR_CM = 3.0


# ============================================================
# HELPERS
# ============================================================

def _load_calibration():
    try:
        with open(CALIBRATION_FILE, "r") as f:
            raw = f.read()
    except OSError as exc:
        raise RuntimeError(
            "Cannot open {}: {}".format(
                CALIBRATION_FILE,
                repr(exc),
            )
        )

    try:
        data = json.loads(raw)
    except Exception as exc:
        print()
        print("INVALID CALIBRATION JSON")
        print("FILE:", CALIBRATION_FILE)
        print("CONTENT:")
        print(raw)
        raise RuntimeError(
            "Calibration JSON is invalid: {}".format(
                repr(exc)
            )
        )

    if not isinstance(data, dict):
        raise RuntimeError(
            "Calibration file must contain a JSON object."
        )

    if "sensor1_offset_mm" not in data:
        raise RuntimeError(
            "Missing sensor1_offset_mm in calibration file."
        )

    if "sensor2_offset_mm" not in data:
        raise RuntimeError(
            "Missing sensor2_offset_mm in calibration file."
        )

    return (
        data,
        float(data["sensor1_offset_mm"]),
        float(data["sensor2_offset_mm"]),
    )


def _verdict(error_cm):
    absolute_error = abs(float(error_cm))

    if absolute_error <= PASS_ERROR_CM:
        return "PASS"

    if absolute_error <= USABLE_ERROR_CM:
        return "USABLE"

    return "CHECK"


def _measure_sensor(
    pair,
    sensor_no,
    actual_cm,
    offset_mm,
):
    values = sample_sensor(
        pair,
        sensor_no,
        count=SAMPLES_PER_SENSOR,
        interval_ms=SAMPLE_INTERVAL_MS,
    )

    raw_median_mm = median(values)

    if raw_median_mm is None:
        return {
            "sensor": int(sensor_no),
            "actual_cm": float(actual_cm),
            "valid": False,
        }

    raw_mm = float(raw_median_mm)
    raw_cm = raw_mm / 10.0

    corrected_mm = raw_mm + float(offset_mm)
    corrected_cm = corrected_mm / 10.0

    raw_error_cm = raw_cm - float(actual_cm)
    corrected_error_cm = corrected_cm - float(actual_cm)

    return {
        "sensor": int(sensor_no),
        "actual_cm": float(actual_cm),
        "actual_mm": float(actual_cm) * 10.0,

        "raw_mm": raw_mm,
        "raw_cm": raw_cm,

        "offset_mm": float(offset_mm),

        "corrected_mm": corrected_mm,
        "corrected_cm": corrected_cm,

        "raw_error_cm": raw_error_cm,
        "corrected_error_cm": corrected_error_cm,

        "valid_samples": len(values),
        "valid": True,

        "verdict": _verdict(
            corrected_error_cm
        ),
    }


def _print_measurement(result):
    sensor_no = result["sensor"]

    if not result.get("valid"):
        print(
            "S{} | NO VALID SAMPLES".format(
                sensor_no
            )
        )
        return

    print(
        "S{} | raw {:>6.2f} cm | corrected {:>6.2f} cm | "
        "error {:+6.2f} cm | {}".format(
            sensor_no,
            result["raw_cm"],
            result["corrected_cm"],
            result["corrected_error_cm"],
            result["verdict"],
        )
    )


def _print_sensor_summary(
    sensor_no,
    offset_mm,
    results,
):
    print()
    print("============================================================")
    print("SENSOR #{} SUMMARY".format(sensor_no))
    print("============================================================")
    print(
        "Saved offset: {:+.2f} mm".format(
            offset_mm
        )
    )

    valid_results = [
        item
        for item in results
        if item.get("valid")
    ]

    if not valid_results:
        print("No valid measurements.")
        return

    for item in valid_results:
        print(
            "{:>5.1f} cm | raw {:>6.2f} cm | "
            "corrected {:>6.2f} cm | "
            "error {:+6.2f} cm | {}".format(
                item["actual_cm"],
                item["raw_cm"],
                item["corrected_cm"],
                item["corrected_error_cm"],
                item["verdict"],
            )
        )

    absolute_errors = [
        abs(item["corrected_error_cm"])
        for item in valid_results
    ]

    mean_abs_error = (
        sum(absolute_errors)
        /
        len(absolute_errors)
    )

    maximum_abs_error = max(
        absolute_errors
    )

    pass_count = sum(
        1
        for item in valid_results
        if item["verdict"] == "PASS"
    )

    usable_count = sum(
        1
        for item in valid_results
        if item["verdict"] in (
            "PASS",
            "USABLE",
        )
    )

    print()
    print(
        "PASS points          : {}/{}".format(
            pass_count,
            len(valid_results),
        )
    )
    print(
        "PASS/USABLE points   : {}/{}".format(
            usable_count,
            len(valid_results),
        )
    )
    print(
        "Mean abs error       : {:.2f} cm".format(
            mean_abs_error
        )
    )
    print(
        "Maximum abs error    : {:.2f} cm".format(
            maximum_abs_error
        )
    )


# ============================================================
# MAIN
# ============================================================

def main():
    print()
    print("============================================================")
    print("FASTLANE PRESENCE CALIBRATION VALIDATION")
    print("DUAL-SENSOR / SAME-DISTANCE TEST")
    print("============================================================")
    print("Calibration file:", CALIBRATION_FILE)
    print(
        "Test distances:",
        ", ".join(
            "{} cm".format(x)
            for x in TEST_DISTANCES_CM
        ),
    )
    print(
        "PASS <= +/-{:.1f} cm | USABLE <= +/-{:.1f} cm".format(
            PASS_ERROR_CM,
            USABLE_ERROR_CM,
        )
    )
    print("This test does NOT modify calibration.")
    print("============================================================")

    data, sensor1_offset, sensor2_offset = _load_calibration()

    print()
    print("CALIBRATION LOADED")
    print(
        "Sensor #1 offset: {:+.2f} mm".format(
            sensor1_offset
        )
    )
    print(
        "Sensor #2 offset: {:+.2f} mm".format(
            sensor2_offset
        )
    )

    pair = PresenceSensorPair()
    pair.initialize()

    sensor1_results = []
    sensor2_results = []

    for actual_cm in TEST_DISTANCES_CM:
        print()
        print("============================================================")
        print("TARGET DISTANCE: {} cm".format(actual_cm))
        print("============================================================")
        print(
            "Place ONE large flat target exactly {} cm from BOTH sensors.".format(
                actual_cm
            )
        )
        print(
            "Do NOT move the target between Sensor #1 and Sensor #2."
        )
        print()
        print("ENTER = measure both sensors")
        print("S     = skip this distance")
        print("Q     = stop validation")

        answer = input("> ").strip().lower()

        if answer == "s":
            print(
                "{} cm skipped.".format(
                    actual_cm
                )
            )
            continue

        if answer == "q":
            print("Validation stopped by user.")
            break

        print()
        print("Measuring Sensor #1...")
        s1 = _measure_sensor(
            pair,
            1,
            actual_cm,
            sensor1_offset,
        )
        sensor1_results.append(s1)
        _print_measurement(s1)

        print("Measuring Sensor #2...")
        s2 = _measure_sensor(
            pair,
            2,
            actual_cm,
            sensor2_offset,
        )
        sensor2_results.append(s2)
        _print_measurement(s2)

        print()
        print(
            "{} cm COMPLETE for BOTH sensors.".format(
                actual_cm
            )
        )

    _print_sensor_summary(
        1,
        sensor1_offset,
        sensor1_results,
    )

    _print_sensor_summary(
        2,
        sensor2_offset,
        sensor2_results,
    )

    print()
    print("============================================================")
    print("VALIDATION COMPLETE")
    print("============================================================")
    print("No calibration values were changed.")
    print("No new file was created.")
    print("============================================================")


main()
