# ============================================================
# FASTLANE PRESENCE CALIBRATION
# File: /tools/presence_calibrate.py
# ============================================================
#
# MODES:
#   1 = Range calibration (5 / 10 / 15 / 20 cm)
#   2 = Empty-gate background calibration
#
# Uses existing:
#   /tools/presence_sensor_pair.py
#
# Saves existing:
#   /presence_range_calibration.json
#
# No SD card required.
# ============================================================

import os

try:
    import ujson as json
except ImportError:
    import json

from tools.presence_sensor_pair import PresenceSensorPair, sample_sensor, median


CALIBRATION_FILE = "/presence_range_calibration.json"

RANGE_DISTANCES_CM = (5, 10, 15, 20)
RANGE_SAMPLES_PER_SENSOR = 30
RANGE_SAMPLE_INTERVAL_MS = 40

BACKGROUND_BATCHES = 5
BACKGROUND_SAMPLES_PER_BATCH = 20
BACKGROUND_SAMPLE_INTERVAL_MS = 40
BACKGROUND_STABLE_SPREAD_MM = 30.0


def _mean(values):
    if not values:
        return None
    return sum(values) / len(values)


def _percentile(values, fraction):
    if not values:
        return None

    ordered = sorted(float(v) for v in values)

    if len(ordered) == 1:
        return ordered[0]

    pos = (len(ordered) - 1) * float(fraction)
    low = int(pos)
    high = low + 1

    if high >= len(ordered):
        return ordered[-1]

    weight = pos - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _load_existing_calibration(required=False):
    try:
        with open(CALIBRATION_FILE, "r") as f:
            raw = f.read()
    except OSError:
        if required:
            raise RuntimeError(
                "Missing {}. Run range calibration first.".format(
                    CALIBRATION_FILE
                )
            )
        return {}

    try:
        data = json.loads(raw)
    except Exception as exc:
        raise RuntimeError(
            "Invalid JSON in {}: {}".format(
                CALIBRATION_FILE,
                repr(exc),
            )
        )

    if not isinstance(data, dict):
        raise RuntimeError(
            "{} must contain a JSON object.".format(
                CALIBRATION_FILE
            )
        )

    return data


def _atomic_save_json(path, data):
    temp_path = path + ".tmp"

    with open(temp_path, "w") as f:
        f.write(json.dumps(data))

    try:
        if hasattr(os, "sync"):
            os.sync()
    except Exception:
        pass

    try:
        os.remove(path)
    except OSError:
        pass

    os.rename(temp_path, path)

    try:
        if hasattr(os, "sync"):
            os.sync()
    except Exception:
        pass


# ============================================================
# RANGE CALIBRATION
# ============================================================

def _measure_range_sensor(pair, sensor_no, actual_cm):
    actual_mm = float(actual_cm) * 10.0

    values = sample_sensor(
        pair,
        sensor_no,
        count=RANGE_SAMPLES_PER_SENSOR,
        interval_ms=RANGE_SAMPLE_INTERVAL_MS,
    )

    raw_median_mm = median(values)

    if raw_median_mm is None:
        raise RuntimeError(
            "Sensor #{} produced no valid samples at {} cm.".format(
                sensor_no,
                actual_cm,
            )
        )

    raw_median_mm = float(raw_median_mm)
    point_offset_mm = actual_mm - raw_median_mm

    return {
        "actual_cm": float(actual_cm),
        "actual_mm": actual_mm,
        "median_raw_mm": raw_median_mm,
        "offset_mm": point_offset_mm,
        "valid_samples": len(values),
    }


def _calculate_range_result(points):
    offsets = [float(item["offset_mm"]) for item in points]

    return {
        "offset_mm": round(_mean(offsets), 3),
        "offset_spread_mm": round(max(offsets) - min(offsets), 3),
        "points": points,
    }


def _print_range_summary(sensor_no, result):
    print()
    print("============================================================")
    print("SENSOR #{} RANGE CALIBRATION SUMMARY".format(sensor_no))
    print("============================================================")

    for item in result["points"]:
        print(
            "{:>5.1f} cm | raw {:>6.1f} mm | point offset {:+6.1f} mm".format(
                item["actual_cm"],
                item["median_raw_mm"],
                item["offset_mm"],
            )
        )

    print()
    print("FINAL OFFSET : {:+.3f} mm".format(result["offset_mm"]))
    print("OFFSET SPREAD: {:.1f} mm".format(result["offset_spread_mm"]))


def run_range_calibration(pair):
    print()
    print("============================================================")
    print("FASTLANE RANGE CALIBRATION")
    print("BOTH SENSORS AT EACH DISTANCE")
    print("============================================================")
    print("Distances: 5 cm, 10 cm, 15 cm, 20 cm")
    print("Use ONE large flat target covering BOTH sensor beams.")
    print("============================================================")

    sensor1_points = []
    sensor2_points = []

    for actual_cm in RANGE_DISTANCES_CM:
        print()
        print("============================================================")
        print("TARGET DISTANCE: {} cm".format(actual_cm))
        print("============================================================")
        print(
            "Place ONE flat target exactly {} cm from BOTH sensors.".format(
                actual_cm
            )
        )
        print("Do not move the target between S1 and S2.")
        input("Press ENTER when stable...")

        print("Measuring Sensor #1...")
        s1 = _measure_range_sensor(pair, 1, actual_cm)
        sensor1_points.append(s1)
        print(
            "S1 | raw {:.1f} mm | point offset {:+.1f} mm".format(
                s1["median_raw_mm"],
                s1["offset_mm"],
            )
        )

        print("Measuring Sensor #2...")
        s2 = _measure_range_sensor(pair, 2, actual_cm)
        sensor2_points.append(s2)
        print(
            "S2 | raw {:.1f} mm | point offset {:+.1f} mm".format(
                s2["median_raw_mm"],
                s2["offset_mm"],
            )
        )

    sensor1_result = _calculate_range_result(sensor1_points)
    sensor2_result = _calculate_range_result(sensor2_points)

    _print_range_summary(1, sensor1_result)
    _print_range_summary(2, sensor2_result)

    data = {
        "schema": "fastlane_presence_range_calibration",
        "schema_version": 2,
        "calibration_method": "dual_sensor_multi_point_constant_offset",
        "calibration_distances_cm": list(RANGE_DISTANCES_CM),

        "sensor1_offset_mm": sensor1_result["offset_mm"],
        "sensor2_offset_mm": sensor2_result["offset_mm"],

        "sensor1_offset_spread_mm": sensor1_result["offset_spread_mm"],
        "sensor2_offset_spread_mm": sensor2_result["offset_spread_mm"],

        "sensor1_points": sensor1_result["points"],
        "sensor2_points": sensor2_result["points"],

        "background_calibrated": False,
    }

    _atomic_save_json(CALIBRATION_FILE, data)

    print()
    print("============================================================")
    print("RANGE CALIBRATION COMPLETE")
    print("============================================================")
    print("Sensor #1 offset: {:+.3f} mm".format(sensor1_result["offset_mm"]))
    print("Sensor #2 offset: {:+.3f} mm".format(sensor2_result["offset_mm"]))
    print("Saved:", CALIBRATION_FILE)
    print("============================================================")


# ============================================================
# BACKGROUND CALIBRATION
# ============================================================

def _make_background_result(all_values, batch_medians, offset_mm):
    raw_median_mm = float(median(all_values))
    raw_mean_mm = _mean(all_values)

    p05_mm = _percentile(all_values, 0.05)
    p95_mm = _percentile(all_values, 0.95)
    spread_90_mm = p95_mm - p05_mm

    corrected_median_mm = raw_median_mm + float(offset_mm)

    return {
        "raw_median_mm": round(raw_median_mm, 3),
        "corrected_median_mm": round(corrected_median_mm, 3),
        "raw_mean_mm": round(raw_mean_mm, 3),
        "raw_min_mm": round(min(all_values), 3),
        "raw_max_mm": round(max(all_values), 3),
        "raw_p05_mm": round(p05_mm, 3),
        "raw_p95_mm": round(p95_mm, 3),
        "spread_90_mm": round(spread_90_mm, 3),
        "valid_samples": len(all_values),
        "batch_medians_mm": [round(v, 3) for v in batch_medians],
        "stable": spread_90_mm <= BACKGROUND_STABLE_SPREAD_MM,
    }


def _print_background_summary(sensor_no, result):
    print()
    print("============================================================")
    print("SENSOR #{} EMPTY BACKGROUND".format(sensor_no))
    print("============================================================")
    print("Raw median       : {:.1f} mm".format(result["raw_median_mm"]))
    print("Corrected median : {:.1f} mm".format(result["corrected_median_mm"]))
    print("Raw mean         : {:.1f} mm".format(result["raw_mean_mm"]))
    print("5th percentile   : {:.1f} mm".format(result["raw_p05_mm"]))
    print("95th percentile  : {:.1f} mm".format(result["raw_p95_mm"]))
    print("90% spread       : {:.1f} mm".format(result["spread_90_mm"]))
    print("Valid samples    : {}".format(result["valid_samples"]))
    print(
        "Stability        : {}".format(
            "PASS" if result["stable"] else "CHECK"
        )
    )


def run_background_calibration(pair):
    data = _load_existing_calibration(required=True)

    if "sensor1_offset_mm" not in data:
        raise RuntimeError(
            "sensor1_offset_mm missing. Run range calibration first."
        )

    if "sensor2_offset_mm" not in data:
        raise RuntimeError(
            "sensor2_offset_mm missing. Run range calibration first."
        )

    sensor1_offset_mm = float(data["sensor1_offset_mm"])
    sensor2_offset_mm = float(data["sensor2_offset_mm"])

    print()
    print("============================================================")
    print("FASTLANE EMPTY-GATE BACKGROUND CALIBRATION")
    print("============================================================")
    print("Sensors must already be in FINAL gate positions.")
    print("Sensor #1 = final 70-degree mounting.")
    print("Sensor #2 = final 110-degree mounting.")
    print("Passage must be COMPLETELY EMPTY.")
    print("Do not stand in either sensor beam.")
    print()
    print(
        "Each sensor: {} batches x {} samples = {} samples.".format(
            BACKGROUND_BATCHES,
            BACKGROUND_SAMPLES_PER_BATCH,
            BACKGROUND_BATCHES * BACKGROUND_SAMPLES_PER_BATCH,
        )
    )
    print("============================================================")

    input("Clear the gate, then press ENTER to begin...")

    sensor1_all = []
    sensor2_all = []
    sensor1_batch_medians = []
    sensor2_batch_medians = []

    print()
    print("Collecting empty-gate background...")

    for batch_no in range(1, BACKGROUND_BATCHES + 1):
        s1_values = sample_sensor(
            pair,
            1,
            count=BACKGROUND_SAMPLES_PER_BATCH,
            interval_ms=BACKGROUND_SAMPLE_INTERVAL_MS,
        )

        if not s1_values:
            raise RuntimeError(
                "Sensor #1 produced no valid samples in batch {}.".format(
                    batch_no
                )
            )

        s1_median = float(median(s1_values))
        sensor1_all.extend(float(v) for v in s1_values)
        sensor1_batch_medians.append(s1_median)

        s2_values = sample_sensor(
            pair,
            2,
            count=BACKGROUND_SAMPLES_PER_BATCH,
            interval_ms=BACKGROUND_SAMPLE_INTERVAL_MS,
        )

        if not s2_values:
            raise RuntimeError(
                "Sensor #2 produced no valid samples in batch {}.".format(
                    batch_no
                )
            )

        s2_median = float(median(s2_values))
        sensor2_all.extend(float(v) for v in s2_values)
        sensor2_batch_medians.append(s2_median)

        print(
            "Batch {}/{} | S1 {:>7.1f} mm | S2 {:>7.1f} mm".format(
                batch_no,
                BACKGROUND_BATCHES,
                s1_median,
                s2_median,
            )
        )

    sensor1_result = _make_background_result(
        sensor1_all,
        sensor1_batch_medians,
        sensor1_offset_mm,
    )

    sensor2_result = _make_background_result(
        sensor2_all,
        sensor2_batch_medians,
        sensor2_offset_mm,
    )

    _print_background_summary(1, sensor1_result)
    _print_background_summary(2, sensor2_result)

    data["schema_version"] = 3
    data["background_calibrated"] = True
    data["background_method"] = "empty_gate_robust_median"

    # RAW values stay authoritative for current production ToF logic.
    data["sensor1_background_mm"] = sensor1_result["raw_median_mm"]
    data["sensor2_background_mm"] = sensor2_result["raw_median_mm"]

    # Corrected values kept for diagnostics / future AI feature work.
    data["sensor1_background_corrected_mm"] = (
        sensor1_result["corrected_median_mm"]
    )
    data["sensor2_background_corrected_mm"] = (
        sensor2_result["corrected_median_mm"]
    )

    data["sensor1_background"] = sensor1_result
    data["sensor2_background"] = sensor2_result

    _atomic_save_json(CALIBRATION_FILE, data)

    print()
    print("============================================================")
    print("BACKGROUND CALIBRATION COMPLETE")
    print("============================================================")
    print(
        "Sensor #1 RAW background: {:.1f} mm".format(
            sensor1_result["raw_median_mm"]
        )
    )
    print(
        "Sensor #2 RAW background: {:.1f} mm".format(
            sensor2_result["raw_median_mm"]
        )
    )
    print(
        "Sensor #1 corrected background: {:.1f} mm".format(
            sensor1_result["corrected_median_mm"]
        )
    )
    print(
        "Sensor #2 corrected background: {:.1f} mm".format(
            sensor2_result["corrected_median_mm"]
        )
    )
    print()
    print("Saved into SAME file:")
    print(CALIBRATION_FILE)

    if sensor1_result["stable"] and sensor2_result["stable"]:
        print()
        print("BACKGROUND STABILITY: PASS")
        print("NEXT: test presence/intrusion.")
    else:
        print()
        print("BACKGROUND STABILITY: CHECK")
        print(
            "One or both sensors varied by more than {:.0f} mm.".format(
                BACKGROUND_STABLE_SPREAD_MM
            )
        )

    print("============================================================")


# ============================================================
# MAIN MENU
# ============================================================

def main():
    print()
    print("============================================================")
    print("FASTLANE PRESENCE CALIBRATION")
    print("============================================================")
    print("1 = Range calibration")
    print("    5 / 10 / 15 / 20 cm")
    print()
    print("2 = Empty-gate background calibration")
    print("    Use this now after final sensor installation")
    print()
    print("Q = Quit")
    print("============================================================")

    choice = input("Select calibration: ").strip().lower()

    if choice == "q":
        print("Calibration cancelled.")
        return

    if choice not in ("1", "2"):
        print("Invalid selection.")
        return

    pair = PresenceSensorPair()
    pair.initialize()

    if choice == "1":
        run_range_calibration(pair)
    else:
        run_background_calibration(pair)


main()
