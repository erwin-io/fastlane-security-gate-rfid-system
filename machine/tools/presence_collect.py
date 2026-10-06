# ============================================================
# FASTLANE PORTABLE PRESENCE / DIRECTION OBSERVER V4
# File: /tools/presence_collect.py
# ============================================================
#
# PURPOSE
# ------------------------------------------------------------
# Fast, background-distance-independent presence / direction test
# for two VL53L0X sensors with <= 100 mm (10 cm) spacing.
#
# THIS VERSION FIXES TWO PROBLEMS:
#
# 1) FALSE / SLOW CLEARING
#    The previous auto reference could initialize too high because
#    it used a high percentile. Example: a clear sensor could learn
#    ~1500 mm even though most normal clear readings were ~1200 mm.
#    That made the sensor remain ACTIVE for a very long time.
#
#    V4 uses a ROBUST MEDIAN clear reference and capped adaptation.
#
# 2) FALSE PEEK DURING NORMAL EXIT
#    PEEK_BACK is now based on RETURNING TO THE SAME SIDE WHERE THE
#    EPISODE STARTED.
#
#    Start S1:
#       S1 -> BOTH/S2 -> S1 = PEEK_BACK_CANDIDATE
#
#    Start S2:
#       S2 -> BOTH/S1 -> S2 = PEEK_BACK_CANDIDATE
#
#    Normal EXIT:
#       S2 -> BOTH -> S1 -> IDLE = EXITING_CANDIDATE
#
#    Normal ENTER:
#       S1 -> BOTH -> S2 -> IDLE = ENTERING_CANDIDATE
#
# FAST DECISION
# ------------------------------------------------------------
# - ACTIVE confirmation: 30 ms
# - CLEAR confirmation : 120 ms
# - TRUE IDLE          : 500 ms continuously clear
#
# So a completed passage normally finalizes about half a second
# after both sensors are confirmed clear.
#
# STAYING
# ------------------------------------------------------------
# Both sensors must remain continuously ACTIVE for >= 1500 ms.
# This is only a clue and NEVER overrides a valid terminal direction.
#
# SENSOR GEOMETRY
# ------------------------------------------------------------
# - Both sensors approximately 90 degrees across the passage.
# - Parallel beams.
# - Sensors separated along walking direction.
# - MAXIMUM gap = 100 mm / 10 cm.
#
# NO FIXED BACKGROUND CONFIGURATION IS USED.
#
# Run only while main.py is stopped.
# ============================================================

import time

try:
    import ujson as json
except ImportError:
    import json

from tools.presence_sensor_pair import PresenceSensorPair


# ============================================================
# FILES
# ============================================================

CONFIG_FILE = "/config.json"
RANGE_CALIBRATION_FILE = "/presence_range_calibration.json"


# ============================================================
# GEOMETRY
# ============================================================

DEFAULT_SENSOR_GAP_MM = 100
MIN_SENSOR_GAP_MM = 10
MAX_SENSOR_GAP_MM = 100


# ============================================================
# VALID RANGE
# ============================================================

DEFAULT_MIN_VALID_MM = 50
DEFAULT_MAX_VALID_MM = 3000


# ============================================================
# PRESENCE THRESHOLDS
# ============================================================

# Relative DROP from the automatically learned clear reference.
DEFAULT_ENTER_DROP_MM = 100
DEFAULT_EXIT_DROP_MM = 50

DEFAULT_FILTER_SIZE = 3


# ============================================================
# AUTO CLEAR REFERENCE
# ============================================================

# Median is intentionally used instead of a high percentile.
REFERENCE_INIT_SAMPLES = 15
REFERENCE_WINDOW_SIZE = 21

# Reference adapts only while sensor is CLEAR.
REFERENCE_ALPHA = 0.10

# Never move the reference too much in one update.
REFERENCE_MAX_STEP_MM = 18.0

# A closer reading can update the CLEAR reference only when it is
# very close to the current reference. This prevents a person from
# dragging the baseline downward before ACTIVE confirmation.
REFERENCE_CLOSER_UPDATE_MAX_DROP_MM = 35.0


# ============================================================
# TIMING
# ============================================================

ACTIVE_CONFIRM_MS = 30
CLEAR_CONFIRM_MS = 120

# 10 cm spacing naturally creates overlap. Require a meaningful
# continuous BOTH interval before marking STAYING.
STAYING_CONFIRM_MS = 1500

# Fast final decision after both sensors are truly clear.
IDLE_CONFIRM_MS = 500

STATUS_INTERVAL_MS = 250
LOOP_DELAY_MS = 5


# ============================================================
# HELPERS
# ============================================================

def _load_json(path):
    try:
        with open(path, "r") as file:
            data = json.loads(
                file.read()
            )

        if isinstance(data, dict):
            return data

    except Exception:
        pass

    return {}


def _clamp_int(
    value,
    minimum,
    maximum,
    default_value,
):
    try:
        value = int(value)
    except Exception:
        value = int(default_value)

    if value < minimum:
        value = minimum

    if value > maximum:
        value = maximum

    return value


def _median(values):
    values = sorted(
        float(value)
        for value in values
        if value is not None
    )

    if not values:
        return None

    count = len(values)
    middle = count // 2

    if count % 2:
        return values[middle]

    return (
        values[middle - 1]
        +
        values[middle]
    ) / 2.0


def _fmt(value):
    if value is None:
        return "NA"

    return "{:.1f}".format(
        float(value)
    )


def _sensor_name(sensor_no):
    if sensor_no == 1:
        return "S1"

    if sensor_no == 2:
        return "S2"

    return "NA"


def _mask_name(mask):
    if mask == 0:
        return "NONE"

    if mask == 1:
        return "S1_ONLY"

    if mask == 2:
        return "S2_ONLY"

    if mask == 3:
        return "S1+S2"

    return "UNKNOWN"


def _sequence_text(sequence):
    if not sequence:
        return "NONE"

    return ">".join(
        _sensor_name(sensor_no)
        for sensor_no in sequence
    )


# ============================================================
# SETTINGS
# ============================================================

def _load_settings():
    config = _load_json(
        CONFIG_FILE
    )

    tof = config.get(
        "tof",
        {},
    )

    if not isinstance(tof, dict):
        tof = {}

    calibration = _load_json(
        RANGE_CALIBRATION_FILE
    )

    settings = {
        "sensor_gap_mm": _clamp_int(
            tof.get(
                "sensor_gap_mm",
                DEFAULT_SENSOR_GAP_MM,
            ),
            MIN_SENSOR_GAP_MM,
            MAX_SENSOR_GAP_MM,
            DEFAULT_SENSOR_GAP_MM,
        ),

        "sensor1_offset_mm": float(
            calibration.get(
                "sensor1_offset_mm",
                0.0,
            )
        ),

        "sensor2_offset_mm": float(
            calibration.get(
                "sensor2_offset_mm",
                0.0,
            )
        ),

        "min_valid_mm": max(
            DEFAULT_MIN_VALID_MM,
            _clamp_int(
                tof.get(
                    "presence_min_valid_mm",
                    DEFAULT_MIN_VALID_MM,
                ),
                DEFAULT_MIN_VALID_MM,
                1000,
                DEFAULT_MIN_VALID_MM,
            ),
        ),

        "max_valid_mm": _clamp_int(
            tof.get(
                "presence_max_valid_mm",
                DEFAULT_MAX_VALID_MM,
            ),
            200,
            4000,
            DEFAULT_MAX_VALID_MM,
        ),

        "enter_drop_mm": _clamp_int(
            tof.get(
                "presence_enter_drop_mm",
                DEFAULT_ENTER_DROP_MM,
            ),
            20,
            1000,
            DEFAULT_ENTER_DROP_MM,
        ),

        "exit_drop_mm": _clamp_int(
            tof.get(
                "presence_exit_drop_mm",
                DEFAULT_EXIT_DROP_MM,
            ),
            10,
            900,
            DEFAULT_EXIT_DROP_MM,
        ),

        "filter_size": _clamp_int(
            tof.get(
                "presence_filter_size",
                DEFAULT_FILTER_SIZE,
            ),
            1,
            7,
            DEFAULT_FILTER_SIZE,
        ),
    }

    if (
        settings["exit_drop_mm"]
        >=
        settings["enter_drop_mm"]
    ):
        settings["exit_drop_mm"] = max(
            10,
            settings["enter_drop_mm"] // 2,
        )

    return settings


# ============================================================
# SENSOR CHANNEL
# ============================================================

class AutoPresenceChannel:
    def __init__(
        self,
        sensor_no,
        offset_mm,
        settings,
    ):
        self.sensor_no = int(
            sensor_no
        )

        self.offset_mm = float(
            offset_mm
        )

        self.min_valid_mm = int(
            settings[
                "min_valid_mm"
            ]
        )

        self.max_valid_mm = int(
            settings[
                "max_valid_mm"
            ]
        )

        self.enter_drop_mm = int(
            settings[
                "enter_drop_mm"
            ]
        )

        self.exit_drop_mm = int(
            settings[
                "exit_drop_mm"
            ]
        )

        self.filter_size = int(
            settings[
                "filter_size"
            ]
        )

        self.raw_mm = None
        self.corrected_mm = None
        self.filtered_mm = None

        self.reference_mm = None
        self.drop_mm = 0.0

        self.measurements = []
        self.reference_samples = []

        self.active = False

        self.active_candidate_since = None
        self.clear_candidate_since = None

        self.rejected_near = 0
        self.rejected_far = 0

    # --------------------------------------------------------
    # TEXT
    # --------------------------------------------------------

    def state_text(self):
        if self.reference_mm is None:
            return "LEARNING"

        if self.active:
            return "ACTIVE"

        return "CLEAR"

    # --------------------------------------------------------
    # VALIDITY
    # --------------------------------------------------------

    def _valid_raw(
        self,
        raw_mm,
    ):
        if raw_mm is None:
            return False

        try:
            value = float(
                raw_mm
            )
        except Exception:
            return False

        if value < self.min_valid_mm:
            self.rejected_near += 1
            return False

        if value > self.max_valid_mm:
            self.rejected_far += 1
            return False

        return True

    # --------------------------------------------------------
    # REFERENCE WINDOW
    # --------------------------------------------------------

    def _append_reference_sample(
        self,
        value,
    ):
        self.reference_samples.append(
            float(value)
        )

        while (
            len(self.reference_samples)
            >
            REFERENCE_WINDOW_SIZE
        ):
            self.reference_samples.pop(0)

    def _try_initialize_reference(
        self,
    ):
        if self.reference_mm is not None:
            return True

        if (
            len(self.reference_samples)
            <
            REFERENCE_INIT_SAMPLES
        ):
            return False

        # ROBUST MEDIAN:
        # isolated long/short returns cannot dominate initialization.
        self.reference_mm = _median(
            self.reference_samples
        )

        return (
            self.reference_mm
            is not None
        )

    def _adapt_reference(
        self,
        clear_value,
    ):
        clear_value = float(
            clear_value
        )

        if self.reference_mm is None:
            self._append_reference_sample(
                clear_value
            )

            self._try_initialize_reference()

            return

        difference = (
            clear_value
            -
            self.reference_mm
        )

        # Farther readings are safe to learn slowly because a person
        # normally makes the measured distance shorter, not farther.
        if difference >= 0:
            self._append_reference_sample(
                clear_value
            )

        else:
            # A closer reading is allowed to affect the baseline only
            # when it is still very close to the current CLEAR level.
            if (
                abs(
                    difference
                )
                <=
                REFERENCE_CLOSER_UPDATE_MAX_DROP_MM
            ):
                self._append_reference_sample(
                    clear_value
                )
            else:
                return

        target = _median(
            self.reference_samples
        )

        if target is None:
            return

        delta = (
            target
            -
            self.reference_mm
        )

        step = (
            delta
            *
            REFERENCE_ALPHA
        )

        if (
            step
            >
            REFERENCE_MAX_STEP_MM
        ):
            step = (
                REFERENCE_MAX_STEP_MM
            )

        elif (
            step
            <
            -REFERENCE_MAX_STEP_MM
        ):
            step = (
                -REFERENCE_MAX_STEP_MM
            )

        self.reference_mm += (
            step
        )

    # --------------------------------------------------------
    # UPDATE
    # --------------------------------------------------------

    def update(
        self,
        raw_mm,
        now_ms,
    ):
        previous_active = (
            self.active
        )

        event = {
            "valid": False,

            "changed": False,

            "rising": False,
            "falling": False,

            # Transition-start timestamp.
            "rising_ms": None,
            "falling_ms": None,
        }

        # Invalid readings never enter filtering/state logic.
        if not self._valid_raw(
            raw_mm
        ):
            return event

        self.raw_mm = float(
            raw_mm
        )

        self.corrected_mm = (
            self.raw_mm
            +
            self.offset_mm
        )

        if self.corrected_mm <= 0:
            return event

        event["valid"] = True

        self.measurements.append(
            self.corrected_mm
        )

        while (
            len(self.measurements)
            >
            self.filter_size
        ):
            self.measurements.pop(0)

        self.filtered_mm = _median(
            self.measurements
        )

        # ----------------------------------------------------
        # INITIAL CLEAR REFERENCE
        # ----------------------------------------------------

        if self.reference_mm is None:
            self._append_reference_sample(
                self.filtered_mm
            )

            self._try_initialize_reference()

            self.drop_mm = 0.0

            return event

        # ----------------------------------------------------
        # RELATIVE DROP
        # ----------------------------------------------------

        self.drop_mm = max(
            0.0,
            self.reference_mm
            -
            self.filtered_mm,
        )

        # ----------------------------------------------------
        # ACTIVE -> CLEAR
        # ----------------------------------------------------

        if self.active:
            if (
                self.drop_mm
                <=
                self.exit_drop_mm
            ):
                if (
                    self.clear_candidate_since
                    is None
                ):
                    self.clear_candidate_since = (
                        now_ms
                    )

                elif (
                    time.ticks_diff(
                        now_ms,
                        self.clear_candidate_since,
                    )
                    >=
                    CLEAR_CONFIRM_MS
                ):
                    falling_ms = (
                        self.clear_candidate_since
                    )

                    self.active = False

                    self.active_candidate_since = None
                    self.clear_candidate_since = None

                    # Start fresh after the person clears this beam.
                    self.measurements = [
                        self.filtered_mm
                    ]

                    self._adapt_reference(
                        self.filtered_mm
                    )

                    event["falling_ms"] = (
                        falling_ms
                    )

            else:
                self.clear_candidate_since = None

        # ----------------------------------------------------
        # CLEAR -> ACTIVE
        # ----------------------------------------------------

        else:
            if (
                self.drop_mm
                >=
                self.enter_drop_mm
            ):
                if (
                    self.active_candidate_since
                    is None
                ):
                    self.active_candidate_since = (
                        now_ms
                    )

                elif (
                    time.ticks_diff(
                        now_ms,
                        self.active_candidate_since,
                    )
                    >=
                    ACTIVE_CONFIRM_MS
                ):
                    rising_ms = (
                        self.active_candidate_since
                    )

                    self.active = True

                    self.active_candidate_since = None
                    self.clear_candidate_since = None

                    event["rising_ms"] = (
                        rising_ms
                    )

            else:
                self.active_candidate_since = None
                self.clear_candidate_since = None

                self._adapt_reference(
                    self.filtered_mm
                )

        changed = (
            previous_active
            !=
            self.active
        )

        event["changed"] = (
            changed
        )

        event["rising"] = (
            changed
            and
            self.active
        )

        event["falling"] = (
            changed
            and
            not self.active
        )

        return event


# ============================================================
# PASSAGE TRACKER
# ============================================================

class PassageTracker:
    def __init__(
        self,
        sensor_gap_mm,
    ):
        self.sensor_gap_mm = float(
            sensor_gap_mm
        )

        self.episode_number = 0

        self.reset()

    # --------------------------------------------------------
    # RESET
    # --------------------------------------------------------

    def reset(self):
        self.episode_active = False

        self.started_ms = None
        self.last_update_ms = None

        self.clear_since_ms = None

        self.first_sensor = 0
        self.first_sensor_ms = None

        self.last_sensor = 0
        self.last_sensor_ms = None

        self.current_mask = 0
        self.seen_mask = 0

        self.initial_mask = 0
        self.last_nonzero_mask = 0

        self.mask_history = []
        self.sequence = []
        self.events = []

        self.entering_seen = False
        self.exiting_seen = False

        self.both_since_ms = None
        self.staying_seen = False

        # Symmetric direction-aware state.
        self.reached_opposite_side = False
        self.returned_to_start_side = False

        self.peek_pending = False

        self.first_transfer_ms = None
        self.first_transfer_direction = "NA"
        self.speed_proxy_mps = None

        self.s1_active_ms = 0
        self.s2_active_ms = 0
        self.both_active_ms = 0

        self.s1_rises = 0
        self.s2_rises = 0

    # --------------------------------------------------------
    # START SIDE
    # --------------------------------------------------------

    def _start_side(self):
        if self.first_sensor in (1, 2):
            return self.first_sensor

        if self.initial_mask == 1:
            return 1

        if self.initial_mask == 2:
            return 2

        return 0

    def _start_mask(self):
        side = self._start_side()

        if side == 1:
            return 1

        if side == 2:
            return 2

        return 0

    def _opposite_mask(self):
        side = self._start_side()

        if side == 1:
            return 2

        if side == 2:
            return 1

        return 0

    # --------------------------------------------------------
    # START
    # --------------------------------------------------------

    def _start(
        self,
        now_ms,
        initial_mask,
    ):
        self.episode_number += 1

        self.episode_active = True

        self.started_ms = (
            now_ms
        )

        self.last_update_ms = (
            now_ms
        )

        self.initial_mask = int(
            initial_mask
        )

        self.current_mask = int(
            initial_mask
        )

        self.last_nonzero_mask = int(
            initial_mask
        )

        self.seen_mask |= int(
            initial_mask
        )

        self.mask_history.append(
            (
                int(
                    initial_mask
                ),
                0,
            )
        )

        if initial_mask == 3:
            self.both_since_ms = (
                now_ms
            )

        print()
        print(
            ">>> EPISODE #{} START | {}".format(
                self.episode_number,
                _mask_name(
                    initial_mask
                ),
            )
        )

    # --------------------------------------------------------
    # EDGES
    # --------------------------------------------------------

    def _rise(
        self,
        sensor_no,
        event_ms,
    ):
        if event_ms is None:
            event_ms = (
                time.ticks_ms()
            )

        if self.first_sensor == 0:
            self.first_sensor = int(
                sensor_no
            )

            self.first_sensor_ms = (
                event_ms
            )

        self.last_sensor = int(
            sensor_no
        )

        self.last_sensor_ms = (
            event_ms
        )

        if sensor_no == 1:
            self.s1_rises += 1
            self.seen_mask |= 1

        else:
            self.s2_rises += 1
            self.seen_mask |= 2

        if (
            not self.sequence
            or
            self.sequence[-1]
            !=
            sensor_no
        ):
            self.sequence.append(
                int(
                    sensor_no
                )
            )

        self.events.append(
            (
                "S{}_ON".format(
                    sensor_no
                ),
                max(
                    0,
                    time.ticks_diff(
                        event_ms,
                        self.started_ms,
                    ),
                ),
            )
        )

        # First cross-sensor timing.
        if (
            self.first_transfer_ms
            is None
            and
            self.first_sensor
            != 0
            and
            sensor_no
            !=
            self.first_sensor
            and
            self.first_sensor_ms
            is not None
        ):
            transfer_ms = time.ticks_diff(
                event_ms,
                self.first_sensor_ms,
            )

            if transfer_ms > 0:
                self.first_transfer_ms = (
                    transfer_ms
                )

                if (
                    self.first_sensor == 1
                    and
                    sensor_no == 2
                ):
                    self.first_transfer_direction = (
                        "S1_TO_S2"
                    )

                else:
                    self.first_transfer_direction = (
                        "S2_TO_S1"
                    )

                self.speed_proxy_mps = (
                    self.sensor_gap_mm
                    /
                    float(
                        transfer_ms
                    )
                )

    def _fall(
        self,
        sensor_no,
        event_ms,
    ):
        if event_ms is None:
            event_ms = (
                time.ticks_ms()
            )

        self.events.append(
            (
                "S{}_OFF".format(
                    sensor_no
                ),
                max(
                    0,
                    time.ticks_diff(
                        event_ms,
                        self.started_ms,
                    ),
                ),
            )
        )

    # --------------------------------------------------------
    # MASK TRANSITIONS
    # --------------------------------------------------------

    def _record_mask_transition(
        self,
        old_mask,
        new_mask,
        now_ms,
    ):
        if new_mask == old_mask:
            return

        if new_mask != 0:
            if self.initial_mask == 0:
                self.initial_mask = (
                    new_mask
                )

            self.last_nonzero_mask = (
                new_mask
            )

            self.seen_mask |= (
                new_mask
            )

            self.mask_history.append(
                (
                    int(
                        new_mask
                    ),
                    max(
                        0,
                        time.ticks_diff(
                            now_ms,
                            self.started_ms,
                        ),
                    ),
                )
            )

        start_side = (
            self._start_side()
        )

        start_mask = (
            self._start_mask()
        )

        opposite_mask = (
            self._opposite_mask()
        )

        # ----------------------------------------------------
        # REACHED OPPOSITE DIRECTION
        # ----------------------------------------------------

        if start_side == 1:
            if new_mask in (2, 3):
                self.entering_seen = True
                self.reached_opposite_side = True

        elif start_side == 2:
            if new_mask in (1, 3):
                self.exiting_seen = True
                self.reached_opposite_side = True

        # ----------------------------------------------------
        # BOTH TIMER
        # ----------------------------------------------------

        if new_mask == 3:
            if self.both_since_ms is None:
                self.both_since_ms = (
                    now_ms
                )

        else:
            self.both_since_ms = None

        # ----------------------------------------------------
        # RETURN TO SAME START SIDE = PEEK/BACKOUT
        # ----------------------------------------------------

        if (
            self.reached_opposite_side
            and
            start_mask != 0
            and
            new_mask == start_mask
        ):
            self.returned_to_start_side = True
            self.peek_pending = True

        # ----------------------------------------------------
        # IF PERSON CONTINUES TO OPPOSITE TERMINAL SIDE,
        # CANCEL THE PREVIOUS BACKOUT CANDIDATE.
        # ----------------------------------------------------

        if (
            self.peek_pending
            and
            opposite_mask != 0
            and
            new_mask == opposite_mask
        ):
            self.peek_pending = False
            self.returned_to_start_side = False

    # --------------------------------------------------------
    # STAYING
    # --------------------------------------------------------

    def _update_staying(
        self,
        now_ms,
    ):
        if (
            self.current_mask == 3
            and
            self.both_since_ms
            is not None
        ):
            both_ms = max(
                0,
                time.ticks_diff(
                    now_ms,
                    self.both_since_ms,
                ),
            )

            if (
                both_ms
                >=
                STAYING_CONFIRM_MS
            ):
                self.staying_seen = True

    # --------------------------------------------------------
    # PEEK RULE
    # --------------------------------------------------------

    def _peek_rule(
        self,
        final=False,
    ):
        reasons = []

        start_side = (
            self._start_side()
        )

        start_mask = (
            self._start_mask()
        )

        if start_side not in (1, 2):
            return (
                False,
                reasons,
            )

        # S1_ONLY -> IDLE or S2_ONLY -> IDLE
        if (
            final
            and
            self.seen_mask == start_mask
            and
            self.last_nonzero_mask == start_mask
        ):
            reasons.append(
                "{}_ONLY -> IDLE without reaching opposite sensor".format(
                    _sensor_name(
                        start_side
                    )
                )
            )

        # Main return-to-start rule.
        if (
            self.returned_to_start_side
            and
            self.last_nonzero_mask == start_mask
        ):
            if self.staying_seen:
                reasons.append(
                    "started {} -> reached opposite side -> stayed -> returned to {}".format(
                        _sensor_name(
                            start_side
                        ),
                        _sensor_name(
                            start_side
                        ),
                    )
                )

            else:
                reasons.append(
                    "started {} -> reached opposite side -> returned to {}".format(
                        _sensor_name(
                            start_side
                        ),
                        _sensor_name(
                            start_side
                        ),
                    )
                )

        return (
            len(
                reasons
            )
            >
            0,
            reasons,
        )

    # --------------------------------------------------------
    # LIVE HINT
    # --------------------------------------------------------

    def live_hint(
        self,
        now_ms,
    ):
        if not self.episode_active:
            return "IDLE"

        self._update_staying(
            now_ms
        )

        start_side = (
            self._start_side()
        )

        start_mask = (
            self._start_mask()
        )

        opposite_mask = (
            self._opposite_mask()
        )

        is_peek, _ = (
            self._peek_rule(
                final=False
            )
        )

        # True return to starting side has highest priority.
        if is_peek:
            return "PEEK_BACK_CANDIDATE"

        # ----------------------------------------------------
        # BOTH CLEAR, WAITING BRIEFLY FOR TRUE IDLE
        # ----------------------------------------------------

        if self.current_mask == 0:
            clear_ms = 0

            if (
                self.clear_since_ms
                is not None
            ):
                clear_ms = max(
                    0,
                    time.ticks_diff(
                        now_ms,
                        self.clear_since_ms,
                    ),
                )

            if (
                self.returned_to_start_side
                and
                self.last_nonzero_mask
                ==
                start_mask
            ):
                return (
                    "PEEK_BACK_PENDING_IDLE {}/{}ms".format(
                        clear_ms,
                        IDLE_CONFIRM_MS,
                    )
                )

            if (
                start_side == 1
                and
                self.entering_seen
                and
                self.last_nonzero_mask
                ==
                opposite_mask
            ):
                return (
                    "ENTERING_PENDING_IDLE {}/{}ms".format(
                        clear_ms,
                        IDLE_CONFIRM_MS,
                    )
                )

            if (
                start_side == 2
                and
                self.exiting_seen
                and
                self.last_nonzero_mask
                ==
                opposite_mask
            ):
                return (
                    "EXITING_PENDING_IDLE {}/{}ms".format(
                        clear_ms,
                        IDLE_CONFIRM_MS,
                    )
                )

            return (
                "CLEAR_WAIT {}/{}ms".format(
                    clear_ms,
                    IDLE_CONFIRM_MS,
                )
            )

        # ----------------------------------------------------
        # TERMINAL SINGLE-SENSOR STATES HAVE PRIORITY OVER STAYING
        # ----------------------------------------------------

        if (
            start_side == 1
            and
            self.current_mask == 2
        ):
            return "ENTERING_CANDIDATE"

        if (
            start_side == 2
            and
            self.current_mask == 1
        ):
            return "EXITING_CANDIDATE"

        # ----------------------------------------------------
        # STAYING ONLY WHILE BOTH ARE CURRENTLY ACTIVE
        # ----------------------------------------------------

        if (
            self.current_mask == 3
            and
            self.staying_seen
        ):
            return "STAYING_CANDIDATE"

        # ----------------------------------------------------
        # NORMAL IN-PROGRESS DIRECTION
        # ----------------------------------------------------

        if (
            start_side == 1
            and
            self.entering_seen
        ):
            return "ENTERING_CANDIDATE"

        if (
            start_side == 2
            and
            self.exiting_seen
        ):
            return "EXITING_CANDIDATE"

        if self.current_mask == 1:
            return "S1_ONLY"

        if self.current_mask == 2:
            return "S2_ONLY"

        if self.current_mask == 3:
            return "S1+S2"

        return "OBSERVING"

    # --------------------------------------------------------
    # UPDATE
    # --------------------------------------------------------

    def update(
        self,
        s1_active,
        s2_active,
        event1,
        event2,
        now_ms,
    ):
        new_mask = (
            (1 if s1_active else 0)
            |
            (2 if s2_active else 0)
        )

        if (
            not self.episode_active
            and
            new_mask != 0
        ):
            self._start(
                now_ms,
                new_mask,
            )

        if not self.episode_active:
            self.current_mask = 0
            return

        # ----------------------------------------------------
        # DWELL TIMES
        # ----------------------------------------------------

        if (
            self.last_update_ms
            is not None
        ):
            delta_ms = max(
                0,
                time.ticks_diff(
                    now_ms,
                    self.last_update_ms,
                ),
            )

            if self.current_mask & 1:
                self.s1_active_ms += (
                    delta_ms
                )

            if self.current_mask & 2:
                self.s2_active_ms += (
                    delta_ms
                )

            if self.current_mask == 3:
                self.both_active_ms += (
                    delta_ms
                )

        self.last_update_ms = (
            now_ms
        )

        # ----------------------------------------------------
        # RISING EDGES
        # ----------------------------------------------------

        # If both confirm in the same loop, keep the earlier
        # transition-start timestamp as Q1.
        if (
            event1["rising"]
            and
            event2["rising"]
        ):
            s1_ms = (
                event1[
                    "rising_ms"
                ]
            )

            s2_ms = (
                event2[
                    "rising_ms"
                ]
            )

            if (
                s1_ms is not None
                and
                s2_ms is not None
                and
                time.ticks_diff(
                    s1_ms,
                    s2_ms,
                )
                >
                0
            ):
                self._rise(
                    2,
                    s2_ms,
                )

                self._rise(
                    1,
                    s1_ms,
                )

            else:
                self._rise(
                    1,
                    s1_ms,
                )

                self._rise(
                    2,
                    s2_ms,
                )

        else:
            if event1["rising"]:
                self._rise(
                    1,
                    event1[
                        "rising_ms"
                    ],
                )

            if event2["rising"]:
                self._rise(
                    2,
                    event2[
                        "rising_ms"
                    ],
                )

        # ----------------------------------------------------
        # FALLING EDGES
        # ----------------------------------------------------

        if event1["falling"]:
            self._fall(
                1,
                event1[
                    "falling_ms"
                ],
            )

        if event2["falling"]:
            self._fall(
                2,
                event2[
                    "falling_ms"
                ],
            )

        # ----------------------------------------------------
        # MASK
        # ----------------------------------------------------

        old_mask = (
            self.current_mask
        )

        self._record_mask_transition(
            old_mask,
            new_mask,
            now_ms,
        )

        self.current_mask = (
            new_mask
        )

        self.seen_mask |= (
            new_mask
        )

        self._update_staying(
            now_ms
        )

        # ----------------------------------------------------
        # TRUE IDLE
        # ----------------------------------------------------

        if new_mask == 0:
            if (
                self.clear_since_ms
                is None
            ):
                self.clear_since_ms = (
                    now_ms
                )

            elif (
                time.ticks_diff(
                    now_ms,
                    self.clear_since_ms,
                )
                >=
                IDLE_CONFIRM_MS
            ):
                self._finish(
                    now_ms
                )

        else:
            self.clear_since_ms = None

    # --------------------------------------------------------
    # FINAL RESULT
    # --------------------------------------------------------

    def _final_result(
        self,
    ):
        is_peek, reasons = (
            self._peek_rule(
                final=True
            )
        )

        if is_peek:
            return (
                "PEEK_BACK_CANDIDATE",
                reasons,
            )

        start_side = (
            self._start_side()
        )

        opposite_mask = (
            self._opposite_mask()
        )

        # Normal ENTER.
        if (
            start_side == 1
            and
            self.entering_seen
            and
            self.last_nonzero_mask
            ==
            opposite_mask
        ):
            return (
                "ENTERING_CANDIDATE",
                [
                    "S1 start -> finished on S2 side"
                ],
            )

        # Normal EXIT.
        if (
            start_side == 2
            and
            self.exiting_seen
            and
            self.last_nonzero_mask
            ==
            opposite_mask
        ):
            return (
                "EXITING_CANDIDATE",
                [
                    "S2 start -> finished on S1 side"
                ],
            )

        if self.staying_seen:
            return (
                "STAYING_CANDIDATE",
                [
                    "continuous BOTH >= {} ms without a terminal completion".format(
                        STAYING_CONFIRM_MS
                    )
                ],
            )

        return (
            "UNKNOWN_CANDIDATE",
            [
                "no complete direction / return pattern"
            ],
        )

    # --------------------------------------------------------
    # FINISH
    # --------------------------------------------------------

    def _finish(
        self,
        now_ms,
    ):
        duration_ms = max(
            0,
            time.ticks_diff(
                now_ms,
                self.started_ms,
            ),
        )

        result, reasons = (
            self._final_result()
        )

        print()
        print(
            "============================================================"
        )

        print(
            "EPISODE #{} COMPLETE".format(
                self.episode_number
            )
        )

        print(
            "============================================================"
        )

        print(
            "Duration              : {:.2f} sec".format(
                duration_ms
                /
                1000.0
            )
        )

        print(
            "Sensor gap            : {:.0f} mm ({:.1f} cm)".format(
                self.sensor_gap_mm,
                self.sensor_gap_mm
                /
                10.0,
            )
        )

        print(
            "Initial state         : {}".format(
                _mask_name(
                    self.initial_mask
                )
            )
        )

        print(
            "Start side (Q1)       : {}".format(
                _sensor_name(
                    self._start_side()
                )
            )
        )

        print(
            "Last active state     : {}".format(
                _mask_name(
                    self.last_nonzero_mask
                )
            )
        )

        print(
            "Final state           : IDLE"
        )

        print(
            "Q1 first              : {}".format(
                _sensor_name(
                    self.first_sensor
                )
            )
        )

        print(
            "Q2 latest             : {}".format(
                _sensor_name(
                    self.last_sensor
                )
            )
        )

        print(
            "Q3 current            : NONE"
        )

        print(
            "Q4 seen               : {}".format(
                _mask_name(
                    self.seen_mask
                )
            )
        )

        print(
            "Confirmed rise seq    : {}".format(
                _sequence_text(
                    self.sequence
                )
            )
        )

        print(
            "Stable state history  : {}".format(
                " -> ".join(
                    "{}@{}ms".format(
                        _mask_name(
                            mask
                        ),
                        offset_ms,
                    )
                    for mask, offset_ms
                    in self.mask_history
                )
                if self.mask_history
                else "NONE"
            )
        )

        print(
            "Entering seen         : {}".format(
                self.entering_seen
            )
        )

        print(
            "Exiting seen          : {}".format(
                self.exiting_seen
            )
        )

        print(
            "Staying seen          : {}".format(
                self.staying_seen
            )
        )

        print(
            "Reached opposite side : {}".format(
                self.reached_opposite_side
            )
        )

        print(
            "Returned to start side: {}".format(
                self.returned_to_start_side
            )
        )

        print(
            "S1 active time        : {:.2f} sec".format(
                self.s1_active_ms
                /
                1000.0
            )
        )

        print(
            "S2 active time        : {:.2f} sec".format(
                self.s2_active_ms
                /
                1000.0
            )
        )

        print(
            "Both active overlap   : {:.2f} sec".format(
                self.both_active_ms
                /
                1000.0
            )
        )

        print(
            "First transfer        : {}".format(
                self.first_transfer_direction
            )
        )

        if (
            self.first_transfer_ms
            is None
        ):
            print(
                "First transfer time   : NA"
            )

            print(
                "Speed proxy           : NA"
            )

        else:
            print(
                "First transfer time   : {} ms".format(
                    self.first_transfer_ms
                )
            )

            print(
                "Speed proxy           : {:.3f} m/s".format(
                    self.speed_proxy_mps
                )
            )

        print()

        print(
            "RULE RESULT           : {}".format(
                result
            )
        )

        for reason in reasons:
            print(
                "REASON                : {}".format(
                    reason
                )
            )

        print(
            "ML CLASSIFICATION     : NOT RUN YET"
        )

        print(
            "============================================================"
        )
        print()

        self.reset()

    # --------------------------------------------------------
    # LIVE
    # --------------------------------------------------------

    def live_text(
        self,
        now_ms,
    ):
        return (
            "HINT={} | "
            "Q1={} Q2={} Q3={} Q4={} | "
            "SEQ={} | STAY={} | OPP={} | BACK_START={}".format(
                self.live_hint(
                    now_ms
                ),

                _sensor_name(
                    self.first_sensor
                ),

                _sensor_name(
                    self.last_sensor
                ),

                _mask_name(
                    self.current_mask
                ),

                _mask_name(
                    self.seen_mask
                ),

                _sequence_text(
                    self.sequence
                ),

                self.staying_seen,

                self.reached_opposite_side,

                self.returned_to_start_side,
            )
        )


# ============================================================
# DISPLAY
# ============================================================

def _channel_text(
    channel,
):
    return (
        "S{} raw={} corr={} filt={} ref={} drop={} {}".format(
            channel.sensor_no,

            _fmt(
                channel.raw_mm
            ),

            _fmt(
                channel.corrected_mm
            ),

            _fmt(
                channel.filtered_mm
            ),

            _fmt(
                channel.reference_mm
            ),

            _fmt(
                channel.drop_mm
            ),

            channel.state_text(),
        )
    )


# ============================================================
# MAIN
# ============================================================

def main():
    print()
    print(
        "============================================================"
    )

    print(
        "FASTLANE PORTABLE PRESENCE / DIRECTION OBSERVER V4"
    )

    print(
        "============================================================"
    )

    print(
        "Both sensors: ~90 degrees and parallel"
    )

    print(
        "Maximum sensor gap: 100 mm / 10 cm"
    )

    print(
        "No fixed background distance"
    )

    print(
        "Clear reference: ROBUST MEDIAN, not high percentile"
    )

    print(
        "PEEK: reach opposite side then return to SAME start side"
    )

    print(
        "TRUE IDLE: both sensors clear for {} ms".format(
            IDLE_CONFIRM_MS
        )
    )

    print(
        "============================================================"
    )

    settings = (
        _load_settings()
    )

    print()
    print(
        "Sensor gap: {} mm ({:.1f} cm)".format(
            settings[
                "sensor_gap_mm"
            ],

            settings[
                "sensor_gap_mm"
            ]
            /
            10.0,
        )
    )

    print(
        "S1 range offset: {:+.3f} mm".format(
            settings[
                "sensor1_offset_mm"
            ]
        )
    )

    print(
        "S2 range offset: {:+.3f} mm".format(
            settings[
                "sensor2_offset_mm"
            ]
        )
    )

    print(
        "Valid raw range: {}..{} mm".format(
            settings[
                "min_valid_mm"
            ],

            settings[
                "max_valid_mm"
            ],
        )
    )

    print(
        "Enter drop: {} mm".format(
            settings[
                "enter_drop_mm"
            ]
        )
    )

    print(
        "Exit drop : {} mm".format(
            settings[
                "exit_drop_mm"
            ]
        )
    )

    print(
        "ACTIVE confirm: {} ms".format(
            ACTIVE_CONFIRM_MS
        )
    )

    print(
        "CLEAR confirm : {} ms".format(
            CLEAR_CONFIRM_MS
        )
    )

    print(
        "IDLE confirm  : {} ms".format(
            IDLE_CONFIRM_MS
        )
    )

    print(
        "STAYING       : {} ms continuous BOTH".format(
            STAYING_CONFIRM_MS
        )
    )

    print()
    print(
        "IMPORTANT: keep the lane CLEAR during initial reference learning."
    )

    pair = (
        PresenceSensorPair()
    )

    pair.initialize()

    sensor1 = AutoPresenceChannel(
        1,

        settings[
            "sensor1_offset_mm"
        ],

        settings,
    )

    sensor2 = AutoPresenceChannel(
        2,

        settings[
            "sensor2_offset_mm"
        ],

        settings,
    )

    tracker = PassageTracker(
        settings[
            "sensor_gap_mm"
        ]
    )

    ready_announced = False

    last_print_ms = (
        time.ticks_ms()
    )

    try:
        while True:
            raw1, raw2 = (
                pair.read_pair()
            )

            now_ms = (
                time.ticks_ms()
            )

            event1 = sensor1.update(
                raw1,
                now_ms,
            )

            event2 = sensor2.update(
                raw2,
                now_ms,
            )

            ready = (
                sensor1.reference_mm
                is not None
                and
                sensor2.reference_mm
                is not None
            )

            if (
                ready
                and
                not ready_announced
            ):
                print()
                print(
                    "============================================================"
                )

                print(
                    "AUTO REFERENCE READY"
                )

                print(
                    "S1 reference: {:.1f} mm".format(
                        sensor1.reference_mm
                    )
                )

                print(
                    "S2 reference: {:.1f} mm".format(
                        sensor2.reference_mm
                    )
                )

                print(
                    "Walking tests may start now."
                )

                print(
                    "============================================================"
                )
                print()

                ready_announced = (
                    True
                )

            if ready:
                tracker.update(
                    sensor1.active,
                    sensor2.active,
                    event1,
                    event2,
                    now_ms,
                )

            should_print = (
                event1[
                    "changed"
                ]
                or
                event2[
                    "changed"
                ]
                or
                time.ticks_diff(
                    now_ms,
                    last_print_ms,
                )
                >=
                STATUS_INTERVAL_MS
            )

            if should_print:
                print(
                    "{} | {} | {}".format(
                        _channel_text(
                            sensor1
                        ),

                        _channel_text(
                            sensor2
                        ),

                        tracker.live_text(
                            now_ms
                        ),
                    )
                )

                last_print_ms = (
                    now_ms
                )

            time.sleep_ms(
                LOOP_DELAY_MS
            )

    except KeyboardInterrupt:
        print()
        print(
            "============================================================"
        )

        print(
            "LIVE TEST STOPPED"
        )

        print(
            "============================================================"
        )

        print(
            "S1 rejected near/noise samples:",
            sensor1.rejected_near,
        )

        print(
            "S2 rejected near/noise samples:",
            sensor2.rejected_near,
        )


main()
