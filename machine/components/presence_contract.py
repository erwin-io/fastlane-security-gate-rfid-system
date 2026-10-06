# FASTLANE Presence Intelligence Contract v1
# Pure Python / MicroPython compatible. Keep this file stable between model versions.

CONTRACT_VERSION = 1
MODEL_SCHEMA = "fastlane_presence_rf"
MODEL_SCHEMA_VERSION = 1

# The trained model is allowed to emit only these semantic labels.
# UNKNOWN is produced by runtime when confidence is below threshold or a model
# is unavailable. Do not add/remove labels without intentionally changing the
# contract version.
LABEL_CLEAR = "CLEAR"
LABEL_ENTERING = "ENTERING"
LABEL_EXITING = "EXITING"
LABEL_STAYING = "STAYING"
LABEL_PEEK_BACK = "PEEK_BACK"
LABEL_UNKNOWN = "UNKNOWN"

MODEL_LABELS = (
    LABEL_CLEAR,
    LABEL_ENTERING,
    LABEL_EXITING,
    LABEL_STAYING,
    LABEL_PEEK_BACK,
)

OUTPUT_LABELS = MODEL_LABELS + (LABEL_UNKNOWN,)

# Stable feature order used by both the PC trainer and the ESP32 runtime.
FEATURE_NAMES = (
    "q1_first_sensor",
    "q2_last_sensor",
    "q3_active_mask",
    "q4_seen_mask",
    "sequence_len",
    "transition_count",
    "reversal_count",
    "episode_duration_ms",
    "s1_current_norm",
    "s2_current_norm",
    "s1_min_norm",
    "s2_min_norm",
    "s1_mean_norm",
    "s2_mean_norm",
    "s1_max_intrusion_norm",
    "s2_max_intrusion_norm",
    "s1_presence_fraction",
    "s2_presence_fraction",
    "both_presence_fraction",
    "active_fraction",
    "s1_motion_norm",
    "s2_motion_norm",
    "s1_net_change_norm",
    "s2_net_change_norm",
    "s1_range_norm",
    "s2_range_norm",
    "stationary_fraction",
    "activation_gap_ms",
)

MASK_NONE = 0
MASK_S1 = 1
MASK_S2 = 2
MASK_BOTH = 3


def sensor_name(sensor_no):
    if sensor_no == 1:
        return "SENSOR_1"
    if sensor_no == 2:
        return "SENSOR_2"
    return "NA"


def mask_name(mask):
    if mask == MASK_S1:
        return "SENSOR_1"
    if mask == MASK_S2:
        return "SENSOR_2"
    if mask == MASK_BOTH:
        return "SENSOR_1&SENSOR_2"
    return "NA"


def active_mask(p1, p2):
    return (MASK_S1 if p1 else 0) | (MASK_S2 if p2 else 0)


def _clip(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def _mean(values):
    if not values:
        return 0.0
    return sum(values) / float(len(values))


class DirectionEpisodeTracker:
    """Tracks the user's four Q values and the ordered sensor activations.

    Q1 = first sensor that became present during the current episode.
    Q2 = most recent sensor that became present. If the person reverses, the
         same sensor may become the last sensor again (S1 -> S2 -> S1).
    Q3 = sensor(s) currently detecting presence.
    Q4 = sensor(s) that detected presence at any time in the episode.

    A completed episode is preserved in last_episode after both sensors have
    remained clear for clear_stable_ms. This lets the runtime decide whether a
    forward/reverse passage really completed before the tracker resets.
    """

    def __init__(self, clear_stable_ms=500):
        self.clear_stable_ms = int(clear_stable_ms)
        self.last_episode = None
        self.reset()

    def reset(self):
        self.started_at = 0
        self.last_active_at = 0
        self.clear_since = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = MASK_NONE
        self.q4 = MASK_NONE
        self.sequence = []
        self.previous_p1 = False
        self.previous_p2 = False
        self.first_s1_at = 0
        self.first_s2_at = 0
        self.transition_count = 0
        self.reversal_count = 0
        self.active = False

    def _append_activation(self, sensor_no, now_ms):
        if sensor_no not in (1, 2):
            return
        if self.q1 == 0:
            self.q1 = sensor_no
        self.q2 = sensor_no
        if sensor_no == 1:
            self.q4 |= MASK_S1
            if not self.first_s1_at:
                self.first_s1_at = now_ms
        else:
            self.q4 |= MASK_S2
            if not self.first_s2_at:
                self.first_s2_at = now_ms

        if not self.sequence or self.sequence[-1] != sensor_no:
            if len(self.sequence) >= 2 and self.sequence[-2] == sensor_no:
                self.reversal_count += 1
            self.sequence.append(sensor_no)
            self.transition_count += 1

    def update(self, now_ms, p1, p2, s1_intrusion=0.0, s2_intrusion=0.0):
        now_ms = int(now_ms)
        p1 = bool(p1)
        p2 = bool(p2)
        mask = active_mask(p1, p2)

        rising1 = p1 and not self.previous_p1
        rising2 = p2 and not self.previous_p2

        if mask != MASK_NONE and not self.active:
            self.active = True
            self.started_at = now_ms
            self.clear_since = 0

        if rising1 and rising2:
            # Simultaneous sample: whichever has the stronger intrusion is
            # considered first, then the other. This keeps Q1 deterministic.
            if _safe_float(s1_intrusion) >= _safe_float(s2_intrusion):
                self._append_activation(1, now_ms)
                self._append_activation(2, now_ms)
            else:
                self._append_activation(2, now_ms)
                self._append_activation(1, now_ms)
        elif rising1:
            self._append_activation(1, now_ms)
        elif rising2:
            self._append_activation(2, now_ms)

        self.q3 = mask
        if mask != MASK_NONE:
            self.q4 |= mask
            self.last_active_at = now_ms
            self.clear_since = 0
        elif self.active:
            if not self.clear_since:
                self.clear_since = now_ms
            if now_ms - self.clear_since >= self.clear_stable_ms:
                self.last_episode = self.snapshot(now_ms, completed=True)
                self.reset()
                # Preserve that sensors are currently clear after reset.
                self.previous_p1 = p1
                self.previous_p2 = p2
                return self.last_episode

        self.previous_p1 = p1
        self.previous_p2 = p2
        return None

    def snapshot(self, now_ms, completed=False):
        now_ms = int(now_ms)
        if self.started_at:
            duration = max(0, now_ms - self.started_at)
        else:
            duration = 0

        gap = 0
        if self.first_s1_at and self.first_s2_at:
            # Positive = S1 triggered before S2. Negative = S2 before S1.
            gap = self.first_s2_at - self.first_s1_at

        return {
            "completed": bool(completed),
            "q1": int(self.q1),
            "q2": int(self.q2),
            "q3": int(self.q3),
            "q4": int(self.q4),
            "q1_name": sensor_name(self.q1),
            "q2_name": sensor_name(self.q2),
            "q3_name": mask_name(self.q3),
            "q4_name": mask_name(self.q4),
            "sequence": list(self.sequence),
            "sequence_text": ">".join("S{}".format(x) for x in self.sequence),
            "transition_count": int(self.transition_count),
            "reversal_count": int(self.reversal_count),
            "duration_ms": int(duration),
            "first_s1_at": int(self.first_s1_at),
            "first_s2_at": int(self.first_s2_at),
            "activation_gap_ms": int(gap),
        }


def _series_stats(values, background):
    bg = max(1.0, _safe_float(background, 1.0))
    clean = [_safe_float(v, bg) for v in values]
    if not clean:
        clean = [bg]

    current = clean[-1]
    minimum = min(clean)
    maximum = max(clean)
    mean = _mean(clean)
    max_intrusion = max(0.0, bg - minimum)
    motion = 0.0
    for i in range(1, len(clean)):
        motion += abs(clean[i] - clean[i - 1])
    net_change = clean[-1] - clean[0]
    value_range = maximum - minimum

    return {
        "current_norm": _clip(current / bg, 0.0, 3.0),
        "min_norm": _clip(minimum / bg, 0.0, 3.0),
        "mean_norm": _clip(mean / bg, 0.0, 3.0),
        "max_intrusion_norm": _clip(max_intrusion / bg, 0.0, 2.0),
        "motion_norm": _clip(motion / bg, 0.0, 20.0),
        "net_change_norm": _clip(net_change / bg, -3.0, 3.0),
        "range_norm": _clip(value_range / bg, 0.0, 3.0),
    }


def build_features(samples, snapshot, background1_mm, background2_mm, stationary_delta_mm=20):
    """Build one stable numeric feature vector from recent sensor history."""
    samples = samples or []
    snapshot = snapshot or {}

    s1_values = []
    s2_values = []
    p1_count = 0
    p2_count = 0
    both_count = 0
    active_count = 0
    stationary_steps = 0
    interval_count = 0

    previous = None
    for sample in samples:
        s1 = _safe_float(sample.get("s1_mm"), background1_mm)
        s2 = _safe_float(sample.get("s2_mm"), background2_mm)
        p1 = bool(sample.get("p1", False))
        p2 = bool(sample.get("p2", False))
        s1_values.append(s1)
        s2_values.append(s2)
        if p1:
            p1_count += 1
        if p2:
            p2_count += 1
        if p1 and p2:
            both_count += 1
        if p1 or p2:
            active_count += 1

        if previous is not None:
            interval_count += 1
            d1 = abs(s1 - previous[0])
            d2 = abs(s2 - previous[1])
            # Small movement on both channels counts as stationary, but only
            # while at least one sensor currently sees presence.
            if (p1 or p2) and d1 <= stationary_delta_mm and d2 <= stationary_delta_mm:
                stationary_steps += 1
        previous = (s1, s2)

    count = max(1, len(samples))
    s1_stats = _series_stats(s1_values, background1_mm)
    s2_stats = _series_stats(s2_values, background2_mm)

    values = {
        "q1_first_sensor": float(snapshot.get("q1", 0)),
        "q2_last_sensor": float(snapshot.get("q2", 0)),
        "q3_active_mask": float(snapshot.get("q3", 0)),
        "q4_seen_mask": float(snapshot.get("q4", 0)),
        "sequence_len": float(len(snapshot.get("sequence", []))),
        "transition_count": float(snapshot.get("transition_count", 0)),
        "reversal_count": float(snapshot.get("reversal_count", 0)),
        "episode_duration_ms": float(snapshot.get("duration_ms", 0)),
        "s1_current_norm": s1_stats["current_norm"],
        "s2_current_norm": s2_stats["current_norm"],
        "s1_min_norm": s1_stats["min_norm"],
        "s2_min_norm": s2_stats["min_norm"],
        "s1_mean_norm": s1_stats["mean_norm"],
        "s2_mean_norm": s2_stats["mean_norm"],
        "s1_max_intrusion_norm": s1_stats["max_intrusion_norm"],
        "s2_max_intrusion_norm": s2_stats["max_intrusion_norm"],
        "s1_presence_fraction": p1_count / float(count),
        "s2_presence_fraction": p2_count / float(count),
        "both_presence_fraction": both_count / float(count),
        "active_fraction": active_count / float(count),
        "s1_motion_norm": s1_stats["motion_norm"],
        "s2_motion_norm": s2_stats["motion_norm"],
        "s1_net_change_norm": s1_stats["net_change_norm"],
        "s2_net_change_norm": s2_stats["net_change_norm"],
        "s1_range_norm": s1_stats["range_norm"],
        "s2_range_norm": s2_stats["range_norm"],
        "stationary_fraction": (
            stationary_steps / float(max(1, interval_count))
        ),
        "activation_gap_ms": float(snapshot.get("activation_gap_ms", 0)),
    }

    return [float(values[name]) for name in FEATURE_NAMES]


def q_summary(snapshot):
    snapshot = snapshot or {}
    return {
        "Q1_first": sensor_name(snapshot.get("q1", 0)),
        "Q2_last": sensor_name(snapshot.get("q2", 0)),
        "Q3_current": mask_name(snapshot.get("q3", 0)),
        "Q4_seen": mask_name(snapshot.get("q4", 0)),
        "sequence": snapshot.get("sequence_text", ""),
    }
