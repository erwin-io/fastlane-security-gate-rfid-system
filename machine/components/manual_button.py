"""Debounced GPIO19 staff button, release-based; no blocking sleeps or IRQs.

v2.3.1 rules (one result per press, decided by how long it was held):
  held <  hold_ms (1.5 s), then released        -> TOO_SHORT  (nothing happens)
  held >= hold_ms and < long_hold_ms, released  -> SHORT      (staff open)
  held >= long_hold_ms (5 s)                    -> LONG       (configuration mode
                                                   lock/unlock, fires at 5 s while
                                                   still held; the release after it
                                                   is swallowed - never an open)
While held, ARMED is reported once when the hold reaches hold_ms, so the
firmware can chirp "release now to open".

Examples: 2 s + release = open; 1.0-1.4 s + release = nothing;
1.5 s held on to 4 s + release = open; held to 5 s = configuration mode.
"""
from machine import Pin
import time

ARMED = "ARMED"
SHORT = "SHORT"
TOO_SHORT = "TOO_SHORT"
LONG = "LONG"


class ManualOpenButton:
    def __init__(self, gpio=19, hold_ms=1500, debounce_ms=50, long_hold_ms=5000):
        self.gpio, self.hold_ms, self.debounce_ms = gpio, int(hold_ms), int(debounce_ms)
        self.long_hold_ms = max(int(long_hold_ms), self.hold_ms + 500)
        self.pin = None
        self.candidate = self.pressed = self.armed = False
        self.changed_at = 0
        self.press_started_at = 0
        self.long_fired = False
        self.armed_reported = False
        self.last_held_ms = 0          # duration of the last finished press

    def initialize(self):
        self.pin = Pin(self.gpio, Pin.IN, Pin.PULL_UP)
        self.candidate = self.pressed = self.pin.value() == 0
        self.changed_at = time.ticks_ms()
        # A button already held at boot must be released before it counts.
        self.armed = not self.pressed
        self.press_started_at = 0
        self.long_fired = False
        self.armed_reported = False
        print("STAFF BUTTON READY: GPIO{} -> GND | OPEN: HOLD >= {} ms THEN RELEASE"
              " (before {} ms) | CONFIG LOCK/UNLOCK: HOLD {} ms".format(
                  self.gpio, self.hold_ms, self.long_hold_ms, self.long_hold_ms))

    def held_ms(self):
        """How long the current (debounced) press has lasted; 0 if released."""
        if not self.pressed or not self.press_started_at:
            return 0
        return time.ticks_diff(time.ticks_ms(), self.press_started_at)

    def update(self):
        """Return ARMED, SHORT, TOO_SHORT, LONG or None. Call every loop pass."""
        if self.pin is None:
            return None
        now = time.ticks_ms()
        raw = self.pin.value() == 0
        if raw != self.candidate:
            self.candidate, self.changed_at = raw, now
        if time.ticks_diff(now, self.changed_at) < self.debounce_ms:
            return None

        if raw and not self.pressed:
            # Debounced press edge (press time = first stable contact).
            self.pressed = True
            self.press_started_at = self.changed_at
            self.long_fired = False
            self.armed_reported = False
            return None

        if not raw and self.pressed:
            # Debounced release edge: the decision is made here.
            self.pressed = False
            held = time.ticks_diff(self.changed_at, self.press_started_at)
            self.last_held_ms = held
            fired_long = self.long_fired
            self.press_started_at = 0
            self.long_fired = False
            self.armed_reported = False
            if not self.armed:
                self.armed = True          # first release after a boot-time hold
                return None
            if fired_long:
                return None                # release after LONG is swallowed
            if held >= self.hold_ms:
                return SHORT
            return TOO_SHORT

        if raw and self.pressed and self.armed and not self.long_fired:
            held = time.ticks_diff(now, self.press_started_at)
            if held >= self.long_hold_ms:
                self.long_fired = True
                self.last_held_ms = held
                return LONG
            if held >= self.hold_ms and not self.armed_reported:
                self.armed_reported = True
                return ARMED

        if not raw:
            self.armed = True
        return None
