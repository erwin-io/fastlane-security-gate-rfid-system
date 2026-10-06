from machine import Pin
import time


class SolenoidRelay:
    """Fail-safe FASTLANE solenoid relay with idempotent state writes.

    For this machine the relay is ACTIVE LOW:
        GPIO HIGH -> relay OFF -> solenoid unpowered -> LOCKED
        GPIO LOW  -> relay ON  -> solenoid powered   -> RELEASED/UNLOCKED

    v1.7.20 fixes serial flooding by printing only when the logical relay state
    actually changes. Repeated set_released(True) calls are intentionally silent.
    """

    def __init__(self, pin, active_low=True):
        self.gpio = int(pin)
        self.active_low = bool(active_low)
        self.pin = None
        self.initialized = False
        self.released = False
        self.unlock_until = 0

    @property
    def locked_level(self):
        return 1 if self.active_low else 0

    @property
    def released_level(self):
        return 0 if self.active_low else 1

    def initialize(self):
        # Create the GPIO already in the fail-safe LOCKED state. This preserves
        # the boot.py active-low rule and avoids an unwanted relay pulse.
        self.pin = Pin(self.gpio, Pin.OUT, value=self.locked_level)
        self.released = False
        self.unlock_until = 0
        self.initialized = True
        print(
            "SOLENOID RELAY READY: LOCKED GPIO{}={} {}".format(
                self.gpio,
                self.pin.value(),
                "ACTIVE_LOW" if self.active_low else "ACTIVE_HIGH",
            )
        )
        return True

    def _apply_state(self, released, force_gpio=False):
        if not self.initialized or self.pin is None:
            return False

        released = bool(released)
        desired_level = self.released_level if released else self.locked_level

        try:
            actual_level = self.pin.value()
        except Exception:
            actual_level = None

        changed = (self.released != released) or (actual_level != desired_level)

        # force_gpio is used by fail-safe lock paths. It guarantees the output
        # level without creating repeated serial output when already locked.
        if changed or force_gpio:
            self.pin.value(desired_level)

        if self.released != released:
            self.released = released
            print("SOLENOID LOCK:", "RELEASED" if released else "LOCKED")
        else:
            self.released = released

        if not released:
            self.unlock_until = 0

        return changed

    def set_released(self, released):
        """Set latch state. Repeating the same state is a silent no-op."""
        return self._apply_state(bool(released), force_gpio=False)

    def force_locked(self):
        """Fail-safe lock; always assert GPIO but log only on a state change."""
        self.unlock_until = 0
        return self._apply_state(False, force_gpio=True)

    # Backward-compatible optional timed-pulse helper. main.py currently owns
    # the gate timing, but keeping this API makes the component safe for older
    # callers and diagnostics.
    def unlock_for(self, duration_ms):
        try:
            duration_ms = max(0, int(duration_ms))
        except Exception:
            duration_ms = 0

        self.set_released(True)
        self.unlock_until = (
            time.ticks_add(time.ticks_ms(), duration_ms)
            if duration_ms > 0 else 0
        )
        return True

    def update(self):
        # IMPORTANT: no print and no GPIO rewrite while nothing changes.
        if not self.initialized:
            return False

        if self.unlock_until and time.ticks_diff(
            time.ticks_ms(), self.unlock_until
        ) >= 0:
            self.unlock_until = 0
            self.force_locked()
            return True

        return False

    def status(self):
        remaining_ms = 0
        if self.unlock_until:
            remaining_ms = max(
                0,
                time.ticks_diff(self.unlock_until, time.ticks_ms()),
            )

        level = None
        if self.pin is not None:
            try:
                level = self.pin.value()
            except Exception:
                pass

        return {
            "initialized": self.initialized,
            "gpio": self.gpio,
            "active_low": self.active_low,
            "released": bool(self.released),
            "locked": not bool(self.released),
            "gpio_level": level,
            "remaining_ms": remaining_ms,
        }
