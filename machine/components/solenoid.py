from machine import Pin
import time


class SolenoidRelay:
    """Relay-controlled solenoid lock.

    released=True means relay is triggered and the mechanical lock is released.
    released=False means relay is idle and the mechanical lock is in its normal
    locked state.
    """

    def __init__(self, pin, active_low=False):
        self.gpio = pin
        self.active_low = bool(active_low)
        self.locked_level = 1 if self.active_low else 0
        self.unlocked_level = 0 if self.active_low else 1
        self.pin = None
        self.released = False
        self.unlock_until = 0

    def initialize(self):
        self.pin = Pin(self.gpio, Pin.OUT, value=self.locked_level)
        self.released = False
        self.unlock_until = 0
        print(
            "SOLENOID RELAY READY: LOCKED GPIO{}={} ACTIVE_{}".format(
                self.gpio,
                self.pin.value(),
                "LOW" if self.active_low else "HIGH",
            )
        )
        return True

    def set_released(self, released):
        if self.pin is None:
            return
        released = bool(released)
        self.pin.value(self.unlocked_level if released else self.locked_level)
        self.released = released
        print("SOLENOID LOCK:", "RELEASED" if released else "LOCKED")

    def start_unlock_pulse(self, duration_ms=1000):
        duration_ms = max(1, int(duration_ms))
        self.set_released(True)
        self.unlock_until = time.ticks_add(time.ticks_ms(), duration_ms)
        print("SOLENOID UNLOCK PULSE:", duration_ms, "ms")

    def update(self):
        if not self.released:
            self.unlock_until = 0
            return
        if self.unlock_until == 0:
            return
        if time.ticks_diff(time.ticks_ms(), self.unlock_until) >= 0:
            self.force_locked()
            print("SOLENOID PULSE COMPLETE: RELAY BACK TO LOCK")

    def force_locked(self):
        self.unlock_until = 0
        self.set_released(False)

    def remaining_ms(self):
        if not self.released or self.unlock_until == 0:
            return 0
        return max(0, time.ticks_diff(self.unlock_until, time.ticks_ms()))

    def status(self):
        return {
            "released": self.released,
            "remaining_ms": self.remaining_ms(),
            "gpio": self.gpio,
            "active_low": self.active_low,
        }
