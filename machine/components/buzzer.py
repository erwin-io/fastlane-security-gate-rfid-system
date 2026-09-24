from machine import Pin
import time


class Buzzer:
    """Active buzzer/module controller with cooperative non-blocking patterns."""

    def __init__(self, pin, active_high=True):
        self.gpio = pin
        self.active_high = bool(active_high)
        self.pin = None

        self.running = False
        self.output_on = False
        self.remaining_beeps = 0
        self.on_ms = 0
        self.off_ms = 0
        self.next_change_ms = 0

    def _level(self, active):
        if self.active_high:
            return 1 if active else 0
        return 0 if active else 1

    def initialize(self):
        self.pin = Pin(self.gpio, Pin.OUT, value=self._level(False))
        self.stop()
        print("BUZZER READY: GPIO", self.gpio)
        return True

    def set(self, active):
        self.output_on = bool(active)
        if self.pin is not None:
            self.pin.value(self._level(self.output_on))

    def start_beep(self, duration_ms):
        duration_ms = int(duration_ms)
        if duration_ms <= 0:
            self.stop()
            return
        self.start_pattern(1, duration_ms, 0)

    def start_pattern(self, count, on_ms, off_ms):
        count = max(0, int(count))
        on_ms = max(1, int(on_ms))
        off_ms = max(0, int(off_ms))

        if count <= 0:
            self.stop()
            return

        self.remaining_beeps = count
        self.on_ms = on_ms
        self.off_ms = off_ms
        self.running = True
        self.set(True)
        self.next_change_ms = time.ticks_add(time.ticks_ms(), self.on_ms)

    def update(self):
        if not self.running:
            return

        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_change_ms) < 0:
            return

        if self.output_on:
            self.set(False)
            self.remaining_beeps -= 1

            if self.remaining_beeps <= 0:
                self.running = False
                self.next_change_ms = 0
                return

            self.next_change_ms = time.ticks_add(now, self.off_ms)
            return

        self.set(True)
        self.next_change_ms = time.ticks_add(now, self.on_ms)

    # Compatibility blocking APIs retained for standalone use.
    def beep(self, duration_ms):
        self.start_beep(duration_ms)
        while self.running:
            self.update()
            time.sleep_ms(1)

    def beep_pattern(self, count, on_ms, off_ms):
        self.start_pattern(count, on_ms, off_ms)
        while self.running:
            self.update()
            time.sleep_ms(1)

    def stop(self):
        self.running = False
        self.remaining_beeps = 0
        self.next_change_ms = 0
        self.set(False)
