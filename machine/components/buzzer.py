from machine import Pin
try:
    from machine import PWM
except ImportError:
    PWM = None
import time


class Buzzer:
    """Active buzzer/module controller with cooperative non-blocking patterns."""

    def __init__(self, pin, active_high=True, buzzer_type="ACTIVE",
                 volume_percent=100, tone_hz=2700, active_pwm_hz=20000):
        self.gpio = pin
        self.active_high = bool(active_high)
        self.pin = None
        # v2.1.9 volume control
        self.buzzer_type = "PASSIVE" if str(buzzer_type).upper() == "PASSIVE" else "ACTIVE"
        self.volume_percent = max(0, min(100, int(volume_percent)))
        self.tone_hz = max(100, int(tone_hz))
        self.active_pwm_hz = max(1000, int(active_pwm_hz))
        self.pwm = None

        self.running = False
        self.output_on = False
        self.remaining_beeps = 0
        self.on_ms = 0
        self.off_ms = 0
        self.next_change_ms = 0
        # v2.3.0 melody: list of (freq_hz, on_ms, off_ms) notes.
        self.melody = None
        self.melody_index = 0

    def _level(self, active):
        if self.active_high:
            return 1 if active else 0
        return 0 if active else 1

    def _needs_pwm(self):
        return self.buzzer_type == "PASSIVE" or self.volume_percent < 100

    def _on_duty_u16(self):
        if self.buzzer_type == "PASSIVE":
            duty = int(32768 * self.volume_percent / 100)   # 100 % = 50 % square
        else:
            duty = int(65535 * self.volume_percent / 100)
        return duty if self.active_high else 65535 - duty

    def _off_duty_u16(self):
        return 0 if self.active_high else 65535

    def initialize(self):
        self.pin = Pin(self.gpio, Pin.OUT, value=self._level(False))
        self.pwm = None
        if self._needs_pwm() and PWM is not None:
            freq = self.tone_hz if self.buzzer_type == "PASSIVE" else self.active_pwm_hz
            try:
                self.pwm = PWM(self.pin, freq=freq, duty_u16=self._off_duty_u16())
            except Exception as exc:
                print("BUZZER PWM INIT FAILED -> plain ON/OFF:", repr(exc))
                self.pwm = None
                self.pin = Pin(self.gpio, Pin.OUT, value=self._level(False))
        self.stop()
        print("BUZZER READY: GPIO", self.gpio, "| TYPE", self.buzzer_type,
              "| VOLUME", self.volume_percent, "%",
              "| PWM", ("{} Hz".format(self.tone_hz if self.buzzer_type == "PASSIVE"
                                        else self.active_pwm_hz) if self.pwm else "OFF"))
        return True

    def set_volume(self, percent):
        """Change volume at runtime (0-100). Re-initializes PWM if needed."""
        self.volume_percent = max(0, min(100, int(percent)))
        if self.pin is not None:
            self.initialize()

    def set(self, active):
        self.output_on = bool(active)
        if self.pwm is not None:
            try:
                self.pwm.duty_u16(self._on_duty_u16() if self.output_on else self._off_duty_u16())
            except Exception:
                pass
            return
        if self.pin is not None:
            if self.output_on and self.volume_percent <= 0:
                self.pin.value(self._level(False))
                return
            self.pin.value(self._level(self.output_on))

    def start_beep(self, duration_ms):
        duration_ms = int(duration_ms)
        if duration_ms <= 0:
            self.stop()
            return
        self.start_pattern(1, duration_ms, 0)

    def _set_tone(self, freq_hz):
        """PASSIVE buzzer: change the PWM pitch. ACTIVE buzzers make their own
        tone, so a melody is played as its rhythm (note lengths and gaps)."""
        if self.pwm is None or self.buzzer_type != "PASSIVE":
            return
        try:
            if freq_hz:
                self.pwm.freq(max(100, int(freq_hz)))
            else:
                self.pwm.freq(self.tone_hz)
        except Exception:
            pass

    def start_melody(self, notes):
        """Non-blocking melody. notes = ((freq_hz, on_ms, off_ms), ...).
        Replaces any running beep/pattern; update() advances it."""
        notes = tuple(notes or ())
        if not notes:
            self.stop()
            return
        self.stop()
        self.melody = notes
        self.melody_index = 0
        self.running = True
        self._start_note()

    def _start_note(self):
        freq, on_ms, off_ms = self.melody[self.melody_index]
        self.on_ms = max(1, int(on_ms))
        self.off_ms = max(0, int(off_ms))
        self._set_tone(freq)
        self.set(True)
        self.next_change_ms = time.ticks_add(time.ticks_ms(), self.on_ms)

    @staticmethod
    def melody_duration_ms(notes):
        return sum(int(n[1]) + int(n[2]) for n in notes)

    def start_pattern(self, count, on_ms, off_ms):
        if self.melody is not None:
            self.melody = None
            self._set_tone(0)
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

        if self.melody is not None:
            if self.output_on and self.off_ms > 0:
                self.set(False)
                self.next_change_ms = time.ticks_add(now, self.off_ms)
                return
            self.melody_index += 1
            if self.melody_index >= len(self.melody):
                self.melody = None
                self.set(False)
                self._set_tone(0)
                self.running = False
                self.next_change_ms = 0
                return
            self._start_note()
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
        if self.melody is not None:
            self.melody = None
            self._set_tone(0)
        self.running = False
        self.remaining_beeps = 0
        self.next_change_ms = 0
        self.set(False)
