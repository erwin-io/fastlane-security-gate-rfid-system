from machine import Pin
import neopixel
import time


class StatusMatrix:
    def __init__(
        self,
        pin,
        num_leds,
        width,
        height,
        serpentine,
        flip_x,
        flip_y,
        off,
        green,
        red,
        blue,
        arrow_image,
        x_image,
        standby_images,
    ):
        self.gpio = pin
        self.num_leds = num_leds
        self.width = width
        self.height = height
        self.serpentine = bool(serpentine)
        self.flip_x = bool(flip_x)
        self.flip_y = bool(flip_y)
        self.off = off
        self.green_base = green
        self.red_base = red
        self.blue_base = blue
        self.arrow_image = arrow_image
        self.x_image = x_image
        self.standby_images = standby_images

        self.matrix = None
        self.brightness_percent = 15
        self.grant_display_ms = 2000
        self.deny_display_ms = 2000
        self.standby_delay_ms = 1000
        self.standby_frame_ms = 300

        self.state = 0
        self.timer = 0
        self.result_duration_ms = 2000
        self.standby_frame = 0
        self.standby_frame_timer = 0
        # v2.1.11: steady red while the lane is not clear / taps not allowed.
        self.hold_red = False
        self.result_granted = False
        # v2.3.0: machine configuration mode = permanent red X; nothing else
        # (result, standby, hold release) may replace it until it is cleared.
        self.config_lock = False
        self._last_write = 0
        self.refresh_ms = 1000      # v2.5.2: periodic re-send of the frame

    def initialize(self):
        self.matrix = neopixel.NeoPixel(Pin(self.gpio, Pin.OUT), self.num_leds)
        self.clear()
        print("WS2812B READY: GPIO", self.gpio)
        return True

    def configure(self, tap_cfg):
        self.brightness_percent = max(1, min(100, int(tap_cfg.get("led_brightness_percent", 100))))
        self.grant_display_ms = int(tap_cfg.get("grant_display_ms", 2000))
        self.deny_display_ms = int(tap_cfg.get("deny_display_ms", 2000))
        self.standby_delay_ms = int(tap_cfg.get("standby_delay_ms", 1000))
        self.standby_frame_ms = int(tap_cfg.get("standby_frame_ms", 300))

    def _scaled_color(self, base):
        p = self.brightness_percent
        return tuple((x * p) // 100 for x in base)

    def clear(self):
        if self.matrix is None:
            return
        for i in range(self.num_leds):
            self.matrix[i] = self.off
        self._write()

    def _xy_to_index(self, x, y):
        if self.flip_x:
            x = self.width - 1 - x
        if self.flip_y:
            y = self.height - 1 - y
        if not self.serpentine:
            return y * self.width + x
        if y % 2 == 0:
            return y * self.width + x
        return y * self.width + (self.width - 1 - x)

    def _draw_image(self, image, color):
        if self.matrix is None:
            return
        for y in range(self.height):
            for x in range(self.width):
                bit_index = (self.height - 1 - y) * self.width + (self.width - 1 - x)
                on = (image >> bit_index) & 1
                self.matrix[self._xy_to_index(x, y)] = color if on else self.off
        self._write()

    def _write(self):
        self.matrix.write()
        self._last_write = time.ticks_ms()

    def refresh(self):
        """v2.5.2: re-send the current frame. A static image (red X, blank) is
        otherwise written once, so pixels corrupted by EMI on the data line
        stayed random until the next state change."""
        if self.matrix is None or not self.refresh_ms:
            return
        if time.ticks_diff(time.ticks_ms(), self._last_write) >= self.refresh_ms:
            self._write()

    def _draw_hold_red(self):
        self.state = 3
        self._draw_image(self.x_image, self._scaled_color(self.red_base))

    def set_config_lock(self, active):
        """v2.3.0: True -> steady red X now and until set_config_lock(False)."""
        active = bool(active)
        self.config_lock = active
        if active:
            self.hold_red = True
            self._draw_hold_red()
        else:
            self.hold_red = False
            self.clear()
            self.state = 2
            self.timer = time.ticks_ms()

    def set_hold_red(self, hold):
        """v2.1.11: hold the red X while presence / tap lockout is active.
        A running GREEN/RED result finishes first; standby resumes on release."""
        hold = bool(hold)
        if self.config_lock:
            return
        if hold == self.hold_red:
            return
        self.hold_red = hold
        if hold:
            if self.state != 1:
                self._draw_hold_red()
        elif self.state == 3 or (self.state in (1, 2) and not self.result_granted):
            # Taps are allowed again: show BLUE at once, never a stale red X.
            self.start_standby()

    def start_standby(self):
        if self.config_lock:
            if self.state != 3:
                self._draw_hold_red()
            return
        if self.hold_red:
            if self.state != 1:
                self._draw_hold_red()
            return
        self.state = 0
        self.standby_frame = 0
        self.standby_frame_timer = 0
        self._draw_image(self.standby_images[0], self._scaled_color(self.blue_base))

    def show_result(self, granted, duration_ms=None):
        if self.config_lock:
            return
        self.state = 1
        self.result_granted = bool(granted)
        self.timer = time.ticks_ms()
        if granted:
            self.result_duration_ms = self.grant_display_ms
            self._draw_image(self.arrow_image, self._scaled_color(self.green_base))
        else:
            self.result_duration_ms = self.deny_display_ms
            self._draw_image(self.x_image, self._scaled_color(self.red_base))
        if duration_ms is not None:
            self.result_duration_ms = max(1, int(duration_ms))

    def update(self):
        self.refresh()
        if self.config_lock:
            return
        now = time.ticks_ms()

        if self.state == 1:
            if time.ticks_diff(now, self.timer) >= self.result_duration_ms:
                if self.hold_red:
                    self._draw_hold_red()
                    return
                self.clear()
                self.state = 2
                self.timer = now
            return

        if self.state == 3:
            if not self.hold_red:
                self.start_standby()
            return

        if self.state == 2:
            if time.ticks_diff(now, self.timer) >= self.standby_delay_ms:
                self.start_standby()
            return

        if time.ticks_diff(now, self.standby_frame_timer) >= self.standby_frame_ms:
            self.standby_frame_timer = now
            self.standby_frame = (self.standby_frame + 1) % len(self.standby_images)
            self._draw_image(
                self.standby_images[self.standby_frame],
                self._scaled_color(self.blue_base),
            )
