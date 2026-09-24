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
        self.brightness_percent = 100
        self.grant_display_ms = 2000
        self.deny_display_ms = 2000
        self.standby_delay_ms = 1000
        self.standby_frame_ms = 300

        self.state = 0
        self.timer = 0
        self.result_duration_ms = 2000
        self.standby_frame = 0
        self.standby_frame_timer = 0

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
        self.matrix.write()

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
        self.matrix.write()

    def start_standby(self):
        self.state = 0
        self.standby_frame = 0
        self.standby_frame_timer = 0
        self._draw_image(self.standby_images[0], self._scaled_color(self.blue_base))

    def show_result(self, granted):
        self.state = 1
        self.timer = time.ticks_ms()
        if granted:
            self.result_duration_ms = self.grant_display_ms
            self._draw_image(self.arrow_image, self._scaled_color(self.green_base))
        else:
            self.result_duration_ms = self.deny_display_ms
            self._draw_image(self.x_image, self._scaled_color(self.red_base))

    def update(self):
        now = time.ticks_ms()

        if self.state == 1:
            if time.ticks_diff(now, self.timer) >= self.result_duration_ms:
                self.clear()
                self.state = 2
                self.timer = now
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
