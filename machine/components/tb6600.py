from machine import Pin
import time


class TB6600DualMotor:
    """One TB6600 driving two NEMA17 motors from one STEP/DIR stream.

    Motor 2 rotates in the opposite physical direction because one of its
    coil pairs is reversed at the TB6600 terminals. No second STEP/DIR output
    is required.

    The preferred API is non-blocking:
        start_move_to_angle(...)
        update() repeatedly from main loop

    move_to_angle(...) is kept as a compatibility blocking wrapper.
    """

    def __init__(
        self,
        step_pin,
        dir_pin,
        full_steps_per_rev=200,
        microstep=8,
        start_delay_us=2500,
        run_delay_us=900,
        accel_steps=80,
        step_idle_level=1,
        step_active_level=0,
        forward_dir_level=0,
        reverse_dir_level=1,
        dir_setup_ms=20,
    ):
        self.step_gpio = int(step_pin)
        self.dir_gpio = int(dir_pin)
        self.full_steps_per_rev = int(full_steps_per_rev)
        self.microstep = int(microstep)
        self.steps_per_rev = self.full_steps_per_rev * self.microstep
        self.steps_per_degree = self.steps_per_rev / 360.0
        self.start_delay_us = int(start_delay_us)
        self.run_delay_us = int(run_delay_us)
        self.accel_steps = int(accel_steps)
        self.step_idle_level = 1 if step_idle_level else 0
        self.step_active_level = 1 if step_active_level else 0
        self.forward_dir_level = 1 if forward_dir_level else 0
        self.reverse_dir_level = 1 if reverse_dir_level else 0
        self.dir_setup_ms = int(dir_setup_ms)

        self.step_pin = None
        self.dir_pin = None
        self.current_steps = 0
        self.moving = False
        self.initialized = False

        self.target_steps = 0
        self.target_angle = 0.0
        self.total_steps = 0
        self.steps_done = 0
        self.step_sign = 1
        self.direction_inverted = False

        # Non-blocking pulse state.
        # 0 = waiting to drive STEP active
        # 1 = waiting to return STEP idle / complete one pulse
        self.pulse_phase = 0
        self.next_edge_us = 0
        self.direction_ready_us = 0
        self.current_delay_us = self.start_delay_us

        self.last_result = "IDLE"
        self.last_error = ""

    def angle_to_steps(self, angle):
        return int(round(float(angle) * self.steps_per_degree))

    def steps_to_angle(self, steps):
        return float(steps) / self.steps_per_degree

    @property
    def angle(self):
        return self.steps_to_angle(self.current_steps)

    def initialize(self, assumed_closed_angle=0.0):
        self.step_pin = Pin(self.step_gpio, Pin.OUT, value=self.step_idle_level)
        self.dir_pin = Pin(self.dir_gpio, Pin.OUT, value=self.reverse_dir_level)
        self.current_steps = self.angle_to_steps(assumed_closed_angle)
        self.target_steps = self.current_steps
        self.target_angle = float(assumed_closed_angle)
        self.moving = False
        self.initialized = True
        self.last_result = "READY"
        self.last_error = ""

        print("TB6600 READY")
        print("STEP / PUL- : GPIO", self.step_gpio)
        print("DIR-        : GPIO", self.dir_gpio)
        print("MICROSTEP   : 1/{}".format(self.microstep))
        print("PULSES/REV  :", self.steps_per_rev)
        print("POSITION    :", round(self.angle, 2), "degrees (ASSUMED)")
        return True

    def _get_delay(self, step_index, total_steps):
        if total_steps <= 2:
            return self.start_delay_us

        accel_steps = min(self.accel_steps, total_steps // 2)
        if accel_steps <= 0:
            return self.run_delay_us

        if step_index < accel_steps:
            progress = step_index / accel_steps
            delay = self.start_delay_us - (
                (self.start_delay_us - self.run_delay_us) * progress
            )
            return int(delay)

        remaining = total_steps - step_index
        if remaining <= accel_steps:
            progress = remaining / accel_steps
            delay = self.start_delay_us - (
                (self.start_delay_us - self.run_delay_us) * progress
            )
            return int(delay)

        return self.run_delay_us

    def _set_direction(self, forward, direction_inverted=False):
        if direction_inverted:
            forward = not forward
        self.dir_pin.value(
            self.forward_dir_level if forward else self.reverse_dir_level
        )

    def start_move_to_angle(self, target_angle, enabled=True, direction_inverted=False):
        """Start an absolute move without blocking the application loop."""
        self.last_error = ""

        if not self.initialized or self.step_pin is None or self.dir_pin is None:
            self.last_error = "DRIVER_NOT_INITIALIZED"
            self.last_result = "ERROR"
            print("TB6600 ERROR: driver is not initialized")
            return False

        if not enabled:
            self.last_result = "DISABLED"
            print("TB6600 MOTOR DISABLED BY CONFIG - NO MOVEMENT")
            return True

        try:
            target_angle = float(target_angle)
        except Exception:
            self.last_error = "INVALID_ANGLE"
            self.last_result = "ERROR"
            print("TB6600 ERROR: invalid angle")
            return False

        if target_angle < 0.0 or target_angle > 360.0:
            self.last_error = "ANGLE_OUT_OF_RANGE"
            self.last_result = "ERROR"
            print("TB6600 ERROR: angle must be 0..360")
            return False

        if self.moving:
            self.last_error = "ALREADY_MOVING"
            self.last_result = "ERROR"
            print("TB6600 ERROR: move requested while already moving")
            return False

        target_steps = self.angle_to_steps(target_angle)
        difference = target_steps - self.current_steps

        self.target_steps = target_steps
        self.target_angle = target_angle

        if difference == 0:
            self.total_steps = 0
            self.steps_done = 0
            self.last_result = "AT_TARGET"
            print(
                "TB6600 ALREADY AT TARGET:",
                round(target_angle, 2),
                "deg",
            )
            return True

        forward = difference > 0
        self.total_steps = abs(difference)
        self.steps_done = 0
        self.step_sign = 1 if forward else -1
        self.direction_inverted = bool(direction_inverted)
        self.current_delay_us = self._get_delay(0, self.total_steps)
        self.pulse_phase = 0

        self._set_direction(forward, self.direction_inverted)
        now_us = time.ticks_us()
        self.direction_ready_us = time.ticks_add(
            now_us, self.dir_setup_ms * 1000
        )
        self.next_edge_us = self.direction_ready_us
        self.moving = True
        self.last_result = "MOVING"

        print()
        print("----------------------------------------")
        print("TB6600 DUAL-MOTOR MOVE START")
        print("CURRENT :", round(self.angle, 2), "deg")
        print("TARGET  :", round(target_angle, 2), "deg")
        print("STEPS   :", self.total_steps)
        print("STEP PIN: GPIO", self.step_gpio)
        print("DIR PIN : GPIO", self.dir_gpio)
        print("----------------------------------------")
        return True

    def update(self, during_step=None):
        """Advance the STEP pulse state machine by at most one edge.

        This function never sleeps. Call it as frequently as possible from the
        main loop. If web/network work momentarily takes longer, the motor may
        slow slightly but RFID, LED, relay, and web servicing remain responsive.
        """
        if not self.moving:
            return False

        now_us = time.ticks_us()
        if time.ticks_diff(now_us, self.next_edge_us) < 0:
            return True

        if self.pulse_phase == 0:
            # Start one STEP pulse.
            self.current_delay_us = self._get_delay(
                self.steps_done, self.total_steps
            )
            self.step_pin.value(self.step_active_level)
            self.pulse_phase = 1
            self.next_edge_us = time.ticks_add(now_us, self.current_delay_us)
            return True

        # Finish one STEP pulse and account for the physical step.
        self.step_pin.value(self.step_idle_level)
        self.current_steps += self.step_sign
        self.steps_done += 1
        self.pulse_phase = 0

        if during_step is not None:
            try:
                during_step()
            except Exception as e:
                print("TB6600 DURING-STEP CALLBACK ERROR:", repr(e))

        if self.steps_done >= self.total_steps:
            self.current_steps = self.target_steps
            self.moving = False
            self.next_edge_us = 0
            self.last_result = "COMPLETE"
            print("TB6600 MOVE COMPLETE:", round(self.angle, 2), "deg")
            return False

        self.next_edge_us = time.ticks_add(now_us, self.current_delay_us)
        return True

    def move_to_angle(
        self,
        target_angle,
        enabled=True,
        direction_inverted=False,
        during_step=None,
    ):
        """Compatibility blocking wrapper around the non-blocking motor API."""
        if not self.start_move_to_angle(
            target_angle,
            enabled=enabled,
            direction_inverted=direction_inverted,
        ):
            return False

        if self.last_result in ("DISABLED", "AT_TARGET"):
            return True

        while self.moving:
            self.update(during_step=during_step)
            time.sleep_us(50)

        return self.last_result == "COMPLETE"

    def stop(self):
        self.moving = False
        self.pulse_phase = 0
        self.next_edge_us = 0
        self.last_result = "STOPPED"
        if self.step_pin is not None:
            self.step_pin.value(self.step_idle_level)

    def status(self):
        progress = 100
        if self.total_steps > 0:
            progress = int((self.steps_done * 100) / self.total_steps)
            if progress > 100:
                progress = 100

        return {
            "initialized": self.initialized,
            "moving": self.moving,
            "angle": round(self.angle, 2),
            "target_angle": round(self.target_angle, 2),
            "steps": self.current_steps,
            "steps_done": self.steps_done,
            "total_steps": self.total_steps,
            "progress_percent": progress,
            "steps_per_rev": self.steps_per_rev,
            "microstep": self.microstep,
            "last_result": self.last_result,
            "last_error": self.last_error,
        }
