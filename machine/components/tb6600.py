from machine import Pin
import time

try:
    import esp32
except ImportError:
    esp32 = None


class TB6600DualMotor:
    """FASTLANE dual-driver / dual-NEMA17 gate component.

    FASTLANE v1.7.1 hardware:

        DRIVER #1 / LEFT BARRIER
            ESP32 GPIO1  -> ULN2003A IN1 -> OUT1 -> TB6600 #1 PUL-
            ESP32 GPIO2  -> ULN2003A IN2 -> OUT2 -> TB6600 #1 DIR-

        DRIVER #2 / RIGHT BARRIER
            ESP32 GPIO40 -> ULN2003A IN3 -> OUT3 -> TB6600 #2 PUL-
            ESP32 GPIO41 -> ULN2003A IN4 -> OUT4 -> TB6600 #2 DIR-

        Both TB6600 PUL+ and DIR+ -> 5V.

    Current tested motor-to-driver mapping:
        A+ -> RED   / motor connector Pin 3
        A- -> BLUE  / motor connector Pin 1
        B+ -> GREEN / motor connector Pin 2
        B- -> BLACK / motor connector Pin 4

    Connector color reference remains:
        Pin 1 = BLUE, Pin 2 = GREEN, Pin 3 = RED, Pin 4 = BLACK.

    Motor #2 rotates physically opposite because its own DIR signal is inverted
    in software. Each motor has its own logical position counter so Motor #1 can
    stop at a physical endpoint while Motor #2 finishes its calculated travel.
    This also lets an obstruction reopen calculate a fresh remaining distance for
    each driver instead of assuming the two sides are still at exactly one angle.

    The class name is intentionally kept as TB6600DualMotor so existing imports
    remain compatible.
    """

    def __init__(
        self,
        step_pin,
        dir_pin,
        step2_pin=None,
        dir2_pin=None,
        full_steps_per_rev=200,
        microstep=8,
        start_delay_us=2500,
        run_delay_us=900,
        accel_steps=80,
        step_idle_level=0,
        step_active_level=1,
        forward_dir_level=1,
        reverse_dir_level=0,
        dir_setup_ms=30,
        motor2_opposite=True,
        max_pulses_per_update=6,
        max_burst_us=8000,
        open_soft_land_enabled=True,
        open_decel_start_percent=45,
        open_final_delay_us=3200,
        use_hardware_rmt=True,
        rmt_channel1=0,
        rmt_channel2=1,
        rmt_resolution_hz=1000000,
        rmt_chunk_pulses=12,
        rmt_chunk_max_us=30000,
        rmt_stall_timeout_ms=250,
    ):
        self.step_gpio = int(step_pin)
        self.dir_gpio = int(dir_pin)
        self.step2_gpio = int(step2_pin) if step2_pin is not None else None
        self.dir2_gpio = int(dir2_pin) if dir2_pin is not None else None

        self.full_steps_per_rev = int(full_steps_per_rev)
        self.microstep = int(microstep)
        self.steps_per_rev = self.full_steps_per_rev * self.microstep
        self.steps_per_degree = self.steps_per_rev / 360.0

        self.start_delay_us = int(start_delay_us)
        self.run_delay_us = int(run_delay_us)
        self.accel_steps = int(accel_steps)

        # ESP32-side levels BEFORE ULN2003A inversion.
        self.step_idle_level = 1 if step_idle_level else 0
        self.step_active_level = 1 if step_active_level else 0
        self.forward_dir_level = 1 if forward_dir_level else 0
        self.reverse_dir_level = 1 if reverse_dir_level else 0
        self.dir_setup_ms = int(dir_setup_ms)
        self.motor2_opposite = bool(motor2_opposite)

        # v1.7.5 smooth pulse scheduler.
        #
        # update() used to emit at most one pulse and then return to main.py.
        # That allowed unrelated cooperative services to insert a long,
        # irregular gap after every microstep. The gate therefore looked like
        # a low-frame-rate animation even though each pulse itself was valid.
        #
        # A short bounded burst keeps adjacent STEP pulses evenly spaced while
        # still returning to main.py often for ToF and endpoint safety.
        self.max_pulses_per_update = max(1, int(max_pulses_per_update))
        self.max_burst_us = max(1000, int(max_burst_us))

        # v1.7.6 OPEN soft landing. Maximum speed stays unchanged; only the
        # final section of an OPEN movement is progressively slowed.
        self.open_soft_land_enabled = bool(open_soft_land_enabled)
        # v2.1.7: gentle S-curve start for normal OPEN (set from config).
        self.open_accel_steps = None
        self.open_s_curve = False
        # v2.1.2: normal CLOSE = fast first half, smooth slowdown, slow into
        # the CLOSE switch. main.py sets these from config.
        # v2.1.4 gapless RMT: the next chunk is computed while the current one
        # plays and written rmt_chain_lead_us before it ends. write_pulses()
        # on a busy channel blocks until it finishes, then starts the new
        # waveform (~0.3 ms gap measured) instead of a 4-23 ms refill hole.
        self.rmt_prequeue_enabled = True
        self.rmt_chain_lead_us = 3000
        self.rmt_chained_batches = 0
        self.close_soft_land_enabled = True
        self.close_decel_start_percent = 50
        self.close_final_delay_us = max(self.run_delay_us, 3200)
        self.open_decel_start_percent = max(
            10, min(90, int(open_decel_start_percent))
        )
        self.open_final_delay_us = max(
            self.run_delay_us, int(open_final_delay_us)
        )

        # v2.0.9 ease-out jog state (set per move by start_jog_relative).
        self.ease_launch_steps = 0
        self.ease_final_delay_us = self.open_final_delay_us
        self.ease_fast_ms = 0
        self.ease_slow_ms = 0
        self.ease_start_ms = 0

        # v1.7.13 hardware pulse engine. RMT is the ESP32 peripheral designed
        # for accurate waveform generation. Each TB6600 gets its own channel,
        # so Python no longer has to toggle STEP1 then STEP2 around sleep_us().
        # Short chunks preserve fast limit/obstruction response.
        self.use_hardware_rmt = bool(use_hardware_rmt)
        self.rmt_channel1 = int(rmt_channel1)
        self.rmt_channel2 = int(rmt_channel2)
        self.rmt_resolution_hz = max(100000, int(rmt_resolution_hz))
        self.rmt_chunk_pulses = max(1, min(32, int(rmt_chunk_pulses)))
        self.rmt_chunk_max_us = max(2000, int(rmt_chunk_max_us))
        self.rmt_stall_timeout_ms = max(100, int(rmt_stall_timeout_ms))
        self.rmt1 = None
        self.rmt2 = None
        self.rmt_enabled = False
        self.pulse_engine = "BITBANG"
        self.rmt_init_error = ""
        self._rmt1_pending_pulses = 0
        self._rmt2_pending_pulses = 0
        self._rmt_last_chunk_pulses = 0
        self._rmt_pending_started_ms = None
        self._rmt_fallback_reported = False
        self._rmt_fallback_count = 0
        self._rmt_expected_done_us = None
        self.rmt_refill_gap_max_us = 0
        self.rmt_refill_gap_total_us = 0
        self.rmt_chained_batches = 0
        self.rmt_batches_sent = 0
        self.continuous_until_limit = False
        self._open_preload_until = [0, 0]
        self._open_preload_done = [False, False]
        self._open_preload_angle = [0.0, 0.0]
        self._open_preload_delay_us = 3500
        self._rmt_started_ms = [None, None]
        self._rmt_next = [None, None]
        self._rmt_expected_end_us = [None, None]

        self.step_pin = None
        self.dir_pin = None
        self.step2_pin = None
        self.dir2_pin = None

        self.initialized = False
        self.moving = False

        # Logical gate coordinates. Both use CLOSED=0 and OPEN=positive angle,
        # even though their physical shaft rotations are opposite.
        self.motor1_current_steps = 0
        self.motor2_current_steps = 0
        self.motor1_target_steps = 0
        self.motor2_target_steps = 0
        self.motor1_steps_done = 0
        self.motor2_steps_done = 0
        self.motor1_total_steps = 0
        self.motor2_total_steps = 0
        self.motor1_step_sign = 1
        self.motor2_step_sign = 1

        self.motor1_step_enabled = True
        self.motor2_step_enabled = True

        self.target_angle = 0.0
        self.total_steps = 0
        self.steps_done = 0
        self.direction_inverted = False
        self.movement_command = "IDLE"
        self.move_mode = "NORMAL"
        # Per-move recovery override. Normal OPEN/CLOSE stays on RMT, while
        # slow homing/endpoint recovery can deliberately bypass RMT so a
        # stopped/reversed RMT channel can never prevent physical recovery.
        self.force_bitbang_current_move = False

        self.requested_dir_level = self.reverse_dir_level
        self.applied_dir_level = self.reverse_dir_level
        self.requested_dir2_level = self.forward_dir_level if self.motor2_opposite else self.reverse_dir_level
        self.applied_dir2_level = self.requested_dir2_level

        self.direction_ready_at = 0
        self.current_delay_us = self.start_delay_us

        self.active_start_delay_us = self.start_delay_us
        self.active_run_delay_us = self.run_delay_us
        self.active_accel_steps = self.accel_steps

        self.last_result = "IDLE"
        self.last_error = ""
        self.last_position_sync_reason = ""
        self.last_motor1_sync_reason = ""
        self.last_motor2_sync_reason = ""

    # ========================================================
    # ESP32-S3 RMT HARDWARE PULSE ENGINE - v1.7.16
    # ========================================================

    def _make_rmt(self, channel, pin):
        """Create an RMT transmitter across MicroPython ESP32 API variants."""
        if esp32 is None or not hasattr(esp32, "RMT"):
            raise RuntimeError("esp32.RMT unavailable")

        # MicroPython v1.28/v1.29 style: resolution_hz is supported. Keep the
        # numeric channel for backward compatibility with older ESP32 builds.
        try:
            return esp32.RMT(
                int(channel),
                pin=pin,
                resolution_hz=int(self.rmt_resolution_hz),
                idle_level=bool(self.step_idle_level),
            )
        except TypeError:
            pass

        # Older MicroPython ESP32 API uses clock_div instead of resolution_hz.
        # 80 MHz source / divisor => requested resolution (best integer match).
        source_hz = 80000000
        clock_div = int(round(source_hz / float(self.rmt_resolution_hz)))
        if clock_div < 1:
            clock_div = 1
        elif clock_div > 255:
            clock_div = 255

        return esp32.RMT(
            int(channel),
            pin=pin,
            clock_div=clock_div,
            idle_level=bool(self.step_idle_level),
        )

    def _initialize_rmt_engine(self):
        self.rmt_enabled = False
        self.pulse_engine = "BITBANG"
        self.rmt_init_error = ""
        self.rmt1 = None
        self.rmt2 = None
        self._rmt1_pending_pulses = 0
        self._rmt2_pending_pulses = 0
        self._rmt_pending_started_ms = None

        if not self.use_hardware_rmt:
            self.rmt_init_error = "RMT_DISABLED_BY_CONFIG"
            return False

        if esp32 is None or not hasattr(esp32, "RMT"):
            self.rmt_init_error = "ESP32_RMT_NOT_AVAILABLE"
            return False

        try:
            self.rmt1 = self._make_rmt(self.rmt_channel1, self.step_pin)
            self.rmt2 = self._make_rmt(self.rmt_channel2, self.step2_pin)
            self.rmt_enabled = True
            self.pulse_engine = "RMT_DUAL_HARDWARE"
            return True
        except Exception as e:
            self.rmt_init_error = repr(e)
            try:
                if self.rmt1 is not None:
                    self.rmt1.deinit()
            except Exception:
                pass
            try:
                if self.rmt2 is not None:
                    self.rmt2.deinit()
            except Exception:
                pass
            self.rmt1 = None
            self.rmt2 = None
            self.rmt_enabled = False
            self.pulse_engine = "BITBANG"
            return False

    def _rmt_done(self, rmt):
        if rmt is None:
            return True
        try:
            return bool(rmt.wait_done(timeout=0))
        except TypeError:
            try:
                return bool(rmt.wait_done(0))
            except Exception:
                pass
        except Exception:
            pass
        try:
            return not bool(rmt.active())
        except Exception:
            return True

    def _rmt_stop(self, rmt):
        if rmt is None:
            return
        try:
            rmt.active(False)
            return
        except Exception:
            pass
        try:
            rmt.loop(False)
        except Exception:
            pass

    def _cancel_rmt_motor1(self):
        # Always request an RMT stop, even if our software pending counter is 0.
        # This clears a peripheral that may have become active without the Python
        # bookkeeping being updated (for example after an interrupted retry).
        if self.rmt1 is not None:
            self._rmt_stop(self.rmt1)
        self._rmt1_pending_pulses = 0
        try:
            self._rmt_next[0] = None
        except Exception:
            pass
        if self.step_pin is not None:
            try:
                self.step_pin.value(self.step_idle_level)
            except Exception:
                pass

    def _cancel_rmt_motor2(self):
        if self.rmt2 is not None:
            self._rmt_stop(self.rmt2)
        self._rmt2_pending_pulses = 0
        try:
            self._rmt_next[1] = None
        except Exception:
            pass
        if self.step2_pin is not None:
            try:
                self.step2_pin.value(self.step_idle_level)
            except Exception:
                pass

    def _cancel_all_rmt(self):
        self._cancel_rmt_motor1()
        self._cancel_rmt_motor2()
        self._rmt_pending_started_ms = None

    def _fallback_to_bitbang(self, reason):
        """Disable RMT without blocking and continue using software pulses.

        This is deliberately one-way for the current boot. A reboot can try RMT
        again. Avoiding repeated RMT allocate/deallocate cycles keeps long-running
        endpoint retry behavior deterministic and avoids heap fragmentation.
        """
        reason = str(reason or "RMT_RUNTIME_FALLBACK")
        self._cancel_all_rmt()

        for rmt in (self.rmt1, self.rmt2):
            if rmt is None:
                continue
            try:
                rmt.deinit()
            except Exception:
                pass

        self.rmt1 = None
        self.rmt2 = None
        self.rmt_enabled = False
        self.pulse_engine = "BITBANG_FALLBACK"
        self.rmt_init_error = reason
        self._rmt_fallback_count += 1

        # RMT temporarily owns the STEP GPIO matrix. After deinit, explicitly
        # restore both pins as normal GPIO outputs before software bit-bang.
        # Without this re-assertion a runtime RMT fault can leave the fallback
        # path extremely slow, intermittent, or apparently frozen.
        self.step_pin = Pin(self.step_gpio, Pin.OUT, value=self.step_idle_level)
        self.step2_pin = Pin(self.step2_gpio, Pin.OUT, value=self.step_idle_level)

        if not self._rmt_fallback_reported:
            self._rmt_fallback_reported = True
            print("TB6600 RMT FALLBACK -> BITBANG:", reason)
        return False

    def _rmt_units_from_us(self, microseconds):
        units = int(round(
            (float(microseconds) * float(self.rmt_resolution_hz)) / 1000000.0
        ))
        if units < 1:
            units = 1
        try:
            pulse_max = int(esp32.RMT.PULSE_MAX)
            if units > pulse_max:
                units = pulse_max
        except Exception:
            pass
        return units

    def _refresh_steps_done(self):
        # IMPORTANT dual-driver fix: once Motor #1 is stopped by its physical
        # endpoint, do not let its forced "complete" counter jump the shared
        # speed profile to 100% while Motor #2 is still travelling.
        if self.motor1_step_enabled and self.motor2_step_enabled:
            self.steps_done = max(self.motor1_steps_done, self.motor2_steps_done)
        elif self.motor1_step_enabled:
            self.steps_done = self.motor1_steps_done
        elif self.motor2_step_enabled:
            self.steps_done = self.motor2_steps_done
        else:
            self.steps_done = max(self.motor1_steps_done, self.motor2_steps_done)

    def _commit_rmt_motor1(self):
        count = int(self._rmt1_pending_pulses)
        if count <= 0:
            return False
        self.motor1_current_steps += self.motor1_step_sign * count
        self.motor1_steps_done += count
        self._rmt1_pending_pulses = 0
        return True

    def _commit_rmt_motor2(self):
        count = int(self._rmt2_pending_pulses)
        if count <= 0:
            return False
        self.motor2_current_steps += self.motor2_step_sign * count
        self.motor2_steps_done += count
        self._rmt2_pending_pulses = 0
        return True

    def _service_rmt_completion(self, during_step=None):
        committed = False

        if self._rmt1_pending_pulses > 0 and self._rmt_done(self.rmt1):
            committed = self._commit_rmt_motor1() or committed

        if self._rmt2_pending_pulses > 0 and self._rmt_done(self.rmt2):
            committed = self._commit_rmt_motor2() or committed

        if committed:
            self._refresh_steps_done()
            if during_step is not None:
                try:
                    during_step()
                except Exception as e:
                    print("TB6600 RMT CALLBACK ERROR:", repr(e))

        if self._rmt1_pending_pulses <= 0 and self._rmt2_pending_pulses <= 0:
            self._rmt_pending_started_ms = None

        return committed

    def _build_rmt_chunk(self, shared_remaining):
        """Return (pulse_count, delay_us_list) within the RMT safety budget."""
        count = 0
        elapsed_us = 0
        delays = []
        max_count = min(int(self.rmt_chunk_pulses), int(shared_remaining))

        while count < max_count:
            delay_us = int(self._get_delay(self.steps_done + count, self.total_steps))
            cycle_us = delay_us * 2

            # Always allow at least one pulse. After that, keep the queued
            # hardware motion inside the configured safety/refill window.
            if count > 0 and elapsed_us + cycle_us > self.rmt_chunk_max_us:
                break

            delays.append(delay_us)
            elapsed_us += cycle_us
            count += 1

        return count, delays

    def _durations_for_rmt(self, delays, pulse_count):
        durations = []
        for delay_us in delays[:pulse_count]:
            units = self._rmt_units_from_us(delay_us)
            durations.append(units)
            durations.append(units)
        return tuple(durations)

    def _remaining_pulses(self, motor_no):
        enabled = self.motor1_step_enabled if motor_no == 1 else self.motor2_step_enabled
        if not enabled:
            return 0
        done = self.motor1_steps_done if motor_no == 1 else self.motor2_steps_done
        total = self.motor1_total_steps if motor_no == 1 else self.motor2_total_steps
        if done >= total and (self.continuous_until_limit or self._open_preload_until[motor_no - 1]):
            total = done + self.rmt_chunk_pulses
            if motor_no == 1:
                self.motor1_total_steps = total
            else:
                self.motor2_total_steps = total
        return max(0, total - done)

    def open_preload_active(self):
        return bool(self._open_preload_until[0] or self._open_preload_until[1])

    def preload_motor_open(self, motor_no, angle, duration_ms, delay_us):
        """One bounded slow push on that side's first OPEN-switch arrival."""
        index = motor_no - 1
        if self._open_preload_until[index]:
            return True
        enabled = self.motor1_step_enabled if motor_no == 1 else self.motor2_step_enabled
        if not enabled or self._open_preload_done[index]:
            return False
        # v2.1.3: do NOT cancel the side's RMT channel here. Stopping it with
        # rmt.active(False) mid-chunk left the channel unable to finish the
        # next waveform -> RMT_CHUNK_STALL ~250 ms -> the whole rest of the
        # move fell back to jittery software pulses (seen on every cycle).
        # The queued chunk (<= TB6600_RMT_CHUNK_MAX_US) simply finishes and
        # the next refill already uses the slow push delay.
        self._open_preload_angle[index] = float(angle)
        self._open_preload_delay_us = max(self.run_delay_us, int(delay_us))
        self._open_preload_until[index] = time.ticks_add(time.ticks_ms(), int(duration_ms))
        self._rmt_expected_end_us[index] = None
        self._remaining_pulses(motor_no)
        print("MOTOR #{} {}: SLOW SEATING PUSH {} ms".format(
            motor_no, self.movement_command, duration_ms))
        return True

    def service_open_preloads(self):
        now = time.ticks_ms()
        for index in (0, 1):
            deadline = self._open_preload_until[index]
            if deadline and time.ticks_diff(now, deadline) >= 0:
                self._open_preload_until[index] = 0
                self._open_preload_done[index] = True
                angle = self._open_preload_angle[index]
                if index == 0:
                    self.confirm_motor1_endpoint(angle, "OPEN slow push complete")
                else:
                    self.confirm_motor2_endpoint(angle, "OPEN slow push complete")

    def _motor_delay(self, motor_no, index):
        if self._open_preload_until[motor_no - 1]:
            return self._open_preload_delay_us
        # Once the nominal angle is exhausted, keep its final slow speed.
        return self._get_delay(min(index, max(0, self.total_steps - 1)), self.total_steps)

    def _build_motor_waveform(self, motor_no, base_done):
        """Return (waveform, count, elapsed_us, last_delay) from base_done."""
        index = motor_no - 1
        enabled = self.motor1_step_enabled if motor_no == 1 else self.motor2_step_enabled
        if not enabled:
            return None
        total = self.motor1_total_steps if motor_no == 1 else self.motor2_total_steps
        if base_done >= total and (self.continuous_until_limit or self._open_preload_until[index]):
            total = base_done + self.rmt_chunk_pulses
            if motor_no == 1:
                self.motor1_total_steps = total
            else:
                self.motor2_total_steps = total
        remaining = total - base_done
        if remaining <= 0:
            return None
        delays = []
        elapsed = 0
        for offset in range(min(self.rmt_chunk_pulses, remaining)):
            delay = int(self._motor_delay(motor_no, base_done + offset))
            if delays and elapsed + delay * 2 > self.rmt_chunk_max_us:
                break
            delays.append(delay)
            elapsed += delay * 2
        if not delays:
            return None
        return (self._durations_for_rmt(delays, len(delays)), len(delays), elapsed, delays[-1])

    def _rmt_prequeue(self):
        """v2.1.4: prepare and chain the next chunk of each running side."""
        if not self.rmt_prequeue_enabled or not self.rmt_enabled:
            return False
        chained = False
        for motor_no in (1, 2):
            index = motor_no - 1
            pending = self._rmt1_pending_pulses if motor_no == 1 else self._rmt2_pending_pulses
            if pending <= 0:
                continue
            enabled = self.motor1_step_enabled if motor_no == 1 else self.motor2_step_enabled
            if not enabled:
                self._rmt_next[index] = None
                continue
            done = self.motor1_steps_done if motor_no == 1 else self.motor2_steps_done
            base = done + pending
            preload = bool(self._open_preload_until[index])
            nxt = self._rmt_next[index]
            if nxt is not None and (nxt[4] != base or nxt[5] != preload):
                nxt = None
            if nxt is None:
                built = self._build_motor_waveform(motor_no, base)
                if built is None:
                    self._rmt_next[index] = None
                    continue
                nxt = built + (base, preload)
                self._rmt_next[index] = nxt
            end_us = self._rmt_expected_end_us[index]
            if end_us is None:
                continue
            now_us = time.ticks_us()
            if time.ticks_diff(end_us, now_us) > self.rmt_chain_lead_us:
                continue
            rmt = self.rmt1 if motor_no == 1 else self.rmt2
            if rmt is None:
                continue
            try:
                # Blocks until the running waveform ends, then starts this one.
                rmt.write_pulses(nxt[0], bool(self.step_active_level))
            except Exception as exc:
                self._fallback_to_bitbang("RMT_CHAIN_WRITE_ERROR: " + repr(exc))
                return chained
            ret_us = time.ticks_us()
            # The previous chunk has completed: account for it.
            if motor_no == 1:
                self._commit_rmt_motor1()
                self._rmt1_pending_pulses = nxt[1]
            else:
                self._commit_rmt_motor2()
                self._rmt2_pending_pulses = nxt[1]
            start_us = end_us if time.ticks_diff(ret_us, end_us) < 2000 else ret_us
            gap = max(0, time.ticks_diff(ret_us, end_us) - 1000) if time.ticks_diff(ret_us, end_us) > 1000 else 0
            self.rmt_refill_gap_max_us = max(self.rmt_refill_gap_max_us, gap)
            self.rmt_refill_gap_total_us += gap
            self._rmt_started_ms[index] = time.ticks_ms()
            self._rmt_expected_end_us[index] = time.ticks_add(start_us, nxt[2])
            self._rmt_pending_started_ms = time.ticks_ms()
            self._rmt_last_chunk_pulses = nxt[1]
            self.rmt_batches_sent += 1
            self.rmt_chained_batches += 1
            self.current_delay_us = nxt[3]
            self._rmt_next[index] = None
            self._refresh_steps_done()
            chained = True
        return chained

    def _start_rmt_chunk(self):
        """Refill each idle driver independently, including its slow push.

        A slow Motor #1 must not hold up Motor #2's normal waveform (or vice
        versa). No channel is written while its prior waveform is active.
        """
        submitted = False
        for motor_no in (1, 2):
            pending = self._rmt1_pending_pulses if motor_no == 1 else self._rmt2_pending_pulses
            if pending:
                continue
            remaining = self._remaining_pulses(motor_no)
            if not remaining:
                continue
            rmt = self.rmt1 if motor_no == 1 else self.rmt2
            if rmt is None or not self._rmt_done(rmt):
                return self._fallback_to_bitbang("RMT{}_BUSY_BEFORE_WRITE".format(motor_no))
            done = self.motor1_steps_done if motor_no == 1 else self.motor2_steps_done
            index0 = motor_no - 1
            nxt = self._rmt_next[index0]
            preload_now = bool(self._open_preload_until[index0])
            if nxt is not None and nxt[4] == done and nxt[5] == preload_now:
                # v2.1.4: already computed while the previous chunk played.
                waveform, count_n, elapsed, last_delay = nxt[0], nxt[1], nxt[2], nxt[3]
                delays = [last_delay] * count_n
                self._rmt_next[index0] = None
            else:
                delays = []
                elapsed = 0
                for offset in range(min(self.rmt_chunk_pulses, remaining)):
                    delay = int(self._motor_delay(motor_no, done + offset))
                    if delays and elapsed + delay * 2 > self.rmt_chunk_max_us:
                        break
                    delays.append(delay)
                    elapsed += delay * 2
                waveform = self._durations_for_rmt(delays, len(delays))
                self._rmt_next[index0] = None
            try:
                started_us = time.ticks_us()
                index = motor_no - 1
                previous_end = self._rmt_expected_end_us[index]
                if previous_end is not None:
                    gap = max(0, time.ticks_diff(started_us, previous_end))
                    self.rmt_refill_gap_max_us = max(self.rmt_refill_gap_max_us, gap)
                    self.rmt_refill_gap_total_us += gap
                rmt.write_pulses(waveform, bool(self.step_active_level))
                if motor_no == 1:
                    self._rmt1_pending_pulses = len(delays)
                else:
                    self._rmt2_pending_pulses = len(delays)
                self._rmt_started_ms[index] = time.ticks_ms()
                self._rmt_expected_end_us[index] = time.ticks_add(started_us, elapsed)
                self._rmt_pending_started_ms = time.ticks_ms()
                self._rmt_last_chunk_pulses = len(delays)
                self.rmt_batches_sent += 1
                self.current_delay_us = delays[-1]
                submitted = True
            except Exception as exc:
                return self._fallback_to_bitbang("RMT_RUNTIME_WRITE_ERROR: " + repr(exc))
        return submitted


    def _enter_bitbang_recovery_mode(self, reason=""):
        """Release RMT ownership so slow recovery pulses come from GPIO directly.

        Merely choosing the software scheduler is not enough while an RMT object
        still owns the STEP pin through the ESP32 GPIO matrix. Recovery therefore
        deinitializes both RMT transmitters first, then uses the existing Pin
        objects for deterministic low-speed dual STEP pulses. The next normal
        OPEN/CLOSE calls rearm_rmt_for_new_motion() and restores hardware RMT.
        """
        self._cancel_all_rmt()
        for rmt in (self.rmt1, self.rmt2):
            if rmt is None:
                continue
            try:
                rmt.deinit()
            except Exception:
                pass

        self.rmt1 = None
        self.rmt2 = None
        self.rmt_enabled = False
        self.pulse_engine = "BITBANG_RECOVERY"
        self._rmt_pending_started_ms = None

        # Reassert GPIO output mode/idle level after RMT releases the pins.
        self.step_pin = Pin(self.step_gpio, Pin.OUT, value=self.step_idle_level)
        self.step2_pin = Pin(self.step2_gpio, Pin.OUT, value=self.step_idle_level)

        if reason:
            print("TB6600 RECOVERY PULSE ENGINE:", reason, "-> BITBANG_RECOVERY")
        return True

    def rearm_rmt_for_new_motion(self, reason=""):
        """Recreate both RMT transmitters before every normal OPEN/CLOSE.

        v1.9.6 deliberately makes runtime fallback NON-STICKY. A transient RMT
        stall may finish the current move with bounded bit-bang pulses, but the
        next normal gate travel gets a fresh RMT pair. This prevents cycle #2
        from remaining permanently slow after a cycle #1 peripheral hiccup.
        """
        if self.moving:
            self.last_error = "RMT_REARM_REQUESTED_WHILE_MOVING"
            return False

        if not self.use_hardware_rmt:
            return True

        self._cancel_all_rmt()
        for rmt in (self.rmt1, self.rmt2):
            if rmt is None:
                continue
            try:
                rmt.deinit()
            except Exception:
                pass

        self.rmt1 = None
        self.rmt2 = None
        self.rmt_enabled = False

        ok = self._initialize_rmt_engine()
        label = str(reason or "NEW_MOTION")
        if ok:
            self._rmt_fallback_reported = False
            print("TB6600 RMT REARM:", label, "->", self.pulse_engine)
        else:
            print("TB6600 RMT REARM:", label, "-> BITBANG", self.rmt_init_error)
        # Failure is not fatal because update() will use bit-bang automatically.
        return True

    # ========================================================
    # POSITION CONVERSION / COMPATIBILITY
    # ========================================================

    def angle_to_steps(self, angle):
        return int(round(float(angle) * self.steps_per_degree))

    def steps_to_angle(self, steps):
        return float(steps) / self.steps_per_degree

    @property
    def motor1_angle(self):
        return self.steps_to_angle(self.motor1_current_steps)

    @property
    def motor2_angle(self):
        return self.steps_to_angle(self.motor2_current_steps)

    @property
    def angle(self):
        # Backward-compatible single dashboard angle: use the mean logical
        # position while moving and exact reconciled values at endpoints.
        return (self.motor1_angle + self.motor2_angle) / 2.0

    @property
    def current_steps(self):
        return int(round((self.motor1_current_steps + self.motor2_current_steps) / 2.0))

    @property
    def target_steps(self):
        return int(round((self.motor1_target_steps + self.motor2_target_steps) / 2.0))

    # ========================================================
    # INITIALIZATION / POSITION RECONCILIATION
    # ========================================================

    def initialize(self, assumed_closed_angle=0.0):
        self.last_error = ""

        if self.step_idle_level == self.step_active_level:
            self.last_error = "STEP_IDLE_AND_ACTIVE_LEVELS_ARE_EQUAL"
            self.last_result = "ERROR"
            print("TB6600 CONFIG ERROR:", self.last_error)
            return False

        if self.forward_dir_level == self.reverse_dir_level:
            self.last_error = "OPEN_AND_CLOSE_DIR_LEVELS_ARE_EQUAL"
            self.last_result = "ERROR"
            print("TB6600 CONFIG ERROR:", self.last_error)
            return False

        if self.step2_gpio is None or self.dir2_gpio is None:
            self.last_error = "SECOND_TB6600_PINS_NOT_CONFIGURED"
            self.last_result = "ERROR"
            print("TB6600 CONFIG ERROR:", self.last_error)
            return False

        self.step_pin = Pin(self.step_gpio, Pin.OUT, value=self.step_idle_level)
        self.dir_pin = Pin(self.dir_gpio, Pin.OUT, value=self.reverse_dir_level)
        self.step2_pin = Pin(self.step2_gpio, Pin.OUT, value=self.step_idle_level)

        start_dir2 = self._motor2_level(self.reverse_dir_level)
        self.dir2_pin = Pin(self.dir2_gpio, Pin.OUT, value=start_dir2)

        # Allocate two hardware RMT transmitters after GPIO objects exist.
        # Failure is non-fatal: the legacy bounded bit-bang engine remains as
        # an explicit fallback so the machine can still be diagnosed.
        self._initialize_rmt_engine()

        self.requested_dir_level = self.reverse_dir_level
        self.applied_dir_level = self.dir_pin.value()
        self.requested_dir2_level = start_dir2
        self.applied_dir2_level = self.dir2_pin.value()

        assumed_steps = self.angle_to_steps(assumed_closed_angle)
        self.motor1_current_steps = assumed_steps
        self.motor2_current_steps = assumed_steps
        self.motor1_target_steps = assumed_steps
        self.motor2_target_steps = assumed_steps
        self.target_angle = float(assumed_closed_angle)

        self.motor1_steps_done = 0
        self.motor2_steps_done = 0
        self.motor1_total_steps = 0
        self.motor2_total_steps = 0
        self.total_steps = 0
        self.steps_done = 0
        self.moving = False
        self.motor1_step_enabled = True
        self.motor2_step_enabled = True
        self.direction_ready_at = 0
        self.movement_command = "IDLE"
        self.move_mode = "NORMAL"
        self.initialized = True
        self.last_result = "READY"
        self.last_error = ""

        print("2x TB6600 + 1x ULN2003A READY")
        print("DRIVER #1 STEP: GPIO{} -> ULN IN1 -> OUT1 -> PUL-".format(self.step_gpio))
        print("DRIVER #1 DIR : GPIO{} -> ULN IN2 -> OUT2 -> DIR-".format(self.dir_gpio))
        print("DRIVER #2 STEP: GPIO{} -> ULN IN3 -> OUT3 -> PUL-".format(self.step2_gpio))
        print("DRIVER #2 DIR : GPIO{} -> ULN IN4 -> OUT4 -> DIR-".format(self.dir2_gpio))
        print("BOTH PUL+ / DIR+     : 5V")
        print("MICROSTEP            : 1/{}".format(self.microstep))
        print("PULSES/REV           :", self.steps_per_rev)
        print("M1 POSITION          :", round(self.motor1_angle, 2), "degrees (ASSUMED)")
        print("M2 POSITION          :", round(self.motor2_angle, 2), "degrees (ASSUMED)")
        print("MOTOR #2 OPPOSITE    :", self.motor2_opposite)
        print("BITBANG FALLBACK MAX :", self.max_pulses_per_update, "pulses")
        print("BITBANG FALLBACK BUDG:", self.max_burst_us, "us")
        print("PULSE ENGINE         :", self.pulse_engine)
        print("RMT REQUESTED        :", self.use_hardware_rmt)
        print("RMT CHUNK            :", self.rmt_chunk_pulses, "pulses /", self.rmt_chunk_max_us, "us max")
        print("RMT STALL WATCHDOG   :", self.rmt_stall_timeout_ms, "ms -> BITBANG fallback")
        if self.rmt_init_error:
            print("RMT INIT WARNING     :", self.rmt_init_error)
        print("OPEN SOFT LAND       :", self.open_soft_land_enabled)
        print("OPEN DECEL START     :", self.open_decel_start_percent, "%")
        print("OPEN FINAL DELAY     :", self.open_final_delay_us, "us half-pulse")
        return True

    def reconcile_position(self, angle, reason=""):
        if self.moving:
            return False

        try:
            angle = float(angle)
        except Exception:
            return False

        steps = self.angle_to_steps(angle)
        changed = (
            self.motor1_current_steps != steps
            or self.motor2_current_steps != steps
        )

        self.motor1_current_steps = steps
        self.motor2_current_steps = steps
        self.motor1_target_steps = steps
        self.motor2_target_steps = steps
        self.target_angle = angle
        self.motor1_steps_done = 0
        self.motor2_steps_done = 0
        self.motor1_total_steps = 0
        self.motor2_total_steps = 0
        self.total_steps = 0
        self.steps_done = 0
        self.last_position_sync_reason = str(reason or "")
        self.last_motor1_sync_reason = self.last_position_sync_reason
        self.last_motor2_sync_reason = self.last_position_sync_reason

        if changed:
            print(
                "TB6600 BOTH POSITION SYNC:",
                round(angle, 2),
                "deg",
                ("(" + self.last_position_sync_reason + ")") if self.last_position_sync_reason else "",
            )
        return True

    def reconcile_motor1_position(self, angle, reason=""):
        try:
            angle = float(angle)
        except Exception:
            return False

        steps = self.angle_to_steps(angle)
        self.motor1_current_steps = steps
        self.motor1_target_steps = steps
        self.last_motor1_sync_reason = str(reason or "")
        print(
            "TB6600 #1 POSITION SYNC:",
            round(angle, 2),
            "deg",
            ("(" + self.last_motor1_sync_reason + ")") if self.last_motor1_sync_reason else "",
        )
        return True

    def reconcile_motor2_position(self, angle, reason=""):
        try:
            angle = float(angle)
        except Exception:
            return False

        steps = self.angle_to_steps(angle)
        self.motor2_current_steps = steps
        self.motor2_target_steps = steps
        self.last_motor2_sync_reason = str(reason or "")
        print(
            "TB6600 #2 POSITION SYNC:",
            round(angle, 2),
            "deg",
            ("(" + self.last_motor2_sync_reason + ")") if self.last_motor2_sync_reason else "",
        )
        return True

    # ========================================================
    # SPEED PROFILE
    # ========================================================

    def _get_delay(self, step_index, total_steps):
        # v1.7.6: only NORMAL OPEN gets the extended soft-landing curve.
        # Latch shake, endpoint seek and normal CLOSE keep their prior timing.
        if (
            self.open_soft_land_enabled
            and self.move_mode == "NORMAL"
            and self.movement_command == "OPEN"
        ):
            return self._get_open_soft_landing_delay(step_index, total_steps)

        # v2.0.9: OPEN endpoint seek/retry ease-out (fast first, slowing to
        # a soft crawl before the OPEN switch / hard stop).
        if self.move_mode == "EASE_OUT_JOG":
            return self._get_ease_out_delay(step_index)

        start_delay = self.active_start_delay_us
        run_delay = self.active_run_delay_us
        accel_steps = self.active_accel_steps

        # v2.1.2 CLOSE: fast for the first part of the nominal travel, then a
        # smooth (quintic) slowdown to close_final_delay_us, held at that slow
        # speed past the nominal angle until the CLOSE switch stops the side.
        # Prevents the arm bouncing off the CLOSE switch (v2.1.1 full speed).
        if (
            self.close_soft_land_enabled
            and self.movement_command == "CLOSE"
            and self.move_mode == "NORMAL"
        ):
            return self._soft_landing_delay(
                step_index, total_steps,
                self.close_decel_start_percent, self.close_final_delay_us)

        if total_steps <= 2:
            return start_delay

        accel = min(accel_steps, total_steps // 2)
        if accel <= 0:
            return run_delay

        if step_index < accel:
            progress = step_index / accel
            return int(start_delay - ((start_delay - run_delay) * progress))

        remaining = total_steps - step_index
        if remaining <= accel:
            progress = remaining / accel
            return int(start_delay - ((start_delay - run_delay) * progress))

        return run_delay

    def _get_open_soft_landing_delay(self, step_index, total_steps):
        """Keep the fast v1.7.5 front section, then smoothly decelerate."""
        return self._soft_landing_delay(
            step_index, total_steps,
            self.open_decel_start_percent, self.open_final_delay_us,
            accel_steps=self.open_accel_steps, s_curve=self.open_s_curve)

    def _soft_landing_delay(self, step_index, total_steps, decel_percent, final_delay_us,
                            accel_steps=None, s_curve=False):
        """Shared fast-then-smooth profile (v2.1.2: used by OPEN and CLOSE)."""
        start_delay = self.start_delay_us
        fast_delay = self.run_delay_us
        final_delay_us = max(int(fast_delay), int(final_delay_us))

        if total_steps <= 2:
            return max(start_delay, final_delay_us)

        accel = min(self.accel_steps if accel_steps is None else int(accel_steps),
                    total_steps // 2)
        if accel > 0 and step_index < accel:
            progress = step_index / accel
            if s_curve:
                # v2.1.7 OPEN: smootherstep in SPEED. The old linear-delay
                # ramp raised speed fastest at the very top (where a stepper
                # has least torque) -> the weaker side slipped. Same total
                # time with 120 steps, but no torque spike.
                smooth = progress * progress * progress * (
                    progress * (progress * 6.0 - 15.0) + 10.0)
                v0 = 1.0 / float(start_delay)
                v1 = 1.0 / float(fast_delay)
                return int(round(1.0 / (v0 + (v1 - v0) * smooth)))
            return int(start_delay - ((start_delay - fast_delay) * progress))

        # Then hold the same maximum speed until the configured decel point.
        decel_start = int(
            (total_steps * decel_percent) / 100
        )
        decel_start = max(accel, decel_start)
        decel_start = min(max(0, total_steps - 1), decel_start)

        if step_index < decel_start:
            return fast_delay

        denominator = (total_steps - 1) - decel_start
        if denominator <= 0:
            return final_delay_us

        t = (step_index - decel_start) / denominator
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0

        # Quintic smootherstep: zero slope at both ends, so there is no
        # abrupt speed transition near the start or end of deceleration.
        smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

        # Interpolate speed (1/delay), not raw delay, for a more natural curve.
        fast_speed = 1.0 / float(fast_delay)
        slow_speed = 1.0 / float(final_delay_us)
        speed = fast_speed + ((slow_speed - fast_speed) * smooth)

        if speed <= 0.0:
            return final_delay_us

        delay = int(round(1.0 / speed))
        if delay < fast_delay:
            delay = fast_delay
        elif delay > final_delay_us:
            delay = final_delay_us
        return delay

    def _get_ease_out_delay(self, step_index):
        """v2.0.10 TIME-based ease-out for OPEN limit seek/retry.

        Feels like the normal OPEN: fast first, soft at the end.
          launch steps        : short ramp start_delay -> run_delay (no stall)
          0 .. fast_ms        : full normal OPEN speed (run_delay)
          fast_ms .. +ease_ms : ease-out braking. Speed follows the velocity
                                of an ease-out-cubic motion, (1-u)^2, from
                                full speed down to the final crawl.
          after               : final crawl until the side's OPEN switch
                                stops it (main.py ends the attempt).
        """
        start_delay = float(self.start_delay_us)
        fast_delay = float(self.run_delay_us)
        final_delay = float(self.ease_final_delay_us)
        launch = int(self.ease_launch_steps)

        if launch > 0 and step_index < launch:
            progress = step_index / float(launch)
            return int(start_delay - ((start_delay - fast_delay) * progress))

        elapsed = time.ticks_diff(time.ticks_ms(), self.ease_start_ms)
        if elapsed < self.ease_fast_ms:
            return int(fast_delay)
        if self.ease_slow_ms <= 0:
            return int(final_delay)

        u = (elapsed - self.ease_fast_ms) / float(self.ease_slow_ms)
        if u >= 1.0:
            return int(final_delay)
        if u < 0.0:
            u = 0.0
        shape = (1.0 - u) * (1.0 - u)
        fast_speed = 1.0 / fast_delay
        slow_speed = 1.0 / final_delay
        speed = slow_speed + ((fast_speed - slow_speed) * shape)
        delay = int(round(1.0 / speed))
        if delay < fast_delay:
            delay = int(fast_delay)
        elif delay > final_delay:
            delay = int(final_delay)
        return delay

    def _use_normal_speed_profile(self):
        self.active_start_delay_us = self.start_delay_us
        self.active_run_delay_us = self.run_delay_us
        self.active_accel_steps = self.accel_steps

    def _use_fixed_speed_profile(self, delay_us):
        delay_us = max(200, int(delay_us))
        self.active_start_delay_us = delay_us
        self.active_run_delay_us = delay_us
        self.active_accel_steps = 0

    # ========================================================
    # DIRECTION
    # ========================================================

    def _normalize_movement(self, movement, difference):
        if movement is None:
            return "OPEN" if difference > 0 else "CLOSE"

        movement = str(movement).strip().upper()
        if movement not in ("OPEN", "CLOSE"):
            raise ValueError("movement must be OPEN or CLOSE")
        return movement

    def _direction_level_for_motor1(self, movement, direction_inverted=False):
        opening = movement == "OPEN"
        if direction_inverted:
            opening = not opening
        return self.forward_dir_level if opening else self.reverse_dir_level

    def _motor2_level(self, motor1_equivalent_level):
        if not self.motor2_opposite:
            return motor1_equivalent_level
        return self.reverse_dir_level if motor1_equivalent_level == self.forward_dir_level else self.forward_dir_level

    def _direction_level_for_motor2(self, movement, direction_inverted=False):
        motor1_equivalent = self._direction_level_for_motor1(movement, direction_inverted)
        return self._motor2_level(motor1_equivalent)

    def _apply_directions(self, movement1, movement2, direction_inverted=False):
        # Direction changes are never allowed while a hardware STEP chunk is live.
        self._cancel_all_rmt()
        self._rmt_expected_done_us = None
        self.rmt_refill_gap_max_us = 0
        self.rmt_refill_gap_total_us = 0
        self.rmt_chained_batches = 0
        self.rmt_batches_sent = 0
        self._open_preload_until = [0, 0]
        self._open_preload_done = [False, False]
        self._rmt_started_ms = [None, None]
        self._rmt_expected_end_us = [None, None]
        self._rmt_next = [None, None]
        level1 = self._direction_level_for_motor1(movement1, direction_inverted)
        level2 = self._direction_level_for_motor2(movement2, direction_inverted)

        self.requested_dir_level = level1
        self.requested_dir2_level = level2

        self.step_pin.value(self.step_idle_level)
        self.step2_pin.value(self.step_idle_level)
        self.dir_pin.value(level1)
        self.dir2_pin.value(level2)

        self.applied_dir_level = self.dir_pin.value()
        self.applied_dir2_level = self.dir2_pin.value()

        if self.applied_dir_level != level1:
            self.last_error = "DIR1_GPIO_READBACK_MISMATCH"
            self.last_result = "ERROR"
            print("TB6600 #1 DIR ERROR: requested GPIO{}={} readback={}".format(
                self.dir_gpio, level1, self.applied_dir_level
            ))
            return False

        if self.applied_dir2_level != level2:
            self.last_error = "DIR2_GPIO_READBACK_MISMATCH"
            self.last_result = "ERROR"
            print("TB6600 #2 DIR ERROR: requested GPIO{}={} readback={}".format(
                self.dir2_gpio, level2, self.applied_dir2_level
            ))
            return False

        self.direction_ready_at = time.ticks_add(time.ticks_ms(), self.dir_setup_ms)

        print("TB6600 DIRECTION COMMANDS:")
        print("  DRIVER #1 {} -> GPIO{}={}".format(movement1, self.dir_gpio, level1))
        print("  DRIVER #2 {} -> GPIO{}={} (physical opposite={})".format(
            movement2, self.dir2_gpio, level2, self.motor2_opposite
        ))
        return True

    # ========================================================
    # PER-DRIVER STEP CHANNEL CONTROL
    # ========================================================

    def _set_step_channel_states(self, motor1_enabled, motor2_enabled):
        self.motor1_step_enabled = bool(motor1_enabled)
        self.motor2_step_enabled = bool(motor2_enabled)
        if self.step_pin is not None:
            self.step_pin.value(self.step_idle_level)
        if self.step2_pin is not None:
            self.step2_pin.value(self.step_idle_level)

    def stop_motor1_steps(self, reason=""):
        if not self.motor1_step_enabled:
            return False
        self.motor1_step_enabled = False
        self._cancel_rmt_motor1()
        if self.step_pin is not None:
            self.step_pin.value(self.step_idle_level)
        print("TB6600 #1 STEP CHANNEL STOPPED", ("(" + str(reason) + ")") if reason else "")
        return True

    def stop_motor2_steps(self, reason=""):
        if not self.motor2_step_enabled:
            return False
        self.motor2_step_enabled = False
        self._cancel_rmt_motor2()
        if self.step2_pin is not None:
            self.step2_pin.value(self.step_idle_level)
        print("TB6600 #2 STEP CHANNEL STOPPED", ("(" + str(reason) + ")") if reason else "")
        return True

    def confirm_motor1_endpoint(self, angle, reason=""):
        """Stop Motor #1 immediately and anchor its logical coordinate."""
        try:
            endpoint_steps = self.angle_to_steps(float(angle))
        except Exception:
            return False

        reason = str(reason or "")
        if (
            not self.motor1_step_enabled
            and self.motor1_current_steps == endpoint_steps
            and self.last_motor1_sync_reason == reason
        ):
            return False

        self.stop_motor1_steps(reason)
        # A physical limit switch is stronger evidence than the commanded pulse
        # count. Mark this driver's move as complete at that endpoint.
        self.motor1_steps_done = self.motor1_total_steps
        return self.reconcile_motor1_position(angle, reason)

    def confirm_motor2_endpoint(self, angle, reason=""):
        """Stop Motor #2 immediately and anchor its logical coordinate.

        v1.7.16 gives the right-side motor its own physical OPEN/CLOSE switches,
        so Motor #2 can now terminate independently exactly like Motor #1.
        """
        try:
            endpoint_steps = self.angle_to_steps(float(angle))
        except Exception:
            return False

        reason = str(reason or "")
        if (
            not self.motor2_step_enabled
            and self.motor2_current_steps == endpoint_steps
            and self.last_motor2_sync_reason == reason
        ):
            return False

        self.stop_motor2_steps(reason)
        self.motor2_steps_done = self.motor2_total_steps
        return self.reconcile_motor2_position(angle, reason)

    # ========================================================
    # START NORMAL MOVE
    # ========================================================

    def start_move_to_angle(
        self,
        target_angle,
        enabled=True,
        direction_inverted=False,
        movement=None,
        continuous_until_limit=False,
    ):
        self.last_error = ""
        self.continuous_until_limit = bool(continuous_until_limit)
        self._use_normal_speed_profile()

        if not self.initialized:
            self.last_error = "DRIVER_NOT_INITIALIZED"
            self.last_result = "ERROR"
            print("TB6600 ERROR: drivers are not initialized")
            return False

        if not enabled:
            self.last_result = "DISABLED"
            print("TB6600 MOTORS DISABLED BY CONFIG - NO MOVEMENT")
            return True

        try:
            target_angle = float(target_angle)
        except Exception:
            self.last_error = "INVALID_ANGLE"
            self.last_result = "ERROR"
            print("TB6600 ERROR: invalid target angle")
            return False

        if target_angle < 0.0 or target_angle > 360.0:
            self.last_error = "ANGLE_OUT_OF_RANGE"
            self.last_result = "ERROR"
            print("TB6600 ERROR: target angle must be 0..360")
            return False

        if self.moving:
            self.last_error = "ALREADY_MOVING"
            self.last_result = "ERROR"
            print("TB6600 ERROR: move requested while already moving")
            return False

        target_steps = self.angle_to_steps(target_angle)
        diff1 = target_steps - self.motor1_current_steps
        diff2 = target_steps - self.motor2_current_steps

        try:
            default_diff = diff1 if diff1 != 0 else diff2
            command = self._normalize_movement(movement, default_diff)
        except Exception as e:
            self.last_error = "INVALID_MOVEMENT_COMMAND"
            self.last_result = "ERROR"
            print("TB6600 ERROR:", repr(e))
            return False

        self.target_angle = target_angle
        self.motor1_target_steps = target_steps
        self.motor2_target_steps = target_steps
        self.motor1_total_steps = abs(diff1)
        self.motor2_total_steps = abs(diff2)
        self.motor1_steps_done = 0
        self.motor2_steps_done = 0
        self.motor1_step_sign = 1 if diff1 >= 0 else -1
        self.motor2_step_sign = 1 if diff2 >= 0 else -1
        self.motor1_step_enabled = self.motor1_total_steps > 0
        self.motor2_step_enabled = self.motor2_total_steps > 0
        self.total_steps = max(self.motor1_total_steps, self.motor2_total_steps)
        self.steps_done = 0
        self.direction_inverted = bool(direction_inverted)
        self.movement_command = command
        self.move_mode = "NORMAL"
        self.force_bitbang_current_move = False

        if self.total_steps <= 0:
            self.last_result = "AT_TARGET"
            print("TB6600 BOTH MOTORS ALREADY AT TARGET:", round(target_angle, 2), "deg")
            return True

        movement1 = command if diff1 == 0 else ("OPEN" if diff1 > 0 else "CLOSE")
        movement2 = command if diff2 == 0 else ("OPEN" if diff2 > 0 else "CLOSE")

        self.current_delay_us = self._get_delay(0, self.total_steps)
        if not self._apply_directions(movement1, movement2, self.direction_inverted):
            return False

        self.moving = True
        self.last_result = "MOVING"

        print()
        print("----------------------------------------")
        print("DUAL TB6600 INDEPENDENT MOVE START")
        print("TARGET     :", round(target_angle, 2), "deg")
        print("M1 CURRENT :", round(self.motor1_angle, 2), "deg")
        print("M1 STEPS   :", self.motor1_total_steps)
        print("M2 CURRENT :", round(self.motor2_angle, 2), "deg")
        print("M2 STEPS   :", self.motor2_total_steps)
        print("COMMAND    :", command)
        print("M1 DIR     : GPIO{}={}".format(self.dir_gpio, self.applied_dir_level))
        print("M2 DIR     : GPIO{}={}".format(self.dir2_gpio, self.applied_dir2_level))
        print("----------------------------------------")
        return True

    # ========================================================
    # SYNCHRONIZED LATCH-RELEASE JOG
    # ========================================================
    def start_jog_relative(
        self,
        degrees,
        movement="CLOSE",
        enabled=True,
        direction_inverted=False,
        delay_us=4500,
        motor1_enabled=True,
        motor2_enabled=True,
        force_bitbang=False,
        normal_speed_profile=False,
        ease_out_fast_ms=0,
        ease_out_slow_ms=0,
        ease_out_final_delay_us=None,
        ease_out_launch_steps=10,
        continuous_until_limit=False,
    ):
        self.last_error = ""
        self.continuous_until_limit = bool(continuous_until_limit)

        if not self.initialized:
            self.last_error = "DRIVER_NOT_INITIALIZED"
            self.last_result = "ERROR"
            print("TB6600 ERROR: drivers are not initialized")
            return False

        if not enabled:
            self.last_result = "DISABLED"
            print("TB6600 JOG DISABLED BY CONFIG - NO MOVEMENT")
            return True

        if self.moving:
            self.last_error = "ALREADY_MOVING"
            self.last_result = "ERROR"
            print("TB6600 ERROR: jog requested while already moving")
            return False

        try:
            degrees = abs(float(degrees))
        except Exception:
            self.last_error = "INVALID_JOG_DEGREES"
            self.last_result = "ERROR"
            print("TB6600 ERROR: invalid jog degrees")
            return False

        movement = str(movement).strip().upper()
        if movement not in ("OPEN", "CLOSE"):
            self.last_error = "INVALID_MOVEMENT_COMMAND"
            self.last_result = "ERROR"
            print("TB6600 ERROR: jog movement must be OPEN or CLOSE")
            return False

        jog_steps = abs(self.angle_to_steps(degrees))
        if jog_steps <= 0:
            self.last_result = "AT_TARGET"
            self.total_steps = 0
            self.steps_done = 0
            return True

        # v1.9.6:
        # Recovery jogs may explicitly request the same acceleration and run
        # speed as a normal OPEN/CLOSE. Latch shake and slow OPEN endpoint
        # recovery keep the legacy fixed-speed behavior by leaving this False.
        if normal_speed_profile:
            self._use_normal_speed_profile()
        else:
            self._use_fixed_speed_profile(delay_us)

        use_ease = bool(int(ease_out_fast_ms or 0) > 0 or int(ease_out_slow_ms or 0) > 0)
        if use_ease:
            self._use_normal_speed_profile()
            final = ease_out_final_delay_us
            if final is None:
                final = self.open_final_delay_us
            self.ease_final_delay_us = max(self.run_delay_us, int(final))
            self.ease_launch_steps = max(0, int(ease_out_launch_steps))
            self.ease_fast_ms = max(0, int(ease_out_fast_ms or 0))
            self.ease_slow_ms = max(0, int(ease_out_slow_ms or 0))
            self.ease_start_ms = time.ticks_ms()

        sign = 1 if movement == "OPEN" else -1

        motor1_enabled = bool(motor1_enabled)
        motor2_enabled = bool(motor2_enabled)
        if not motor1_enabled and not motor2_enabled:
            self.last_result = "AT_TARGET"
            self.total_steps = 0
            self.steps_done = 0
            return True

        self.motor1_target_steps = (
            self.motor1_current_steps + (sign * jog_steps)
            if motor1_enabled
            else self.motor1_current_steps
        )
        self.motor2_target_steps = (
            self.motor2_current_steps + (sign * jog_steps)
            if motor2_enabled
            else self.motor2_current_steps
        )
        self.motor1_total_steps = jog_steps if motor1_enabled else 0
        self.motor2_total_steps = jog_steps if motor2_enabled else 0
        self.motor1_steps_done = 0
        self.motor2_steps_done = 0
        self.motor1_step_sign = sign
        self.motor2_step_sign = sign
        self.motor1_step_enabled = motor1_enabled
        self.motor2_step_enabled = motor2_enabled
        self.total_steps = max(self.motor1_total_steps, self.motor2_total_steps)
        self.steps_done = 0
        self.direction_inverted = bool(direction_inverted)
        self.movement_command = movement
        self.move_mode = (
            "EASE_OUT_JOG"
            if use_ease
            else "NORMAL_PROFILE_JOG"
            if normal_speed_profile
            else "LATCH_JOG"
        )
        self.force_bitbang_current_move = bool(force_bitbang)
        self.target_angle = self.angle + (degrees if movement == "OPEN" else -degrees)

        if self.force_bitbang_current_move:
            self._enter_bitbang_recovery_mode(
                "{} {}".format(self.move_mode, movement)
            )

        if not self._apply_directions(movement, movement, self.direction_inverted):
            return False

        self.moving = True
        self.last_result = "MOVING"

        print()
        print("----------------------------------------")
        print("DUAL TB6600 LATCH-RELEASE JOG")
        print("JOG DEG :", round(degrees, 2))
        print("STEPS   :", jog_steps)
        print("MOVE    :", movement)
        if use_ease:
            print("PROFILE : EASE-OUT (fast -> soft landing)")
            print(
                "SPEED   : {} -> {} us launch, FAST {} us for {} ms, EASE-OUT to {} us over {} ms".format(
                    self.start_delay_us, self.run_delay_us, self.run_delay_us,
                    self.ease_fast_ms, self.ease_final_delay_us, self.ease_slow_ms,
                )
            )
        else:
            print(
                "PROFILE :",
                "NORMAL ACCEL/RUN"
                if normal_speed_profile
                else "FIXED",
            )
            print(
                "SPEED   : START={} us / RUN={} us".format(
                    self.active_start_delay_us,
                    self.active_run_delay_us,
                )
                if normal_speed_profile
                else "{} us half-pulse".format(self.active_run_delay_us)
            )
        print("MOTOR #1:", "STEP" if self.motor1_step_enabled else "HOLD")
        print("MOTOR #2:", "STEP" if self.motor2_step_enabled else "HOLD")
        print(
            "PULSE MODE:",
            "BITBANG_RECOVERY"
            if self.force_bitbang_current_move
            else self.pulse_engine,
        )
        print("----------------------------------------")
        return True

    # ========================================================
    # MOTOR #1-ONLY ENDPOINT SEEK
    # ========================================================

    def start_motor1_jog_relative(
        self,
        degrees,
        movement,
        enabled=True,
        direction_inverted=False,
        delay_us=4000,
    ):
        """Move only LEFT/Motor #1 a bounded distance looking for a limit switch."""
        self.last_error = ""

        if not self.initialized:
            self.last_error = "DRIVER_NOT_INITIALIZED"
            self.last_result = "ERROR"
            return False

        if not enabled:
            self.last_result = "DISABLED"
            return True

        if self.moving:
            self.last_error = "ALREADY_MOVING"
            self.last_result = "ERROR"
            return False

        try:
            degrees = abs(float(degrees))
        except Exception:
            self.last_error = "INVALID_JOG_DEGREES"
            self.last_result = "ERROR"
            return False

        movement = str(movement).strip().upper()
        if movement not in ("OPEN", "CLOSE"):
            self.last_error = "INVALID_MOVEMENT_COMMAND"
            self.last_result = "ERROR"
            return False

        jog_steps = abs(self.angle_to_steps(degrees))
        if jog_steps <= 0:
            self.last_result = "AT_TARGET"
            return True

        self._use_fixed_speed_profile(delay_us)
        sign = 1 if movement == "OPEN" else -1

        self.motor1_target_steps = self.motor1_current_steps + (sign * jog_steps)
        self.motor2_target_steps = self.motor2_current_steps
        self.motor1_total_steps = jog_steps
        self.motor2_total_steps = 0
        self.motor1_steps_done = 0
        self.motor2_steps_done = 0
        self.motor1_step_sign = sign
        self.motor2_step_sign = 1
        self.motor1_step_enabled = True
        self.motor2_step_enabled = False
        self.total_steps = jog_steps
        self.steps_done = 0
        self.direction_inverted = bool(direction_inverted)
        self.movement_command = movement
        self.move_mode = "MOTOR1_ENDPOINT_SEEK"
        self.force_bitbang_current_move = False
        self.target_angle = self.motor1_angle + (degrees if movement == "OPEN" else -degrees)

        # Motor #2 receives no STEP pulses, but leave its direction in the same
        # logical movement orientation for predictable status/debug output.
        if not self._apply_directions(movement, movement, self.direction_inverted):
            return False

        self.moving = True
        self.last_result = "MOVING"

        print()
        print("----------------------------------------")
        print("TB6600 #1 ENDPOINT SEEK")
        print("MOVE    :", movement)
        print("MAX DEG :", round(degrees, 2))
        print("STEPS   :", jog_steps)
        print("DELAY   :", self.active_run_delay_us, "us half-pulse")
        print("TB6600 #2 STEP: STOPPED DURING SEEK")
        print("----------------------------------------")
        return True

    # ========================================================
    # COOPERATIVE MOTOR SERVICE
    # ========================================================

    def _update_bitbang(self, during_step=None):
        """Legacy bounded software pulse scheduler used only as RMT fallback."""
        burst_started_us = time.ticks_us()
        pulses_emitted = 0

        while self.moving:
            self.service_open_preloads()
            self._remaining_pulses(1)
            self._remaining_pulses(2)
            m1_pulse = (
                self.motor1_step_enabled
                and self.motor1_steps_done < self.motor1_total_steps
            )
            m2_pulse = (
                self.motor2_step_enabled
                and self.motor2_steps_done < self.motor2_total_steps
            )

            if not m1_pulse and not m2_pulse:
                return self._finish_move()

            # Fallback shares a GPIO pulse period. Hardware mode above keeps
            # the two channels independent even when one is preloading.
            self.current_delay_us = max(
                self._motor_delay(1, self.motor1_steps_done) if m1_pulse else 0,
                self._motor_delay(2, self.motor2_steps_done) if m2_pulse else 0,
            )

            if pulses_emitted > 0:
                elapsed_us = time.ticks_diff(time.ticks_us(), burst_started_us)
                next_cycle_us = int(self.current_delay_us) * 2
                if elapsed_us + next_cycle_us > self.max_burst_us:
                    return True

            if m1_pulse:
                self.step_pin.value(self.step_active_level)
            if m2_pulse:
                self.step2_pin.value(self.step_active_level)
            time.sleep_us(self.current_delay_us)

            if m1_pulse:
                self.step_pin.value(self.step_idle_level)
            if m2_pulse:
                self.step2_pin.value(self.step_idle_level)
            time.sleep_us(self.current_delay_us)

            if m1_pulse:
                self.motor1_current_steps += self.motor1_step_sign
                self.motor1_steps_done += 1
            if m2_pulse:
                self.motor2_current_steps += self.motor2_step_sign
                self.motor2_steps_done += 1

            self._refresh_steps_done()
            pulses_emitted += 1
            self._remaining_pulses(1)
            self._remaining_pulses(2)

            if during_step is not None:
                try:
                    during_step()
                except Exception as e:
                    print("TB6600 BITBANG CALLBACK ERROR:", repr(e))

            m1_remaining = (
                self.motor1_step_enabled
                and self.motor1_steps_done < self.motor1_total_steps
            )
            m2_remaining = (
                self.motor2_step_enabled
                and self.motor2_steps_done < self.motor2_total_steps
            )
            if not m1_remaining and not m2_remaining:
                return self._finish_move()

            if pulses_emitted >= self.max_pulses_per_update:
                return True
            if time.ticks_diff(time.ticks_us(), burst_started_us) >= self.max_burst_us:
                return True

        return self.moving

    def update(self, during_step=None):
        """Service the dual TB6600 move without Python-timed dual STEP edges.

        RMT mode queues a short exact waveform independently to each STEP GPIO.
        Python is then free to service ToF, limit switches and the gate state
        machine while both TB6600s keep receiving evenly spaced pulses.

        The next call polls completion and immediately queues the next chunk.
        """
        if not self.moving:
            return False

        self.service_open_preloads()

        # Honor the existing TB6600 DIR setup delay before any STEP waveform.
        if self.direction_ready_at:
            if time.ticks_diff(time.ticks_ms(), self.direction_ready_at) < 0:
                return True
            self.direction_ready_at = 0

        if self.force_bitbang_current_move:
            return self._update_bitbang(during_step=during_step)

        if not self.rmt_enabled:
            return self._update_bitbang(during_step=during_step)

        # First account for any hardware chunk that completed since the last
        # cooperative service call. If one channel is still active, do not block.
        self._service_rmt_completion(during_step=during_step)

        for index, pending in enumerate((self._rmt1_pending_pulses, self._rmt2_pending_pulses)):
            if pending and self._rmt_started_ms[index] is not None:
                age_ms = time.ticks_diff(time.ticks_ms(), self._rmt_started_ms[index])
                if age_ms >= self.rmt_stall_timeout_ms:
                    self._fallback_to_bitbang(
                        "RMT_CHUNK_STALL_{}MS".format(age_ms)
                    )
                    return self._update_bitbang(during_step=during_step)

        # v2.1.4: chain the next chunk of each running side before it ends.
        self._rmt_prequeue()

        self._remaining_pulses(1)
        self._remaining_pulses(2)

        m1_remaining = (
            self.motor1_step_enabled
            and self.motor1_steps_done < self.motor1_total_steps
        )
        m2_remaining = (
            self.motor2_step_enabled
            and self.motor2_steps_done < self.motor2_total_steps
        )

        if not m1_remaining and not m2_remaining:
            return self._finish_move()

        # No hardware chunk is active, so queue the next one now. This call is
        # asynchronous: the RMT peripheral owns the pulse timing after submission.
        if self._start_rmt_chunk():
            return True

        # RMT may have fallen back at runtime. Continue the same movement with
        # the legacy scheduler rather than aborting the gate cycle.
        if not self.rmt_enabled:
            return self._update_bitbang(during_step=during_step)

        return True

    def _finish_move(self):
        self._cancel_all_rmt()
        self.moving = False
        self.direction_ready_at = 0
        self.last_result = "COMPLETE"
        self.force_bitbang_current_move = False
        if self.step_pin is not None:
            self.step_pin.value(self.step_idle_level)
        if self.step2_pin is not None:
            self.step2_pin.value(self.step_idle_level)

        print(
            "DUAL TB6600 MOVE COMPLETE:",
            self.movement_command,
            "| M1=",
            round(self.motor1_angle, 2),
            "deg | M2=",
            round(self.motor2_angle, 2),
            "deg | MODE=",
            self.move_mode,
        )
        if self.rmt_batches_sent:
            print("RMT CHAINED (gapless) BATCHES:", self.rmt_chained_batches)
            print("RMT REFILL ESTIMATE: max gap", self.rmt_refill_gap_max_us,
                  "us | total gaps", self.rmt_refill_gap_total_us,
                  "us | batches", self.rmt_batches_sent)
        return False

    # ========================================================
    # OPTIONAL BLOCKING COMPATIBILITY WRAPPER
    # ========================================================

    def move_to_angle(
        self,
        target_angle,
        enabled=True,
        direction_inverted=False,
        during_step=None,
        movement=None,
    ):
        if not self.start_move_to_angle(
            target_angle,
            enabled=enabled,
            direction_inverted=direction_inverted,
            movement=movement,
        ):
            return False

        if self.last_result in ("DISABLED", "AT_TARGET"):
            return True

        while self.moving:
            self.update(during_step=during_step)
            time.sleep_us(50)

        return self.last_result == "COMPLETE"

    # ========================================================
    # CONTROL / STATUS
    # ========================================================

    def stop(self):
        self._cancel_all_rmt()
        self._open_preload_until = [0, 0]
        self.continuous_until_limit = False
        self.moving = False
        self.direction_ready_at = 0
        self.last_result = "STOPPED"
        self.force_bitbang_current_move = False
        if self.step_pin is not None:
            self.step_pin.value(self.step_idle_level)
        if self.step2_pin is not None:
            self.step2_pin.value(self.step_idle_level)

    def status(self):
        progress = 100
        if self.total_steps > 0:
            progress = int((self.steps_done * 100) / self.total_steps)
            if progress > 100:
                progress = 100

        m1_progress = 100
        if self.motor1_total_steps > 0:
            m1_progress = int((self.motor1_steps_done * 100) / self.motor1_total_steps)
            if m1_progress > 100:
                m1_progress = 100

        m2_progress = 100
        if self.motor2_total_steps > 0:
            m2_progress = int((self.motor2_steps_done * 100) / self.motor2_total_steps)
            if m2_progress > 100:
                m2_progress = 100

        return {
            "initialized": self.initialized,
            "moving": self.moving,
            "movement": self.movement_command,
            "move_mode": self.move_mode,
            "angle": round(self.angle, 2),
            "motor1_angle": round(self.motor1_angle, 2),
            "motor2_angle": round(self.motor2_angle, 2),
            "target_angle": round(self.target_angle, 2),
            "steps": self.current_steps,
            "steps_done": self.steps_done,
            "total_steps": self.total_steps,
            "progress_percent": progress,
            "steps_per_rev": self.steps_per_rev,
            "microstep": self.microstep,
            "active_start_delay_us": self.active_start_delay_us,
            "active_run_delay_us": self.active_run_delay_us,
            "active_accel_steps": self.active_accel_steps,
            "max_pulses_per_update": self.max_pulses_per_update,
            "max_burst_us": self.max_burst_us,
            "pulse_engine": self.pulse_engine,
            "rmt_requested": self.use_hardware_rmt,
            "rmt_enabled": self.rmt_enabled,
            "rmt_channel1": self.rmt_channel1,
            "rmt_channel2": self.rmt_channel2,
            "rmt_resolution_hz": self.rmt_resolution_hz,
            "rmt_chunk_pulses": self.rmt_chunk_pulses,
            "rmt_chunk_max_us": self.rmt_chunk_max_us,
            "rmt_stall_timeout_ms": self.rmt_stall_timeout_ms,
            "rmt_pending_age_ms": (
                max(0, time.ticks_diff(time.ticks_ms(), self._rmt_pending_started_ms))
                if self._rmt_pending_started_ms is not None else 0
            ),
            "rmt_fallback_count": self._rmt_fallback_count,
            "rmt_pending_motor1_pulses": self._rmt1_pending_pulses,
            "rmt_pending_motor2_pulses": self._rmt2_pending_pulses,
            "rmt_last_chunk_pulses": self._rmt_last_chunk_pulses,
            "rmt_refill_gap_max_us": self.rmt_refill_gap_max_us,
            "rmt_refill_gap_total_us": self.rmt_refill_gap_total_us,
            "rmt_batches_sent": self.rmt_batches_sent,
            "rmt_init_error": self.rmt_init_error,
            "open_soft_land_enabled": self.open_soft_land_enabled,
            "open_decel_start_percent": self.open_decel_start_percent,
            "open_final_delay_us": self.open_final_delay_us,
            "interface": "1x ULN2003A / 2x TB6600",
            "motor2_opposite": self.motor2_opposite,
            "motor1_step_enabled": self.motor1_step_enabled,
            "motor2_step_enabled": self.motor2_step_enabled,
            "driver1": {
                "step_gpio": self.step_gpio,
                "dir_gpio": self.dir_gpio,
                "dir_requested_level": self.requested_dir_level,
                "dir_applied_level": self.applied_dir_level,
                "angle": round(self.motor1_angle, 2),
                "steps": self.motor1_current_steps,
                "steps_done": self.motor1_steps_done,
                "total_steps": self.motor1_total_steps,
                "progress_percent": m1_progress,
                "step_enabled": self.motor1_step_enabled,
                "position_sync_reason": self.last_motor1_sync_reason,
            },
            "driver2": {
                "step_gpio": self.step2_gpio,
                "dir_gpio": self.dir2_gpio,
                "dir_requested_level": self.requested_dir2_level,
                "dir_applied_level": self.applied_dir2_level,
                "angle": round(self.motor2_angle, 2),
                "steps": self.motor2_current_steps,
                "steps_done": self.motor2_steps_done,
                "total_steps": self.motor2_total_steps,
                "progress_percent": m2_progress,
                "step_enabled": self.motor2_step_enabled,
                "position_sync_reason": self.last_motor2_sync_reason,
            },
            # Backward-compatible fields.
            "step_gpio": self.step_gpio,
            "dir_gpio": self.dir_gpio,
            "step2_gpio": self.step2_gpio,
            "dir2_gpio": self.dir2_gpio,
            "step_idle_level": self.step_idle_level,
            "step_active_level": self.step_active_level,
            "open_dir_level": self.forward_dir_level,
            "close_dir_level": self.reverse_dir_level,
            "dir_requested_level": self.requested_dir_level,
            "dir_applied_level": self.applied_dir_level,
            "dir2_requested_level": self.requested_dir2_level,
            "dir2_applied_level": self.applied_dir2_level,
            "direction_inverted": self.direction_inverted,
            "last_result": self.last_result,
            "last_error": self.last_error,
            "position_sync_reason": self.last_position_sync_reason,
        }
