from machine import UART
import time

from components.card_id import card_id_from_frame_hex


class RDM6300:
    """RDM6300 125 kHz UART RFID reader.

    Frame: STX + 10 ASCII hex chars + 2 ASCII checksum chars + ETX.

    v2.2.0: the callback receives the 10-digit DECIMAL card ID
    ("030081DFD8" -> "0008511448", see components/card_id.py). The hex frame
    never leaves this driver.

    v1.9.3 runtime rule:
      - UART parsing is strictly bounded per update() call.
      - A continuously-present card can no longer trap the application in an
        unbounded ``while uart.any()`` loop and starve motors, LEDs or sensors.
      - At most one complete RFID callback is dispatched per cooperative loop.
    """

    def __init__(
        self,
        uart_id,
        rx_pin,
        tx_pin,
        baud=9600,
        max_bytes_per_update=32,
        max_frames_per_update=1,
        max_service_us=2500,
    ):
        self.uart_id = uart_id
        self.rx_pin = rx_pin
        self.tx_pin = tx_pin
        self.baud = baud
        self.uart = None
        self.buffer = bytearray()
        self.last_frame_time = 0

        self.max_bytes_per_update = max(1, int(max_bytes_per_update))
        self.max_frames_per_update = max(1, int(max_frames_per_update))
        self.max_service_us = max(500, int(max_service_us))

        self.bytes_processed = 0
        self.frames_processed = 0
        self.frames_dropped = 0

    def initialize(self):
        kwargs = {
            "baudrate": self.baud,
            "bits": 8,
            "parity": None,
            "stop": 1,
            "rx": self.rx_pin,
            # update() only reads after uart.any() says bytes are present.
            # Zero timeout prevents a UART read from becoming a hidden delay.
            "timeout": 0,
        }

        try:
            kwargs["timeout_char"] = 0
        except Exception:
            pass

        # RDM6300 is receive-only in FASTLANE. Leaving TX unassigned frees
        # GPIO47 for VL53L0X XSHUT while preserving the same UART reader.
        if self.tx_pin is not None:
            kwargs["tx"] = self.tx_pin

        try:
            self.uart = UART(self.uart_id, **kwargs)
        except TypeError:
            # Some MicroPython builds do not accept timeout_char.
            kwargs.pop("timeout_char", None)
            self.uart = UART(self.uart_id, **kwargs)

        self.buffer = bytearray()
        self.last_frame_time = 0
        self.bytes_processed = 0
        self.frames_processed = 0
        self.frames_dropped = 0

        print("RDM6300 READY: RX GPIO", self.rx_pin)
        print(
            "RDM6300 COOPERATIVE BUDGET:",
            self.max_bytes_per_update,
            "bytes /",
            self.max_frames_per_update,
            "frame(s) /",
            self.max_service_us,
            "us",
        )
        return True

    @staticmethod
    def verify_checksum(data_hex, checksum_hex):
        try:
            checksum = 0
            for i in range(0, 10, 2):
                checksum ^= int(data_hex[i:i + 2], 16)
            return checksum == int(checksum_hex, 16)
        except Exception:
            return False

    def process_frame(self, frame, callback, scan_interval_ms=100):
        if len(frame) != 14 or frame[0] != 0x02 or frame[13] != 0x03:
            return False

        now = time.ticks_ms()
        if (
            self.last_frame_time
            and time.ticks_diff(now, self.last_frame_time) < int(scan_interval_ms)
        ):
            return False

        try:
            data_hex = frame[1:11].decode("ascii").upper()
            checksum_hex = frame[11:13].decode("ascii").upper()
            int(data_hex, 16)
            int(checksum_hex, 16)
        except Exception:
            return False

        if not self.verify_checksum(data_hex, checksum_hex):
            print("RFID CHECKSUM ERROR")
            return False

        card_id = card_id_from_frame_hex(data_hex)
        if card_id is None:
            return False

        self.last_frame_time = now

        # A held card repeats frames. One console line per accepted frame;
        # UART parsing/decision stay bounded.
        print("RFID CARD:", card_id)

        if callback is not None:
            callback(card_id)

        self.frames_processed += 1
        return True

    def update(self, callback, scan_interval_ms=100):
        """Service a bounded amount of UART work and return quickly.

        The old implementation drained UART until ``uart.any() == 0``. An
        RDM6300 can continuously retransmit while a card is held near the reader,
        so the loop could keep refilling faster than Python drained it and starve
        the rest of the machine. This version limits bytes, callbacks and wall
        time for every cooperative update.
        """
        if self.uart is None:
            return 0

        started_us = time.ticks_us()
        bytes_this_call = 0
        frames_this_call = 0

        while bytes_this_call < self.max_bytes_per_update:
            if frames_this_call >= self.max_frames_per_update:
                break

            if time.ticks_diff(time.ticks_us(), started_us) >= self.max_service_us:
                break

            try:
                if not self.uart.any():
                    break
                data = self.uart.read(1)
            except Exception as exc:
                print("RDM6300 UART READ ERROR:", repr(exc))
                break

            if not data:
                break

            bytes_this_call += 1
            self.bytes_processed += 1
            value = data[0]

            if value == 0x02:
                self.buffer = bytearray([value])
                continue

            if len(self.buffer) == 0:
                continue

            self.buffer.append(value)

            if value == 0x03:
                frame = bytes(self.buffer)
                self.buffer = bytearray()
                if self.process_frame(frame, callback, scan_interval_ms):
                    frames_this_call += 1
            elif len(self.buffer) > 20:
                self.buffer = bytearray()
                self.frames_dropped += 1

        return frames_this_call

    def discard_pending(self, max_bytes=256):
        """Bounded drain of stale UART bytes.

        v2.0.0: called by main.py exactly once each time the RFID task is
        re-enabled (after motor motion or after the presence lockout), so a
        card frame that arrived while RFID was disabled can never authorize a
        user later. Reads in chunks and is hard-limited to max_bytes.
        """
        if self.uart is None:
            return 0

        drained = 0
        limit = max(1, int(max_bytes))
        self.buffer = bytearray()

        while drained < limit:
            try:
                waiting = self.uart.any()
                if not waiting:
                    break
                chunk = self.uart.read(min(int(waiting), limit - drained, 64))
            except Exception:
                break
            if not chunk:
                break
            drained += len(chunk)

        return drained
