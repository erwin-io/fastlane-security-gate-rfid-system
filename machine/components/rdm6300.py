from machine import UART
import time


class RDM6300:
    """RDM6300 125 kHz UART RFID reader.

    Frame: STX + 10 ASCII hex chars + 2 ASCII checksum chars + ETX.
    """

    def __init__(self, uart_id, rx_pin, tx_pin, baud=9600):
        self.uart_id = uart_id
        self.rx_pin = rx_pin
        self.tx_pin = tx_pin
        self.baud = baud
        self.uart = None
        self.buffer = bytearray()
        self.last_frame_time = 0

    def initialize(self):
        self.uart = UART(
            self.uart_id,
            baudrate=self.baud,
            bits=8,
            parity=None,
            stop=1,
            rx=self.rx_pin,
            tx=self.tx_pin,
            timeout=50,
        )
        self.buffer = bytearray()
        self.last_frame_time = 0
        print("RDM6300 READY: RX GPIO", self.rx_pin)
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
            return

        now = time.ticks_ms()
        if self.last_frame_time and time.ticks_diff(now, self.last_frame_time) < int(scan_interval_ms):
            return
        self.last_frame_time = now

        try:
            data_hex = frame[1:11].decode("ascii").upper()
            checksum_hex = frame[11:13].decode("ascii").upper()
            int(data_hex, 16)
            int(checksum_hex, 16)
        except Exception:
            return

        if not self.verify_checksum(data_hex, checksum_hex):
            print("RFID CHECKSUM ERROR")
            return

        version = data_hex[0:2]
        card_hex = data_hex[2:10]
        card_decimal = int(card_hex, 16)

        print()
        print("----------------------------------------")
        print("RFID CARD DETECTED")
        print("----------------------------------------")
        print("Raw ID       :", data_hex)
        print("Version      :", version)
        print("Card HEX     :", card_hex)
        print("Card Decimal :", card_decimal)
        print("----------------------------------------")

        if callback is not None:
            callback(data_hex)

    def update(self, callback, scan_interval_ms=100):
        if self.uart is None:
            return
        while self.uart.any():
            data = self.uart.read(1)
            if not data:
                return

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
                self.process_frame(frame, callback, scan_interval_ms)
            elif len(self.buffer) > 20:
                self.buffer = bytearray()
