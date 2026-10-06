"""
MicroPython driver for SD cards using SPI bus.

Requires an SPI bus and a CS pin.  Provides readblocks and writeblocks
methods so the device can be mounted as a filesystem.

Example usage on pyboard:

    import pyb, sdcard, os
    sd = sdcard.SDCard(pyb.SPI(1), pyb.Pin.board.X5)
    pyb.mount(sd, '/sd2')
    os.listdir('/')

Example usage on ESP8266:

    import machine, sdcard, os
    sd = sdcard.SDCard(machine.SPI(1), machine.Pin(15))
    os.mount(sd, '/sd')
    os.listdir('/')

"""

from micropython import const
import time


_CMD_TIMEOUT = const(100)
_WRITE_BUSY_TIMEOUT_MS = const(750)
# v2.3.0 read hardening (FASTLANE): the data token is waited for with a time
# deadline instead of 100 polls, and a failed block read is retried after
# draining the card. On 05 Oct 2026 one "timeout waiting for response" left
# the card out of step and EVERY later read failed until a remount, which
# made the web UI's card list come back empty.
_READ_TOKEN_TIMEOUT_MS = const(300)
_READ_RETRIES = const(3)

_R1_IDLE_STATE = const(1 << 0)
# R1_ERASE_RESET = const(1 << 1)
_R1_ILLEGAL_COMMAND = const(1 << 2)
# R1_COM_CRC_ERROR = const(1 << 3)
# R1_ERASE_SEQUENCE_ERROR = const(1 << 4)
# R1_ADDRESS_ERROR = const(1 << 5)
# R1_PARAMETER_ERROR = const(1 << 6)
_TOKEN_CMD25 = const(0xFC)
_TOKEN_STOP_TRAN = const(0xFD)
_TOKEN_DATA = const(0xFE)


def _crc7(buf, n):
    crc = 0
    for i in range(n):
        crc ^= buf[i]
        for j in range(8):
            crc = ((crc << 1) ^ (0x12 * (crc >> 7))) & 0xFF
    return crc


class SDCard:
    def __init__(self, spi, cs, baudrate=1320000):
        self.spi = spi
        self.cs = cs

        self.cmdbuf = bytearray(6)
        self.dummybuf = bytearray(512)
        self.tokenbuf = bytearray(1)
        for i in range(512):
            self.dummybuf[i] = 0xFF
        self.dummybuf_memoryview = memoryview(self.dummybuf)

        # initialise the card
        self.init_card(baudrate)

    def init_spi(self, baudrate):
        try:
            master = self.spi.MASTER
        except AttributeError:
            # on ESP8266
            self.spi.init(baudrate=baudrate, phase=0, polarity=0)
        else:
            # on pyboard
            self.spi.init(master, baudrate=baudrate, phase=0, polarity=0)

    def init_card(self, baudrate):
        # init CS pin
        self.cs.init(self.cs.OUT, value=1)

        # init SPI bus; use low data rate for initialisation
        self.init_spi(100000)

        # clock card at least 100 cycles with cs high
        for i in range(16):
            self.spi.write(b"\xff")

        # CMD0: init card; should return _R1_IDLE_STATE (allow 5 attempts)
        for _ in range(5):
            if self.cmd(0, 0) == _R1_IDLE_STATE:
                break
        else:
            raise OSError("no SD card")

        # CMD8: determine card version
        r = self.cmd(8, 0x01AA, 4)
        self.version2 = False
        if r == _R1_IDLE_STATE:
            self.version2 = True
            self.init_card_v2()
        elif r == (_R1_IDLE_STATE | _R1_ILLEGAL_COMMAND):
            self.init_card_v1()
        else:
            raise OSError("couldn't determine SD card version")

        # get the number of sectors
        # CMD9: response R2 (R1 byte + 16-byte block read)
        if self.cmd(9, 0, 0, False) != 0:
            raise OSError("no response from SD card")
        csd = bytearray(16)
        self.readinto(csd)
        csd_type = csd[0] & 0xC0

        if csd_type == 0x40:  # CSD version 2.0 = SDHC/SDXC
            self.cdv = 1
            self.sectors = (((csd[7] & 0x3F) << 16 | csd[8] << 8 | csd[9]) + 1) * 1024
            print("[SDCard] CSD v2 -> SDHC/SDXC")
            print("[SDCard] sectors:", self.sectors)
            # SDHC/SDXC always uses 512-byte logical blocks. Do NOT send CMD16.

        elif csd_type == 0x00:  # CSD version 1.0
            c_size = (csd[6] & 0b11) << 10 | csd[7] << 2 | csd[8] >> 6
            c_size_mult = (csd[9] & 0b11) << 1 | csd[10] >> 7
            read_bl_len = csd[5] & 0b1111
            native_block_size = 1 << read_bl_len
            capacity = (c_size + 1) * (2 ** (c_size_mult + 2)) * native_block_size
            self.sectors = capacity // 512

            print("[SDCard] CSD v1 -> SDSC-style CSD")
            print("[SDCard] sectors:", self.sectors)
            print("[SDCard] native block size:", native_block_size)

            # If the card already reports 512-byte native blocks, no CMD16 is needed.
            if native_block_size == 512:
                print("[SDCard] native block is already 512 bytes; skipping CMD16")
            else:
                r16 = self.cmd(16, 512)
                if r16 != 0:
                    # A v2 card returning ILLEGAL_COMMAND for CMD16 behaves like
                    # SDHC/SDXC (fixed 512-byte logical blocks). Some cards/modules
                    # have been observed to give inconsistent OCR/CSD reporting.
                    if self.version2 and (r16 & _R1_ILLEGAL_COMMAND):
                        print("[SDCard] WARNING: CMD16 illegal on v2 card")
                        print("[SDCard] falling back to block addressing")
                        self.cdv = 1
                    else:
                        raise OSError("can't set 512 block size on SDSC card (CMD16={})".format(r16))
        else:
            raise OSError("SD card CSD format not supported")

        # set to high data rate now that it's initialised
        self.init_spi(baudrate)

    def init_card_v1(self):
        for i in range(_CMD_TIMEOUT):
            time.sleep_ms(50)
            self.cmd(55, 0)
            if self.cmd(41, 0) == 0:
                # SDSC card, uses byte addressing in read/write/erase commands
                self.cdv = 512
                # print("[SDCard] v1 card")
                return
        raise OSError("timeout waiting for v1 card")

    def init_card_v2(self):
        for i in range(_CMD_TIMEOUT):
            time.sleep_ms(50)
            self.cmd(58, 0, 4)
            self.cmd(55, 0)
            if self.cmd(41, 0x40000000) == 0:
                self.cmd(58, 0, -4)  # 4-byte response, negative means keep the first byte
                ocr = self.tokenbuf[0]  # get first byte of response, which is OCR
                if not ocr & 0x40:
                    # SDSC card, uses byte addressing in read/write/erase commands
                    self.cdv = 512
                else:
                    # SDHC/SDXC card, uses block addressing in read/write/erase commands
                    self.cdv = 1
                # print("[SDCard] v2 card")
                return
        raise OSError("timeout waiting for v2 card")

    def cmd(self, cmd, arg, final=0, release=True, skip1=False):
        self.cs(0)

        # create and send the command
        buf = self.cmdbuf
        buf[0] = 0x40 | cmd
        buf[1] = arg >> 24
        buf[2] = arg >> 16
        buf[3] = arg >> 8
        buf[4] = arg
        buf[5] = _crc7(buf, 5) | 0x01  # ensure stop bit is always set
        self.spi.write(buf)

        if skip1:
            self.spi.readinto(self.tokenbuf, 0xFF)

        # wait for the response (response[7] == 0)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xFF)
            response = self.tokenbuf[0]
            if not (response & 0x80):
                # this could be a big-endian integer that we are getting here
                # if final<0 then store the first byte to tokenbuf and discard the rest
                if final < 0:
                    self.spi.readinto(self.tokenbuf, 0xFF)
                    final = -1 - final
                for j in range(final):
                    self.spi.write(b"\xff")
                if release:
                    self.cs(1)
                    self.spi.write(b"\xff")
                return response

        # timeout
        self.cs(1)
        self.spi.write(b"\xff")
        return -1

    def readinto(self, buf):
        self.cs(0)

        # read until the data start token, with a hard time deadline
        deadline = time.ticks_add(time.ticks_ms(), _READ_TOKEN_TIMEOUT_MS)
        polls = 0
        while True:
            self.spi.readinto(self.tokenbuf, 0xFF)
            token = self.tokenbuf[0]
            if token == _TOKEN_DATA:
                break
            if token != 0xFF and not (token & 0xF0):
                # data error token (0000xxxx): card refused the read
                self.cs(1)
                self.spi.write(b"\xff")
                raise OSError(5)  # EIO
            if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
                self.cs(1)
                self.spi.write(b"\xff")
                raise OSError("timeout waiting for response")
            polls += 1
            if polls > 8:
                time.sleep_ms(1)

        # read data
        mv = self.dummybuf_memoryview
        if len(buf) != len(mv):
            mv = mv[: len(buf)]
        self.spi.write_readinto(mv, buf)

        # read checksum
        self.spi.write(b"\xff")
        self.spi.write(b"\xff")

        self.cs(1)
        self.spi.write(b"\xff")

    def _wait_not_busy(self, timeout_ms=_WRITE_BUSY_TIMEOUT_MS):
        """Wait for the SD busy token with a hard deadline.

        The stock MicroPython driver uses an unbounded while-loop here. If a
        card/module remains busy or MISO is stuck LOW, that loop owns the VM
        forever and looks exactly like a frozen ESP32.
        """
        deadline = time.ticks_add(
            time.ticks_ms(),
            int(timeout_ms),
        )

        while self.spi.read(1, 0xFF)[0] == 0x00:
            if time.ticks_diff(
                time.ticks_ms(),
                deadline,
            ) >= 0:
                raise OSError(
                    "SD write busy timeout after {} ms".format(
                        timeout_ms
                    )
                )
            time.sleep_ms(1)

        return True


    def write(self, token, buf):
        self.cs(0)

        # send: start of block, data, checksum
        self.spi.read(1, token)
        self.spi.write(buf)
        self.spi.write(b"\xff")
        self.spi.write(b"\xff")

        # check the response
        if (self.spi.read(1, 0xFF)[0] & 0x1F) != 0x05:
            self.cs(1)
            self.spi.write(b"\xff")
            return

        # wait for write to finish with a HARD deadline
        try:
            self._wait_not_busy()
        finally:
            self.cs(1)
            self.spi.write(b"\xff")

    def write_token(self, token):
        self.cs(0)
        self.spi.read(1, token)
        self.spi.write(b"\xff")
        # wait for write to finish with a HARD deadline
        try:
            self._wait_not_busy()
        finally:
            self.cs(1)
            self.spi.write(b"\xff")

    def _resync(self):
        """Drain whatever the card may still be sending after a failed read
        and leave the bus idle, so the next command starts in step."""
        try:
            self.cs(1)
            self.spi.write(b"\xff")
            self.cs(0)
            junk = getattr(self, "_junk", None)
            if junk is None:
                junk = self._junk = bytearray(64)
            for _ in range(9):          # > one 512-byte block + CRC
                self.spi.readinto(junk, 0xFF)
            self.cs(1)
            self.spi.write(self.dummybuf_memoryview[:16])
        except Exception:
            try:
                self.cs(1)
            except Exception:
                pass

    def readblocks(self, block_num, buf):
        last = None
        for attempt in range(_READ_RETRIES):
            try:
                return self._readblocks_once(block_num, buf)
            except OSError as e:
                last = e
                if len(buf) > 512:
                    try:
                        self.cmd(12, 0, skip1=True)   # stop a CMD18 stream
                    except Exception:
                        pass
                self._resync()
        raise last

    def _readblocks_once(self, block_num, buf):
        # workaround for shared bus, required for (at least) some Kingston
        # devices, ensure MOSI is high before starting transaction
        self.spi.write(b"\xff")

        nblocks = len(buf) // 512
        assert nblocks and not len(buf) % 512, "Buffer length is invalid"
        if nblocks == 1:
            # CMD17: set read address for single block
            if self.cmd(17, block_num * self.cdv, release=False) != 0:
                # release the card
                self.cs(1)
                raise OSError(5)  # EIO
            # receive the data and release card
            self.readinto(buf)
        else:
            # CMD18: set read address for multiple blocks
            if self.cmd(18, block_num * self.cdv, release=False) != 0:
                # release the card
                self.cs(1)
                raise OSError(5)  # EIO
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                # receive the data and release card
                self.readinto(mv[offset : offset + 512])
                offset += 512
                nblocks -= 1
            if self.cmd(12, 0, skip1=True):
                raise OSError(5)  # EIO

    def writeblocks(self, block_num, buf):
        # workaround for shared bus, required for (at least) some Kingston
        # devices, ensure MOSI is high before starting transaction
        self.spi.write(b"\xff")

        nblocks, err = divmod(len(buf), 512)
        assert nblocks and not err, "Buffer length is invalid"
        if nblocks == 1:
            # CMD24: set write address for single block
            if self.cmd(24, block_num * self.cdv) != 0:
                raise OSError(5)  # EIO

            # send the data
            self.write(_TOKEN_DATA, buf)
        else:
            # CMD25: set write address for first block
            if self.cmd(25, block_num * self.cdv) != 0:
                raise OSError(5)  # EIO
            # send the data
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                self.write(_TOKEN_CMD25, mv[offset : offset + 512])
                offset += 512
                nblocks -= 1
            self.write_token(_TOKEN_STOP_TRAN)

    def ioctl(self, op, arg):
        if op == 4:  # get number of blocks
            return self.sectors
        if op == 5:  # get block size in bytes
            return 512