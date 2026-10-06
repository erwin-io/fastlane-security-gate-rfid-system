"""W5500 Ethernet - direct-register driver using the chip's hardware TCP/IP.

v2.4.1. Replaces ESP-IDF's network.LAN W5500 driver.

Why: on the gate, the NEMA/solenoid supply dips reset the W5500. After such a
reset the chip forgets its configuration (MAC register reads 00:00:00:00:00:00,
socket 0 closed), but ESP-IDF's MACRAW driver never notices: the link still
reports UP while no frame is sent or received, and network.LAN offers no way
to re-initialise without restarting the ESP32. Measured on the gate: requests
worked after a cold boot and failed 8/8 after the first open/close cycles.

This driver owns the chip directly over SPI and:
  * uses the W5500's own TCP/IP engine (8 hardware sockets) - no interrupt
    line, no lwIP; every state is polled, so nothing can be "missed";
  * checks the chip's configuration block (gateway, mask, MAC, IP) before
    every connection and every HEALTH_PERIOD_MS while idle; a mismatch means
    the chip was reset -> the configuration is written back (well under a
    millisecond, no ESP32 restart, no gate interruption) and every socket
    opened before the reset is reported stale so callers retry;
  * keeps the old public API (initialize, ready, is_connected, ifconfig,
    append_query, url_encode_component, parse_http_url, http_request) so the
    sync code is unchanged, and adds non-blocking connections (open_conn) for
    the tap notifier and a hardware listener for the Ethernet web page.

On a host PC (simulator / unit tests, sys.platform != "esp32") the same API is
served by ordinary sockets, so the firmware logic can be tested off-target.

Socket plan: 0,2,3 = HTTP client pool (sync, server test) | 1 = tap notifier
| 6,7 = Ethernet web page listeners (configuration mode only).
"""
from machine import Pin, SPI
import sys
import time

try:
    import ujson as json
except ImportError:
    import json

try:
    import socket as _socket
except ImportError:  # pragma: no cover
    _socket = None

HOST_MODE = sys.platform != "esp32"

# ---------------------------------------------------------------- registers
_COMMON = 0
MR = 0x0000
GAR = 0x0001            # GAR(4) SUBR(4) SHAR(6) SIPR(4) = 18 bytes from 0x0001
RTR = 0x0019
RCR = 0x001B
PHYCFGR = 0x002E
VERSIONR = 0x0039
W5500_VERSION = 0x04

Sn_MR = 0x00
Sn_CR = 0x01
Sn_IR = 0x02
Sn_SR = 0x03
Sn_PORT = 0x04
Sn_DIPR = 0x0C
Sn_DPORT = 0x10
Sn_TX_FSR = 0x20
Sn_TX_WR = 0x24
Sn_RX_RSR = 0x26
Sn_RX_RD = 0x28
Sn_KPALVTR = 0x2F

CMD_OPEN = 0x01
CMD_LISTEN = 0x02
CMD_CONNECT = 0x04
CMD_DISCON = 0x08
CMD_CLOSE = 0x10
CMD_SEND = 0x20
CMD_RECV = 0x40

SR_CLOSED = 0x00
SR_INIT = 0x13
SR_LISTEN = 0x14
SR_SYNSENT = 0x15
SR_ESTABLISHED = 0x17
SR_CLOSE_WAIT = 0x1C

IR_CON = 0x01
IR_DISCON = 0x02
IR_RECV = 0x04
IR_TIMEOUT = 0x08
IR_SENDOK = 0x10

MODE_TCP_NODELAY = 0x21      # TCP + ND (no delayed ACK)
EAGAIN = 11

HTTP_SOCKETS = (0, 2, 3)
NOTIFY_SOCKET = 1
WEB_SOCKETS = (6, 7)
HEALTH_PERIOD_MS = 200
# Retransmission: 100 ms x 4 tries -> ARP/SYN failures surface in ~1.5 s.
RTR_100US = 1000
RCR_RETRIES = 4


class ChipResetError(OSError):
    """The W5500 was reset while this connection was open."""


def _ip_bytes(text):
    parts = str(text).strip().split(".")
    if len(parts) != 4:
        raise ValueError("not an IPv4 address: " + str(text))
    values = [int(p) for p in parts]
    for v in values:
        if v < 0 or v > 255:
            raise ValueError("not an IPv4 address: " + str(text))
    return bytes(values)


def _is_ip(text):
    try:
        _ip_bytes(text)
        return True
    except Exception:
        return False


def _would_block(err):
    code = err.args[0] if getattr(err, "args", None) else None
    return code in (11, 35, 115, 119) or err.__class__.__name__ == "BlockingIOError"


# ======================================================================
# Chip access (SPI, variable-length data mode, CS by GPIO)
# ======================================================================

class W5500Chip:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self._hdr = bytearray(3)

    def read(self, addr, bsb, n):
        h = self._hdr
        h[0] = (addr >> 8) & 0xFF
        h[1] = addr & 0xFF
        h[2] = (bsb << 3)
        self.cs(0)
        try:
            self.spi.write(h)
            return self.spi.read(n)
        finally:
            self.cs(1)

    def write(self, addr, bsb, data):
        h = self._hdr
        h[0] = (addr >> 8) & 0xFF
        h[1] = addr & 0xFF
        h[2] = (bsb << 3) | 0x04
        self.cs(0)
        try:
            self.spi.write(h)
            self.spi.write(data)
        finally:
            self.cs(1)


# ======================================================================
# One hardware socket
# ======================================================================

class HwSocket:
    def __init__(self, eth, n):
        self.eth = eth
        self.n = n
        self.reg = n * 4 + 1
        self.txb = n * 4 + 2
        self.rxb = n * 4 + 3
        self.generation = -1
        self.send_pending = False

    def _r8(self, a):
        return self.eth.chip.read(a, self.reg, 1)[0]

    def _w8(self, a, v):
        self.eth.chip.write(a, self.reg, bytes((v & 0xFF,)))

    def _r16(self, a):
        # 16-bit counters change while being read: read until two agree.
        last = -1
        for _ in range(4):
            b = self.eth.chip.read(a, self.reg, 2)
            v = (b[0] << 8) | b[1]
            if v == last:
                return v
            last = v
        return last

    def _w16(self, a, v):
        self.eth.chip.write(a, self.reg, bytes(((v >> 8) & 0xFF, v & 0xFF)))

    def command(self, c):
        self._w8(Sn_CR, c)
        for _ in range(200):
            if self._r8(Sn_CR) == 0:
                return True
        return False

    def status(self):
        return self._r8(Sn_SR)

    def interrupts(self):
        return self._r8(Sn_IR)

    def clear(self, mask=0x1F):
        self._w8(Sn_IR, mask)

    @property
    def stale(self):
        return self.generation != self.eth.generation

    def open_tcp(self, local_port):
        if self.status() != SR_CLOSED:
            self.command(CMD_CLOSE)
        self.clear()
        self._w8(Sn_MR, MODE_TCP_NODELAY)
        self._w16(Sn_PORT, local_port)
        self._w8(Sn_KPALVTR, 0)
        self.command(CMD_OPEN)
        self.send_pending = False
        self.generation = self.eth.generation
        return self.status() == SR_INIT

    def connect(self, ip4, port):
        self.eth.chip.write(Sn_DIPR, self.reg, ip4)
        self._w16(Sn_DPORT, port)
        self.command(CMD_CONNECT)

    def listen(self, port):
        if not self.open_tcp(port):
            return False
        self.command(CMD_LISTEN)
        return self.status() == SR_LISTEN

    def close(self):
        try:
            self.command(CMD_CLOSE)
            self.clear()
        except Exception:
            pass
        self.send_pending = False

    def disconnect(self):
        try:
            self.command(CMD_DISCON)
        except Exception:
            pass

    def send(self, data):
        """Queue up to the free TX space; None = try again later."""
        if self.send_pending:
            ir = self.interrupts()
            if ir & IR_SENDOK:
                self.clear(IR_SENDOK)
                self.send_pending = False
            elif ir & IR_TIMEOUT:
                raise OSError(110)
            elif self.status() not in (SR_ESTABLISHED, SR_CLOSE_WAIT):
                raise OSError(104)
            else:
                return None
        free = self._r16(Sn_TX_FSR)
        if free <= 0:
            return None
        n = min(len(data), free)
        ptr = self._r16(Sn_TX_WR)
        self.eth.chip.write(ptr, self.txb, data[:n])
        self._w16(Sn_TX_WR, (ptr + n) & 0xFFFF)
        self.command(CMD_SEND)
        self.send_pending = True
        return n

    def recv(self, maxn):
        size = self._r16(Sn_RX_RSR)
        if size <= 0:
            return None
        n = min(size, maxn)
        ptr = self._r16(Sn_RX_RD)
        data = self.eth.chip.read(ptr, self.rxb, n)
        self._w16(Sn_RX_RD, (ptr + n) & 0xFFFF)
        self.command(CMD_RECV)
        return bytes(data)


# ======================================================================
# Connections (same interface on hardware and on a host PC)
# ======================================================================

class HwConn:
    """Non-blocking TCP client connection on one W5500 socket."""

    def __init__(self, eth, sock):
        self.eth = eth
        self.sock = sock

    def start(self, ip4, port):
        self.eth.ensure_healthy()
        if not self.sock.open_tcp(self.eth.next_port()):
            raise OSError("W5500 socket %d did not open" % self.sock.n)
        self.sock.connect(ip4, port)

    def _check(self):
        if self.sock.stale:
            raise ChipResetError("W5500 reset during the request")
        if self.sock.eth.suspended():
            raise OSError(113)

    def poll_connect(self):
        """True once connected; False while connecting; raises on failure."""
        self._check()
        sr = self.sock.status()
        if sr == SR_ESTABLISHED or sr == SR_CLOSE_WAIT:
            return True
        ir = self.sock.interrupts()
        if sr == SR_CLOSED or (ir & IR_TIMEOUT):
            self.eth.check_health()
            self._check()
            if not self.eth.is_connected():
                raise OSError(113)
            # TIMEOUT = no ARP/SYN answer (unreachable/filtered); otherwise
            # the server actively refused (RST).
            raise OSError(110 if (ir & IR_TIMEOUT) else 111)
        return False

    def _closed_unexpectedly(self):
        # A socket that drops to CLOSED may mean the chip was reset: let the
        # health check decide, so the caller gets ChipResetError (= retry).
        self.eth.check_health()
        self._check()

    def send(self, data):
        self._check()
        try:
            return self.sock.send(data)
        except OSError:
            self._closed_unexpectedly()
            raise

    def recv(self, maxn=1024):
        """bytes, None (nothing yet) or b'' (peer closed, all read)."""
        self._check()
        data = self.sock.recv(maxn)
        if data:
            return data
        sr = self.sock.status()
        if sr == SR_CLOSE_WAIT:
            return b""
        if sr == SR_CLOSED:
            self._closed_unexpectedly()
            return b""
        if self.sock.interrupts() & IR_TIMEOUT:
            self._closed_unexpectedly()
            raise OSError(110)
        return None

    def close(self):
        try:
            if not self.sock.stale:
                self.sock.disconnect()
            self.sock.close()
        finally:
            self.eth.release(self.sock.n)


class LwipConn:
    """Host-PC stand-in (simulator, tests): the same calls over BSD sockets."""

    def __init__(self, eth, slot):
        self.eth = eth
        self.slot = slot
        self.s = None

    def start(self, ip4, port):
        addr = ".".join(str(b) for b in ip4)
        self.s = _socket.socket()
        self.s.setblocking(False)
        try:
            self.s.connect((addr, port))
        except OSError:
            pass

    def poll_connect(self):
        import select
        p = select.poll()
        p.register(self.s, select.POLLOUT)
        ev = p.poll(0)
        if not ev:
            return False
        err = 0
        try:
            err = self.s.getsockopt(_socket.SOL_SOCKET, _socket.SO_ERROR)
        except Exception:
            pass
        if err or (ev[0][1] & (8 | 16)):
            raise OSError(err or 111)
        return True

    def send(self, data):
        try:
            return self.s.send(data)
        except OSError as e:
            if _would_block(e):
                return None
            raise

    def recv(self, maxn=1024):
        try:
            return self.s.recv(maxn)
        except OSError as e:
            if _would_block(e):
                return None
            raise

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass
        self.eth.release(self.slot)


# ======================================================================
# Ethernet web page listener (hardware sockets 6/7)
# ======================================================================

class HwWebClient:
    def __init__(self, listener, sock):
        self.listener = listener
        self.sock = sock

    def setblocking(self, flag):
        pass

    def settimeout(self, t):
        pass

    def recv(self, n):
        if self.sock.stale or self.sock.eth.suspended():
            return b""
        data = self.sock.recv(n)
        if data:
            return data
        if self.sock.status() in (SR_CLOSE_WAIT, SR_CLOSED):
            return b""
        raise OSError(EAGAIN)

    def send(self, data):
        if self.sock.stale:
            raise ValueError("W5500 reset")
        n = self.sock.send(data)
        if n is None:
            raise OSError(EAGAIN)
        return n

    def close(self):
        self.listener._client_closed(self.sock)


class HwWebListener:
    """Acts like a non-blocking listening socket for CooperativeWebServer."""

    def __init__(self, eth, port=80, sockets=WEB_SOCKETS):
        self.eth = eth
        self.port = port
        self.socks = [HwSocket(eth, n) for n in sockets]
        self.handed = set()
        self.closing = {}

    def accept(self):
        if not self.eth.ready or self.eth.suspended():
            raise OSError(EAGAIN)
        now = time.ticks_ms()
        for s in self.socks:
            n = s.n
            if n in self.closing:
                if s.stale or s.status() == SR_CLOSED or \
                        time.ticks_diff(now, self.closing[n]) > 1000:
                    s.close()
                    del self.closing[n]
                else:
                    continue
            if n in self.handed:
                continue
            sr = s.status()
            if sr == SR_ESTABLISHED and not s.stale:
                self.handed.add(n)
                return HwWebClient(self, s), ("ethernet", n)
            if s.stale or sr not in (SR_LISTEN, SR_SYNSENT, SR_ESTABLISHED):
                s.listen(self.port)
        raise OSError(EAGAIN)

    def _client_closed(self, s):
        self.handed.discard(s.n)
        s.disconnect()
        self.closing[s.n] = time.ticks_ms()

    def close(self):
        for s in self.socks:
            s.close()
        self.handed.clear()
        self.closing.clear()


# ======================================================================
# Driver
# ======================================================================

class W5500Ethernet:
    def __init__(
        self,
        spi_id,
        sck_pin,
        mosi_pin,
        miso_pin,
        cs_pin,
        int_pin,
        rst_pin,
        baudrate=8000000,
    ):
        self.spi_id = spi_id
        self.sck_pin = sck_pin
        self.mosi_pin = mosi_pin
        self.miso_pin = miso_pin
        self.cs_pin = cs_pin
        self.int_pin = int_pin
        self.rst_pin = rst_pin
        self.baudrate = baudrate
        self.spi = None
        self.chip = None
        self.ready = False
        self.host_mode = HOST_MODE
        self.last_error = ""
        self.cfg = None
        self._image = b""            # GAR+SUBR+SHAR+SIPR as written
        self.mac = b""
        self.generation = 0          # +1 every time the chip had to be reconfigured
        self.recoveries = 0
        self.last_recovery = ""
        self._health_at = 0
        self._port = 49152 + (time.ticks_ms() % 8000)
        self._busy = set()
        self._socks = {}
        # v2.5.1 circuit breaker: a W5500 that stops answering (or keeps
        # resetting) is left alone for a growing back-off so it can never
        # use main-loop time; every network call fails fast meanwhile.
        self._suspend_until = 0
        self._suspend_ms = 0
        self._recovery_times = []
        self.suspensions = 0

    # ------------------------------------------------------------ setup
    def _make_mac(self):
        """ESP32 'Ethernet' MAC = base MAC + 3 (what ESP-IDF's driver used),
        so routers/PCs keep their ARP entry for the gate across upgrades."""
        try:
            import machine
            uid = bytes(machine.unique_id())
        except Exception:
            uid = b""
        if len(uid) < 6:
            uid = (uid + b"\x02\x34\x56\x78\x9a\xbc")[:6]
        value = 0
        for b in uid[:6]:
            value = (value << 8) | b
        value = (value + 3) & 0xFFFFFFFFFFFF
        out = bytearray(6)
        for i in range(5, -1, -1):
            out[i] = value & 0xFF
            value >>= 8
        out[0] &= 0xFE                    # always unicast
        return bytes(out)

    def _write_config(self):
        c = self.chip
        c.write(GAR, _COMMON, self._image)
        c.write(RTR, _COMMON, bytes(((RTR_100US >> 8) & 0xFF, RTR_100US & 0xFF)))
        c.write(RCR, _COMMON, bytes((RCR_RETRIES,)))

    def attach_chip(self, chip):
        """Use an already-built chip object (tests use a fake W5500)."""
        self.chip = chip
        self.host_mode = False
        for n in range(8):
            self._socks[n] = HwSocket(self, n)
        self._write_config()
        self.ready = True

    def initialize(self, eth_cfg):
        print()
        print("========================================")
        print("STARTING W5500 ETHERNET (HW TCP/IP, SELF-HEALING)")
        print("========================================")
        self.ready = False
        self.last_error = ""
        self.cfg = dict(eth_cfg)
        try:
            gw = _ip_bytes(eth_cfg["gateway"])
            mask = _ip_bytes(eth_cfg["subnet"])
            ip = _ip_bytes(eth_cfg["ip"])
            self.mac = self._make_mac()
            self._image = gw + mask + self.mac + ip

            if self.host_mode:
                self.ready = True
                print("W5500 HOST MODE: host sockets stand in for the chip")
                return True

            rst = Pin(self.rst_pin, Pin.OUT, value=1)
            rst.value(0)
            time.sleep_ms(2)
            rst.value(1)
            time.sleep_ms(60)

            cs = Pin(self.cs_pin, Pin.OUT, value=1)
            self.spi = SPI(
                self.spi_id,
                baudrate=self.baudrate,
                polarity=0,
                phase=0,
                sck=Pin(self.sck_pin),
                mosi=Pin(self.mosi_pin),
                miso=Pin(self.miso_pin),
            )
            self.chip = W5500Chip(self.spi, cs)

            version = self.chip.read(VERSIONR, _COMMON, 1)[0]
            if version != W5500_VERSION:
                raise OSError("W5500 not found (VERSIONR=0x%02X, check SPI wiring)" % version)

            self.chip.write(MR, _COMMON, b"\x80")         # software reset
            for _ in range(100):
                if not (self.chip.read(MR, _COMMON, 1)[0] & 0x80):
                    break
                time.sleep_ms(1)
            self._write_config()
            for n in range(8):
                self._socks[n] = HwSocket(self, n)
            self.ready = True
            print("W5500 ACTIVE (SPI %d MHz, hardware TCP/IP)" % (self.baudrate // 1000000))
            print("IP      :", eth_cfg["ip"])
            print("SUBNET  :", eth_cfg["subnet"])
            print("GATEWAY :", eth_cfg["gateway"])
            print("DNS     :", eth_cfg.get("dns", ""))
            print("MAC     :", ":".join("%02X" % b for b in self.mac))
            print("LINK    :", self.is_connected())
            return True
        except Exception as e:
            self.last_error = repr(e)
            print("W5500 ERROR:", self.last_error)
            self.ready = False
            return False

    # ------------------------------------------------------------ breaker
    def suspended(self):
        if not self._suspend_until:
            return False
        if time.ticks_diff(time.ticks_ms(), self._suspend_until) >= 0:
            return False
        return True

    def _suspend(self, reason):
        self._suspend_ms = min(30000, max(500, self._suspend_ms * 2))
        self._suspend_until = time.ticks_add(time.ticks_ms(), self._suspend_ms)
        self.suspensions += 1
        self.last_error = reason
        print("W5500 PAUSED {} ms: {} (gate keeps running)".format(self._suspend_ms, reason))

    def _resume_ok(self):
        if self._suspend_until:
            print("W5500 RESUMED")
        self._suspend_until = 0
        self._suspend_ms = 0

    # ------------------------------------------------------------ health
    def check_health(self):
        """Verify the chip still holds our configuration; restore it if not.

        Returns True when the chip is (again) configured. Cheap: one 18-byte
        SPI read when healthy.
        """
        if self.host_mode:
            return self.ready
        if self.chip is None:
            return False
        try:
            block = bytes(self.chip.read(GAR, _COMMON, 18))
            if block == self._image:
                self._resume_ok()
                return True
            version = self.chip.read(VERSIONR, _COMMON, 1)[0]
            if version != W5500_VERSION:
                self._suspend("not responding (VERSIONR=0x%02X)" % version)
                return False
            # Read again: one noisy read must not trigger a recovery.
            again = bytes(self.chip.read(GAR, _COMMON, 18))
            if again == self._image:
                return True
            t0 = time.ticks_us()
            self._write_config()
            self.generation += 1
            self.recoveries += 1
            dt = time.ticks_diff(time.ticks_us(), t0)
            lost_mac = again[8:14] != self.mac
            self.last_recovery = "chip reset detected ({}) -> reconfigured in {} us".format(
                "MAC lost" if lost_mac else "config changed", dt)
            print("W5500 RECOVERED #{}: {}".format(self.recoveries, self.last_recovery))
            ok = bytes(self.chip.read(GAR, _COMMON, 18)) == self._image
            now = time.ticks_ms()
            rt = [t for t in self._recovery_times if time.ticks_diff(now, t) < 3000]
            rt.append(now)
            self._recovery_times = rt
            if not ok:
                self._suspend("config did not stick after reset")
            elif len(rt) >= 3:
                self._recovery_times = []
                self._suspend("reset storm (3 resets in 3 s)")
            else:
                self.last_error = ""
            return ok
        except Exception as e:
            self._suspend("health check: " + repr(e))
            return False

    def service(self):
        """Call often from the main loop (cheap, rate-limited)."""
        if not self.ready or self.host_mode:
            return
        if self.suspended():
            return
        now = time.ticks_ms()
        if self._health_at and time.ticks_diff(now, self._health_at) < HEALTH_PERIOD_MS:
            return
        self._health_at = now
        self.check_health()

    def ensure_healthy(self):
        if not self.ready:
            raise OSError("W5500 is not initialized")
        if self.suspended():
            raise OSError(113)
        if not self.check_health():
            raise OSError(self.last_error or "W5500 not responding")
        if not self.is_connected():
            raise OSError(113)

    # ------------------------------------------------------------ status
    def is_connected(self):
        if not self.ready:
            return False
        if self.host_mode:
            return True
        if self.suspended():
            return False
        try:
            return bool(self.chip.read(PHYCFGR, _COMMON, 1)[0] & 0x01)
        except Exception:
            return False

    def ifconfig(self):
        if not self.cfg:
            return None
        return (self.cfg["ip"], self.cfg["subnet"], self.cfg["gateway"], self.cfg.get("dns", ""))

    def status(self):
        return {
            "driver": "host sockets" if self.host_mode else "W5500 hardware TCP/IP (self-healing)",
            "recoveries": self.recoveries,
            "last_recovery": self.last_recovery,
            "paused": self.suspended(),
            "pauses": self.suspensions,
            "mac": ":".join("%02X" % b for b in self.mac) if self.mac else "",
        }

    # ------------------------------------------------------------ sockets
    def next_port(self):
        self._port += 1
        if self._port > 64999:
            self._port = 49152
        return self._port

    def release(self, n):
        self._busy.discard(n)

    def open_conn(self, ip4, port, slot=None):
        """Start a non-blocking TCP connection. slot=None -> HTTP pool."""
        if self.suspended():
            raise OSError(113)
        if slot is None:
            for n in HTTP_SOCKETS:
                if n not in self._busy:
                    slot = n
                    break
            if slot is None:
                raise OSError("no free W5500 socket")
        elif slot in self._busy:
            raise OSError("W5500 socket %d busy" % slot)
        self._busy.add(slot)
        try:
            conn = LwipConn(self, slot) if self.host_mode else HwConn(self, self._socks[slot])
            conn.start(ip4, port)
            return conn
        except Exception:
            self._busy.discard(slot)
            raise

    def web_listener(self, port=80):
        if self.host_mode or not self.ready:
            return None
        return HwWebListener(self, port)

    def resolve(self, host):
        if _is_ip(host):
            return _ip_bytes(host)
        if self.host_mode and _socket is not None:
            return _ip_bytes(_socket.getaddrinfo(host, 80)[0][-1][0])
        raise ValueError("use an IP address in the URL (no DNS on the W5500): " + str(host))

    # ------------------------------------------------------------ URL helpers
    @staticmethod
    def parse_http_url(url):
        url = str(url).strip()
        if not url.startswith("http://"):
            raise ValueError("Only http:// URLs are supported")
        remainder = url[7:]
        slash = remainder.find("/")
        if slash < 0:
            hostport = remainder
            path = "/"
        else:
            hostport = remainder[:slash]
            path = remainder[slash:]
        if ":" in hostport:
            host, port_text = hostport.rsplit(":", 1)
            port = int(port_text)
        else:
            host = hostport
            port = 80
        if not host:
            raise ValueError("Missing HTTP host")
        return host, port, path

    @staticmethod
    def url_encode_component(value):
        text = str(value)
        safe = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_.~"
        out = ""
        for ch in text:
            if ch in safe:
                out += ch
            elif ch == " ":
                out += "%20"
            else:
                code = ord(ch)
                if code < 256:
                    out += "%{:02X}".format(code)
                else:
                    out += "%3F"
        return out

    def append_query(self, url, params):
        pairs = []
        for key, value in params.items():
            if value is None or value == "":
                continue
            pairs.append(
                self.url_encode_component(key) + "=" + self.url_encode_component(value)
            )
        if not pairs:
            return url
        return url + ("&" if "?" in url else "?") + "&".join(pairs)

    @staticmethod
    def decode_chunked(data):
        out = bytearray()
        pos = 0
        length = len(data)
        while pos < length:
            line_end = data.find(b"\r\n", pos)
            if line_end < 0:
                break
            size_text = data[pos:line_end].split(b";", 1)[0]
            try:
                size = int(size_text, 16)
            except Exception:
                break
            pos = line_end + 2
            if size == 0:
                break
            if pos + size > length:
                break
            out.extend(data[pos:pos + size])
            pos += size + 2
        return bytes(out)

    @staticmethod
    def _service(service_callback):
        if service_callback is None:
            return
        try:
            service_callback()
        except Exception as e:
            print("W5500 SERVICE CALLBACK ERROR:", repr(e))

    @staticmethod
    def build_request(method, host, port, path, body=b"", headers=None):
        request = (
            "{} {} HTTP/1.1\r\n"
            "Host: {}\r\n"
            "User-Agent: FastlaneESP32/2.4\r\n"
            "Accept: application/json\r\n"
            "Connection: close\r\n"
        ).format(method, path, host if port == 80 else "{}:{}".format(host, port))
        if headers:
            for key, value in headers.items():
                key = str(key).strip()
                if not key or value is None:
                    continue
                value = str(value).replace("\r", "").replace("\n", "")
                request += "{}: {}\r\n".format(key, value)
        if body:
            request += "Content-Type: application/json\r\n"
            request += "Content-Length: {}\r\n".format(len(body))
        request += "\r\n"
        return request.encode() + (body or b"")

    # ------------------------------------------------------------ HTTP client
    def http_request(
        self,
        url,
        method="GET",
        json_body=None,
        timeout_ms=5000,
        max_body=131072,
        service_callback=None,
        headers=None,
        connect_timeout_ms=None,
    ):
        """Blocking-cooperative HTTP request: the caller's loop keeps running
        through service_callback. A W5500 reset in the middle is recovered and
        the request is sent once more automatically."""
        if not self.ready:
            raise OSError("W5500 is not initialized")
        method = str(method).upper()
        if method not in ("GET", "POST"):
            method = "GET"
        body = b"" if json_body is None else json.dumps(json_body).encode()
        host, port, path = self.parse_http_url(url)
        ip4 = self.resolve(host)
        payload = self.build_request(method, host, port, path, body, headers)
        try:
            return self._http_once(ip4, port, payload, timeout_ms, max_body, service_callback)
        except ChipResetError:
            print("W5500: request interrupted by a chip reset - sending again")
            return self._http_once(ip4, port, payload, timeout_ms, max_body, service_callback)

    def _http_once(self, ip4, port, payload, timeout_ms, max_body, service_callback):
        timeout_ms = max(100, int(timeout_ms))
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)

        def expired():
            return time.ticks_diff(deadline, time.ticks_ms()) <= 0

        conn = self.open_conn(ip4, port)
        try:
            while not conn.poll_connect():
                if expired():
                    raise OSError("HTTP connect timeout")
                self._service(service_callback)

            view = memoryview(payload)
            sent = 0
            while sent < len(payload):
                n = conn.send(view[sent:])
                if n:
                    sent += n
                    continue
                if expired():
                    raise OSError("HTTP send timeout")
                self._service(service_callback)

            raw = bytearray()
            header_end = -1
            while header_end < 0:
                chunk = conn.recv(1024)
                if chunk is None:
                    if expired():
                        raise OSError("HTTP receive timeout")
                    self._service(service_callback)
                    continue
                if not chunk:
                    break
                raw.extend(chunk)
                if len(raw) > 16384:
                    raise OSError("HTTP headers too large")
                header_end = raw.find(b"\r\n\r\n")
            if header_end < 0:
                raise OSError("Invalid HTTP response")

            lines = bytes(raw[:header_end]).split(b"\r\n")
            parts = lines[0].decode("utf-8", "ignore").split(" ", 2)
            status = int(parts[1]) if len(parts) > 1 else 0
            headers = {}
            for line in lines[1:]:
                colon = line.find(b":")
                if colon > 0:
                    key = line[:colon].decode("utf-8", "ignore").strip().lower()
                    headers[key] = line[colon + 1:].decode("utf-8", "ignore").strip()
            content_length = None
            try:
                if "content-length" in headers:
                    content_length = int(headers["content-length"])
            except Exception:
                content_length = None

            body = bytearray(raw[header_end + 4:])
            raw = None
            while content_length is None or len(body) < content_length:
                if len(body) > max_body:
                    raise OSError("HTTP response exceeds max_response_bytes")
                chunk = conn.recv(2048)
                if chunk is None:
                    if expired():
                        raise OSError("HTTP receive timeout")
                    self._service(service_callback)
                    continue
                if not chunk:
                    break
                body.extend(chunk)
            if len(body) > max_body:
                raise OSError("HTTP response exceeds max_response_bytes")
            out = bytes(body[:content_length] if content_length is not None else body)
            if headers.get("transfer-encoding", "").lower() == "chunked":
                out = self.decode_chunked(out)
            return status, headers, out
        finally:
            conn.close()
