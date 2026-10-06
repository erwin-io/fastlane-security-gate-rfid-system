"""Server-decided RFID access (v2.5.0).

The web service is the only authority: every tap is sent to
    POST http://<server>/api/turnstile/tap      APIKey: <key>
    {"rfid_uid": "0007212908", "gate_id": "<24 hex>", "direction": "entry",
     "tap_id": "<boot>-<n>"}
and the gate opens only when the reply says  data.access_result == "granted".
There is no card list on the machine any more.

Speed:
  * A WARM keep-alive connection to the server is held open while idle, so
    a tap costs one request/response - no TCP handshake (saved ~80 ms on
    the site LAN). It is refreshed before the server's keep-alive timeout
    and re-opened automatically (also after a W5500 reset).
  * Everything is non-blocking: request() hands the bytes to the W5500 and
    returns; service() is one short step per main-loop pass.
  * tap_id makes a re-send safe: if the connection dies after the request
    left (W5500 reset, keep-alive closed by the server at the same moment),
    the tap is re-sent on a fresh connection and the server answers the same
    decision again instead of logging a second tap.

Fail-closed: no answer within decision_timeout_ms -> the result is
granted=False with reason "server_unreachable"/"timeout".
"""
import time

try:
    import ujson as json
except ImportError:
    import json

try:
    from components.w5500 import NOTIFY_SOCKET, ChipResetError
except ImportError:  # pragma: no cover
    NOTIFY_SOCKET = 1

    class ChipResetError(OSError):
        pass

OFF = "OFF"
IDLE = "IDLE"              # no connection
CONNECTING = "CONNECTING"
READY = "READY"            # warm connection, no request in flight
WAITING = "WAITING"        # request sent, reading the reply

WARM_MAX_AGE_MS = 45000    # refresh before the server's keep-alive timeout
RECONNECT_BACKOFF_MS = 1000
READY_PROBE_MS = 100
WARM_STALL_MS = 900      # warm socket silent this long -> fresh socket + resend


def parse_http_url(url):
    url = str(url).strip()
    if not url.startswith("http://"):
        raise ValueError("Only http:// URLs are supported")
    rest = url[7:]
    slash = rest.find("/")
    hostport, path = (rest, "/") if slash < 0 else (rest[:slash], rest[slash:])
    if ":" in hostport:
        host, port_text = hostport.rsplit(":", 1)
        port = int(port_text)
    else:
        host, port = hostport, 80
    if not host:
        raise ValueError("Missing HTTP host")
    return host, port, path


class TapClient:
    def __init__(self, eth=None):
        self.eth = eth
        self.enabled = False
        self.url = ""
        self.host = ""
        self.port = 80
        self.path = "/"
        self.ip4 = None
        self.api_key = ""
        self.api_key_header = "APIKey"
        self.gate_id = ""
        self.direction = "entry"
        self.decision_timeout_ms = 2500
        self.keep_warm = True

        self.state = IDLE
        self.conn = None
        self.conn_since = 0
        self.connect_deadline = 0
        self.next_connect_at = 0
        self.last_probe = 0

        self.pending = None        # dict while a decision is outstanding
        self.result = None         # finished decision, taken by the caller
        self.rx = bytearray()
        self.out = b""
        self.out_sent = 0

        self.stats = {"requests": 0, "granted": 0, "denied": 0, "failed": 0,
                      "resent": 0, "reconnects": 0, "warm_hits": 0}
        self.last_ms = 0
        self.last_error = ""

    # ------------------------------------------------------------ config
    def configure(self, server_cfg, eth=None):
        if eth is not None:
            self.eth = eth
        self._drop()
        self.enabled = bool(server_cfg.get("enabled", True))
        self.url = str(server_cfg.get("tap_url", "")).strip()
        self.api_key = str(server_cfg.get("api_key", "")).strip()
        self.api_key_header = str(server_cfg.get("api_key_header", "APIKey")).strip() or "APIKey"
        self.gate_id = str(server_cfg.get("gate_id", "")).strip()
        self.direction = str(server_cfg.get("direction", "entry")).strip() or "entry"
        self.keep_warm = bool(server_cfg.get("keep_warm", True))
        try:
            self.decision_timeout_ms = max(500, min(10000, int(server_cfg.get("decision_timeout_ms", 2500))))
        except Exception:
            self.decision_timeout_ms = 2500
        self.ip4 = None
        try:
            self.host, self.port, self.path = parse_http_url(self.url)
            if self.eth is not None:
                self.ip4 = self.eth.resolve(self.host)
        except Exception as e:
            self.last_error = "tap URL: " + repr(e)
        self.state = IDLE if self.enabled else OFF

    def ready_reason(self):
        if not self.enabled:
            return "server integration disabled"
        if not self.url or self.ip4 is None:
            return "tap URL is empty or invalid"
        if not self.gate_id:
            return "gate ID is empty"
        if not self.api_key:
            return "API key is empty"
        if self.eth is None or not self.eth.ready:
            return "Ethernet not ready"
        return ""

    # ------------------------------------------------------------ public
    def busy(self):
        return self.pending is not None

    def request(self, card_id, tap_id, link_up=True):
        """Start a decision. Returns False (and sets .result) if impossible."""
        now = time.ticks_ms()
        reason = self.ready_reason()
        self.result = None
        self.pending = {
            "card": str(card_id), "tap_id": str(tap_id), "started": now,
            "deadline": time.ticks_add(now, self.decision_timeout_ms),
            "attempts": 0, "sent": False,
        }
        self.stats["requests"] += 1
        if reason:
            self._finish(False, "not_configured", error=reason)
            return False
        if not link_up:
            self._finish(False, "server_unreachable", error="Ethernet link down")
            return False
        body = json.dumps({
            "rfid_uid": str(card_id),
            "gate_id": self.gate_id,
            "direction": self.direction,
            "tap_id": str(tap_id),
        })
        head = (
            "POST {} HTTP/1.1\r\n"
            "Host: {}\r\n"
            "User-Agent: FastlaneESP32/2.5\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: {}\r\n"
            "Connection: keep-alive\r\n"
            "{}: {}\r\n\r\n"
        ).format(self.path, self.host if self.port == 80 else "{}:{}".format(self.host, self.port),
                 len(body), self.api_key_header, self.api_key)
        self.out = (head + body).encode()
        if self.state == READY:
            self.stats["warm_hits"] += 1
            self.pending["warm"] = True
            self._send_request()
        elif self.state in (IDLE, OFF):
            self._connect()
        # CONNECTING: the request goes out as soon as the socket is up.
        return True

    def take_result(self):
        r = self.result
        self.result = None
        return r

    def service(self, link_up=True):
        """One short non-blocking step (call every loop pass)."""
        if self.state == OFF:
            return
        now = time.ticks_ms()
        p = self.pending
        if p is not None and time.ticks_diff(now, p["deadline"]) >= 0:
            self._drop()
            self._finish(False, "timeout", error="no reply in %d ms" % self.decision_timeout_ms)
            return
        try:
            if self.state == IDLE:
                if p is not None:
                    if link_up:
                        self._connect()
                elif self.keep_warm and link_up and self.ready_reason() == "" and \
                        time.ticks_diff(now, self.next_connect_at) >= 0:
                    self._connect()
            elif self.state == CONNECTING:
                if self.conn.poll_connect():
                    self.state = READY
                    self.conn_since = now
                    if p is not None:
                        self._send_request()
                elif time.ticks_diff(now, self.connect_deadline) >= 0:
                    raise OSError("connect timeout")
            elif self.state == READY:
                if p is not None:
                    self._send_request()
                elif time.ticks_diff(now, self.last_probe) >= READY_PROBE_MS:
                    self.last_probe = now
                    d = self.conn.recv(64)
                    if d == b"":
                        self._drop()            # server closed the warm socket
                        self.stats["reconnects"] += 1
                    elif time.ticks_diff(now, self.conn_since) > WARM_MAX_AGE_MS:
                        self._drop()            # refresh before keep-alive expiry
                        self.stats["reconnects"] += 1
            elif self.state == WAITING:
                self._read_reply()
                p = self.pending
                if p is not None and self.state == WAITING and p.get("warm") and not self.rx and \
                        time.ticks_diff(time.ticks_ms(), p["sent_at"]) >= WARM_STALL_MS:
                    # A reused keep-alive socket that stays silent is most likely
                    # half-open (server restarted). Re-send on a fresh socket.
                    p["warm"] = False
                    self._attempt_failed("warm connection silent")
        except ChipResetError:
            self._attempt_failed("W5500 reset")
        except Exception as e:
            self._attempt_failed(repr(e))

    def status(self):
        s = dict(self.stats)
        s.update({
            "state": self.state,
            "ready": self.ready_reason() == "",
            "not_ready_reason": self.ready_reason(),
            "url": self.url,
            "gate_id": self.gate_id,
            "direction": self.direction,
            "decision_timeout_ms": self.decision_timeout_ms,
            "last_ms": self.last_ms,
            "last_error": self.last_error,
        })
        return s

    # ------------------------------------------------------------ internals
    def _connect(self):
        if self.ip4 is None:
            raise OSError("tap URL invalid")
        self._drop()
        self.conn = self.eth.open_conn(self.ip4, self.port, slot=NOTIFY_SOCKET)
        self.state = CONNECTING
        self.connect_deadline = time.ticks_add(time.ticks_ms(), 2000)

    def _send_request(self):
        p = self.pending
        p["attempts"] += 1
        if p["attempts"] > 1:
            self.stats["resent"] += 1
        self.rx = bytearray()
        view = memoryview(self.out)
        sent = 0
        guard = 0
        while sent < len(self.out):
            n = self.conn.send(view[sent:])
            if n:
                sent += n
            else:
                guard += 1
                if guard > 200:
                    raise OSError("send stalled")
        p["sent"] = True
        p["sent_at"] = time.ticks_ms()
        self.state = WAITING

    def _read_reply(self):
        d = self.conn.recv(1024)
        if d is None:
            return
        if d == b"":
            raise OSError("server closed the connection before replying")
        self.rx.extend(d)
        end = self.rx.find(b"\r\n\r\n")
        if end < 0:
            return
        head = bytes(self.rx[:end]).decode("utf-8", "ignore").split("\r\n")
        try:
            status = int(head[0].split(" ", 2)[1])
        except Exception:
            status = 0
        length = 0
        close = False
        for line in head[1:]:
            k, _, v = line.partition(":")
            k = k.strip().lower()
            if k == "content-length":
                try:
                    length = int(v.strip())
                except Exception:
                    length = 0
            elif k == "connection" and v.strip().lower() == "close":
                close = True
        if len(self.rx) - (end + 4) < length:
            return                       # body not complete yet
        body = bytes(self.rx[end + 4:end + 4 + length])
        self.rx = bytearray()
        if close:
            self._drop()
        else:
            self.state = READY
            self.conn_since = time.ticks_ms()
        granted = False
        reason = "http_%d" % status
        full_name = None
        if 200 <= status < 300:
            try:
                data = json.loads(body).get("data") or {}
                granted = data.get("access_result") == "granted"
                reason = data.get("reason") or ("granted" if granted else "denied")
                full_name = data.get("full_name")
            except Exception as e:
                reason = "bad_reply"
                self.last_error = "reply: " + repr(e)
        elif status in (401, 403):
            self.last_error = "HTTP %d - API key rejected" % status
        elif status in (400, 422):
            self.last_error = "HTTP %d - check gate ID / direction" % status
        else:
            self.last_error = "HTTP %d" % status
        self._finish(granted, reason, http_status=status, full_name=full_name)

    def _attempt_failed(self, message):
        self.last_error = message
        self._drop()
        self.next_connect_at = time.ticks_add(time.ticks_ms(), RECONNECT_BACKOFF_MS)
        p = self.pending
        if p is None:
            return
        # Re-send on a fresh connection while there is time (tap_id makes it
        # idempotent on the server).
        remaining = time.ticks_diff(p["deadline"], time.ticks_ms())
        if p["attempts"] < 3 and remaining > 200:
            self.next_connect_at = 0
            try:
                self._connect()
            except Exception as e:
                self.last_error = repr(e)
            return
        self._finish(False, "server_unreachable", error=message)

    def _finish(self, granted, reason, http_status=0, full_name=None, error=None):
        p = self.pending
        self.pending = None
        if error:
            self.last_error = error
        ms = time.ticks_diff(time.ticks_ms(), p["started"]) if p else 0
        self.last_ms = ms
        if granted:
            self.stats["granted"] += 1
        elif http_status and 200 <= http_status < 300:
            self.stats["denied"] += 1
        else:
            self.stats["failed"] += 1
        self.result = {
            "card": p["card"] if p else "",
            "tap_id": p["tap_id"] if p else "",
            "granted": bool(granted),
            "reason": reason,
            "full_name": full_name,
            "http_status": http_status,
            "ms": ms,
            "attempts": p["attempts"] if p else 0,
            "error": self.last_error if not granted and not (200 <= http_status < 300) else "",
        }

    def _drop(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
        self.conn = None
        self.rx = bytearray()
        if self.state != OFF:
            self.state = IDLE
