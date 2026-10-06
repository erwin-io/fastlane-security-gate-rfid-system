"""Fire-and-forget tap notifications to the web service (v2.4.1).

Every decided RFID tap (GRANTED or DENIED) is POSTed to the turnstile API,
e.g. POST http://<server>/api/turnstile/tap
     APIKey: <key>
     {"rfid_uid": "0007212908", "gate_id": "<24 hex>", "direction": "entry"}
so the wall screen (/gate/turnstile/<gate_id>) shows the tap over its
WebSocket. The gate NEVER waits for it: the access decision is made locally
from the SD card database before anything is sent.

Everything here is non-blocking. enqueue() appends to a small queue and
starts the connection at once; service() is called on every main-loop pass
(also while the motors run) and does one short step: poll the connection,
hand the request to the W5500, or read the status line.

v2.4.1 delivery guarantee: the gate's motors/solenoid can reset the W5500 in
the middle of a request. The Ethernet driver detects that (and restores the
chip in < 1 ms); the notifier then simply sends the tap again. A tap is
retried until it is delivered or MAX_AGE_MS old (a "tap" shown much later on
the wall screen would be wrong). A request the server answered is never
re-sent, so the server logs every tap exactly once in normal operation.
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

IDLE = "IDLE"
CONNECTING = "CONNECTING"
SENDING = "SENDING"
WAITING = "WAITING"

RETRY_GAP_MS = 150


def parse_http_url(url):
    """Return (host, port, path) for an http:// URL."""
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


class TapNotifier:
    def __init__(self, queue_max=8, max_age_ms=10000, eth=None):
        self.eth = eth
        self.queue_max = int(queue_max)
        self.max_age_ms = int(max_age_ms)
        self.enabled = False
        self.url = ""
        self.api_key = ""
        self.api_key_header = "APIKey"
        self.gate_id = ""
        self.direction = "entry"
        self.timeout_ms = 3000

        self.queue = []          # [card_id, queued_at_ms, attempts]
        self.item = None
        self.state = IDLE
        self.conn = None
        self.payload = b""
        self.sent_bytes = 0
        self.deadline = 0
        self.rx = b""
        self.started_at = 0
        self.retry_at = 0

        self.sent = 0            # delivered (server answered, any status)
        self.accepted = 0        # delivered with HTTP 2xx
        self.failed = 0          # given up
        self.dropped = 0         # never sent (stale / queue full)
        self.retries = 0
        self.last_card = ""
        self.last_status = 0
        self.last_error = ""
        self.last_ms = 0

    # ------------------------------------------------------------------ config
    def configure(self, server_cfg, eth=None):
        if eth is not None:
            self.eth = eth
        self.enabled = bool(server_cfg.get("enabled", True)) and bool(
            server_cfg.get("tap_notify_enabled", True)
        )
        self.url = str(server_cfg.get("tap_url", "")).strip()
        self.api_key = str(server_cfg.get("api_key", "")).strip()
        self.api_key_header = str(server_cfg.get("api_key_header", "APIKey")).strip() or "APIKey"
        self.gate_id = str(server_cfg.get("gate_id", "")).strip()
        self.direction = str(server_cfg.get("direction", "entry")).strip() or "entry"
        try:
            self.timeout_ms = max(500, min(30000, int(server_cfg.get("timeout_ms", 3000))))
        except Exception:
            self.timeout_ms = 3000

    def ready_reason(self):
        """'' when a notification can be sent, otherwise why not."""
        if not self.enabled:
            return "disabled"
        if not self.url:
            return "tap URL is empty"
        if not self.gate_id:
            return "gate ID is empty"
        if self.eth is None:
            return "Ethernet not initialised"
        return ""

    # ------------------------------------------------------------------ public
    def enqueue(self, card_id, link_up=True):
        """Queue one tap. Never blocks; returns False if it was not queued."""
        reason = self.ready_reason()
        if reason:
            if reason != "disabled":
                self.last_error = reason
            return False
        self.queue.append([str(card_id), time.ticks_ms(), 0])
        while len(self.queue) > self.queue_max:
            self.queue.pop(0)
            self.dropped += 1
            self.last_error = "queue full - oldest tap dropped"
        # Start right away so the request overlaps the solenoid delay.
        self.service(link_up)
        return True

    def busy(self):
        return self.state != IDLE or bool(self.queue)

    def service(self, link_up=True):
        """Advance the current request by one non-blocking step."""
        if self.state == IDLE:
            self._start_next(link_up)
            return
        if time.ticks_diff(time.ticks_ms(), self.deadline) >= 0:
            self._attempt_failed("timeout in " + self.state)
            return
        try:
            if self.state == CONNECTING:
                if self.conn.poll_connect():
                    self.state = SENDING
                    self._send_some()
            elif self.state == SENDING:
                self._send_some()
            elif self.state == WAITING:
                self._read_status()
        except ChipResetError as e:
            self._attempt_failed("W5500 reset - resending")
        except Exception as e:
            self._attempt_failed(repr(e))

    def status(self):
        return {
            "enabled": self.enabled,
            "url": self.url,
            "gate_id": self.gate_id,
            "direction": self.direction,
            "ready": self.ready_reason() == "",
            "not_ready_reason": self.ready_reason(),
            "state": self.state,
            "queued": len(self.queue) + (1 if self.item else 0),
            "sent": self.sent,
            "accepted": self.accepted,
            "failed": self.failed,
            "dropped": self.dropped,
            "retries": self.retries,
            "last_card_id": self.last_card,
            "last_http_status": self.last_status,
            "last_error": self.last_error,
            "last_duration_ms": self.last_ms,
        }

    # ------------------------------------------------------------------ steps
    def _start_next(self, link_up):
        now = time.ticks_ms()
        if self.retry_at and time.ticks_diff(now, self.retry_at) < 0:
            return
        self.retry_at = 0
        while self.queue:
            if time.ticks_diff(now, self.queue[0][1]) <= self.max_age_ms:
                break
            card = self.queue.pop(0)[0]
            self.dropped += 1
            self.last_card = card
            self.last_error = "tap %s dropped after %d s (server unreachable?)" % (
                card, self.max_age_ms // 1000)
        if not self.queue:
            return
        if not link_up:
            return                      # wait for the link; the age limit drops it
        self.item = self.queue.pop(0)
        card_id = self.item[0]
        self.item[2] += 1
        if self.item[2] > 1:
            self.retries += 1
        self.last_card = card_id
        self.started_at = now
        self.deadline = time.ticks_add(now, self.timeout_ms)
        self.rx = b""
        self.sent_bytes = 0
        try:
            host, port, path = parse_http_url(self.url)
            body = json.dumps({
                "rfid_uid": card_id,
                "gate_id": self.gate_id,
                "direction": self.direction,
            })
            head = (
                "POST {} HTTP/1.1\r\n"
                "Host: {}\r\n"
                "User-Agent: FastlaneESP32/2.4\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: {}\r\n"
                "Connection: close\r\n"
            ).format(path, host if port == 80 else "{}:{}".format(host, port), len(body))
            if self.api_key:
                head += "{}: {}\r\n".format(self.api_key_header, self.api_key)
            self.payload = (head + "\r\n" + body).encode()
            ip4 = self.eth.resolve(host)
            self.conn = self.eth.open_conn(ip4, port, slot=NOTIFY_SOCKET)
            self.state = CONNECTING
            if self.conn.poll_connect():
                self.state = SENDING
                self._send_some()
        except ChipResetError:
            self._attempt_failed("W5500 reset - resending")
        except Exception as e:
            self._attempt_failed(repr(e))

    def _send_some(self):
        n = self.conn.send(memoryview(self.payload)[self.sent_bytes:])
        if n:
            self.sent_bytes += n
        if self.sent_bytes >= len(self.payload):
            self.state = WAITING

    def _read_status(self):
        # Only the status line is wanted; the body is never parsed.
        chunk = self.conn.recv(128)
        if chunk is None:
            return
        if chunk:
            self.rx += chunk
        if b"\r\n" in self.rx or not chunk:
            status = 0
            try:
                status = int(self.rx.split(b" ", 2)[1])
            except Exception:
                pass
            if status:
                self._finish(status)
            else:
                self._attempt_failed("connection closed before the HTTP status")

    def _finish(self, status):
        self.sent += 1
        self.last_status = status
        if 200 <= status < 300:
            self.accepted += 1
            self.last_error = ""
        elif status in (401, 403):
            self.last_error = "HTTP {} - check the API key".format(status)
        elif status in (400, 422):
            self.last_error = "HTTP {} - check gate ID / direction".format(status)
        else:
            self.last_error = "HTTP {}".format(status)
        self.item = None
        self._close()

    def _attempt_failed(self, message):
        """Close this attempt; requeue the tap at the front unless too old."""
        item = self.item
        self.item = None
        self._close()
        self.last_error = message
        if item is None:
            return
        age = time.ticks_diff(time.ticks_ms(), item[1])
        if age < self.max_age_ms:
            self.queue.insert(0, item)
            self.retry_at = time.ticks_add(time.ticks_ms(), RETRY_GAP_MS)
        else:
            self.failed += 1
            self.last_error = "tap %s not delivered: %s" % (item[0], message)

    def _close(self):
        self.last_ms = time.ticks_diff(time.ticks_ms(), self.started_at) if self.started_at else 0
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
        self.conn = None
        self.payload = b""
        self.rx = b""
        self.state = IDLE
