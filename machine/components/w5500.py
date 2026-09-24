from machine import Pin, SPI
import network
import socket
import time

try:
    import ujson as json
except ImportError:
    import json


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
        baudrate=20000000,
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
        self.lan = None
        self.ready = False
        self.last_error = ""

    def initialize(self, eth_cfg):
        print()
        print("========================================")
        print("STARTING W5500 ETHERNET")
        print("========================================")

        self.ready = False
        self.last_error = ""
        try:
            rst = Pin(self.rst_pin, Pin.OUT, value=1)
            rst.value(0)
            time.sleep_ms(100)
            rst.value(1)
            time.sleep_ms(250)

            self.spi = SPI(
                self.spi_id,
                baudrate=self.baudrate,
                polarity=0,
                phase=0,
                sck=Pin(self.sck_pin),
                mosi=Pin(self.mosi_pin),
                miso=Pin(self.miso_pin),
            )

            kwargs = {
                "spi": self.spi,
                "cs": Pin(self.cs_pin),
                "int": Pin(self.int_pin),
                "phy_type": network.PHY_W5500,
                "phy_addr": 0,
            }

            try:
                kwargs["reset"] = Pin(self.rst_pin)
                self.lan = network.LAN(**kwargs)
            except TypeError:
                try:
                    del kwargs["reset"]
                except Exception:
                    pass
                self.lan = network.LAN(**kwargs)

            self.lan.active(True)
            time.sleep_ms(250)
            self.lan.ifconfig((
                eth_cfg["ip"],
                eth_cfg["subnet"],
                eth_cfg["gateway"],
                eth_cfg["dns"],
            ))
            time.sleep_ms(100)

            self.ready = True
            print("W5500 ACTIVE")
            print("IP      :", self.lan.ifconfig()[0])
            print("SUBNET  :", self.lan.ifconfig()[1])
            print("GATEWAY :", self.lan.ifconfig()[2])
            print("DNS     :", self.lan.ifconfig()[3])
            print("LINK    :", self.lan.isconnected())
            return True
        except Exception as e:
            self.last_error = repr(e)
            print("W5500 ERROR:", self.last_error)
            self.ready = False
            return False

    def is_connected(self):
        try:
            return bool(self.lan and self.lan.isconnected())
        except Exception:
            return False

    def ifconfig(self):
        try:
            return self.lan.ifconfig() if self.lan else None
        except Exception:
            return None

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
        value = str(value)
        safe = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_.~"
        out = ""
        for ch in value:
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
    def _set_short_timeout(sock, timeout_ms):
        # Small socket timeout slices prevent a slow API from freezing RFID,
        # motor, solenoid, and LED servicing for the full configured timeout.
        slice_ms = max(20, min(100, int(timeout_ms)))
        try:
            sock.settimeout(slice_ms / 1000.0)
        except Exception:
            pass

    def _send_all_cooperative(self, sock, data, deadline_ms, service_callback):
        view = memoryview(data)
        sent = 0
        while sent < len(view):
            if time.ticks_diff(deadline_ms, time.ticks_ms()) <= 0:
                raise OSError("HTTP send timeout")

            try:
                n = sock.send(view[sent:])
            except OSError:
                self._service(service_callback)
                continue

            if n is None:
                return
            if n <= 0:
                self._service(service_callback)
                continue

            sent += n
            self._service(service_callback)

    def _recv_cooperative(self, sock, size, deadline_ms, service_callback):
        while True:
            if time.ticks_diff(deadline_ms, time.ticks_ms()) <= 0:
                raise OSError("HTTP receive timeout")

            try:
                return sock.recv(size)
            except OSError:
                self._service(service_callback)

    def http_request(
        self,
        url,
        method="GET",
        json_body=None,
        timeout_ms=5000,
        max_body=131072,
        service_callback=None,
    ):
        """Perform HTTP over W5500 while periodically servicing critical tasks.

        The configured timeout is preserved, but socket waits are broken into
        short slices so the caller can keep RFID/gate/LED logic alive through
        service_callback.
        """
        if not self.ready:
            raise OSError("W5500 is not initialized")

        timeout_ms = max(100, int(timeout_ms))
        host, port, path = self.parse_http_url(url)
        method = str(method).upper()
        if method not in ("GET", "POST"):
            method = "GET"

        body = b""
        if json_body is not None:
            body = json.dumps(json_body).encode()

        # getaddrinfo can still be blocking when a hostname needs DNS. The
        # default Fastlane setup uses the direct local API IP, avoiding DNS.
        address = socket.getaddrinfo(host, port, 0, socket.SOCK_STREAM)[0][-1]
        s = socket.socket()
        deadline_ms = time.ticks_add(time.ticks_ms(), timeout_ms)

        try:
            self._set_short_timeout(s, timeout_ms)

            # Connect is normally immediate on the direct Ethernet link. Keep
            # the normal socket connect semantics for MicroPython portability.
            s.connect(address)
            self._service(service_callback)

            request = (
                "{} {} HTTP/1.1\r\n"
                "Host: {}\r\n"
                "User-Agent: FastlaneESP32/1.4\r\n"
                "Accept: application/json\r\n"
                "Connection: close\r\n"
            ).format(method, path, host)

            if body:
                request += "Content-Type: application/json\r\n"
                request += "Content-Length: {}\r\n".format(len(body))
            request += "\r\n"

            self._send_all_cooperative(
                s,
                request.encode(),
                deadline_ms,
                service_callback,
            )
            if body:
                self._send_all_cooperative(
                    s,
                    body,
                    deadline_ms,
                    service_callback,
                )

            raw = bytearray()
            header_end = -1
            while header_end < 0:
                chunk = self._recv_cooperative(
                    s,
                    1024,
                    deadline_ms,
                    service_callback,
                )
                if not chunk:
                    break
                raw.extend(chunk)
                if len(raw) > 16384:
                    raise OSError("HTTP headers too large")
                header_end = raw.find(b"\r\n\r\n")
                self._service(service_callback)

            if header_end < 0:
                raise OSError("Invalid HTTP response")

            header_bytes = bytes(raw[:header_end])
            response_body = bytearray(raw[header_end + 4:])
            lines = header_bytes.split(b"\r\n")
            status_line = lines[0].decode("utf-8", "ignore")
            parts = status_line.split(" ", 2)
            status = int(parts[1]) if len(parts) > 1 else 0

            headers = {}
            for line in lines[1:]:
                colon = line.find(b":")
                if colon > 0:
                    key = line[:colon].decode("utf-8", "ignore").strip().lower()
                    value = line[colon + 1:].decode("utf-8", "ignore").strip()
                    headers[key] = value

            content_length = None
            try:
                if "content-length" in headers:
                    content_length = int(headers["content-length"])
            except Exception:
                content_length = None

            while True:
                if content_length is not None and len(response_body) >= content_length:
                    break
                if len(response_body) > max_body:
                    raise OSError("HTTP response exceeds max_response_bytes")

                chunk = self._recv_cooperative(
                    s,
                    2048,
                    deadline_ms,
                    service_callback,
                )
                if not chunk:
                    break
                response_body.extend(chunk)
                self._service(service_callback)

            if len(response_body) > max_body:
                raise OSError("HTTP response exceeds max_response_bytes")

            body_bytes = bytes(
                response_body[:content_length]
                if content_length is not None
                else response_body
            )
            if headers.get("transfer-encoding", "").lower() == "chunked":
                body_bytes = self.decode_chunked(body_bytes)

            return status, headers, body_bytes
        finally:
            try:
                s.close()
            except Exception:
                pass
