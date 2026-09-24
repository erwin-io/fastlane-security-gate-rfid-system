import socket
import os
import time


class CooperativeWebServer:
    """Small cooperative HTTP/1.1 server for MicroPython.

    It handles one client incrementally so opening the configuration UI does
    not monopolize the main loop. Large static files are streamed a chunk at a
    time and request bodies are received incrementally.
    """

    def __init__(
        self,
        route_callback,
        root_file,
        file_chunk_size=1024,
        recv_chunk_size=1024,
        send_chunk_size=1024,
        max_header_bytes=8192,
        max_body_bytes=65536,
        client_timeout_ms=2000,
    ):
        self.route_callback = route_callback
        self.root_file = root_file
        self.file_chunk_size = int(file_chunk_size)
        self.recv_chunk_size = int(recv_chunk_size)
        self.send_chunk_size = int(send_chunk_size)
        self.max_header_bytes = int(max_header_bytes)
        self.max_body_bytes = int(max_body_bytes)
        self.client_timeout_ms = int(client_timeout_ms)

        self.server = None
        self.ready = False
        self.bind_ip = ""

        self.client = None
        self.client_address = None
        self.last_activity_ms = 0

        self.request_buffer = bytearray()
        self.header_end = -1
        self.content_length = 0
        self.request_method = ""
        self.request_target = ""
        self.headers_parsed = False

        self.send_buffer = b""
        self.send_offset = 0
        self.file_handle = None
        self.file_done = False
        self.response_active = False

    @staticmethod
    def status_reason(code):
        reasons = {
            200: "OK",
            204: "No Content",
            400: "Bad Request",
            404: "Not Found",
            409: "Conflict",
            500: "Internal Server Error",
            502: "Bad Gateway",
        }
        return reasons.get(int(code), "OK")

    @property
    def busy(self):
        return self.client is not None

    def initialize(self, bind_ip, port=80):
        self.close()
        self.bind_ip = str(bind_ip)

        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except Exception:
                pass
            self.server.bind((self.bind_ip, int(port)))
            self.server.listen(2)
            self.server.setblocking(False)
            self.ready = True
            print("CONFIG WEB SERVER READY: http://{}".format(self.bind_ip))
            print("WEB MODE: COOPERATIVE / NON-BLOCKING")
            return True
        except Exception as e:
            print("WEB SERVER ERROR:", repr(e))
            self.ready = False
            self.close()
            return False

    def _reset_client_state(self):
        self.request_buffer = bytearray()
        self.header_end = -1
        self.content_length = 0
        self.request_method = ""
        self.request_target = ""
        self.headers_parsed = False
        self.send_buffer = b""
        self.send_offset = 0
        self.response_active = False
        self.file_done = False

        if self.file_handle is not None:
            try:
                self.file_handle.close()
            except Exception:
                pass
        self.file_handle = None

    def _close_client(self):
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.client_address = None
        self._reset_client_state()

    def close(self):
        self._close_client()
        if self.server is not None:
            try:
                self.server.close()
            except Exception:
                pass
        self.server = None
        self.ready = False

    def _accept_if_needed(self):
        if not self.ready or self.server is None or self.client is not None:
            return

        try:
            client, address = self.server.accept()
        except OSError:
            return
        except Exception:
            return

        self.client = client
        self.client_address = address
        self.last_activity_ms = time.ticks_ms()
        self._reset_client_state()

        try:
            self.client.setblocking(False)
        except Exception:
            try:
                self.client.settimeout(0)
            except Exception:
                pass

    def _parse_headers_if_ready(self):
        if self.headers_parsed:
            return True

        self.header_end = self.request_buffer.find(b"\r\n\r\n")
        if self.header_end < 0:
            if len(self.request_buffer) > self.max_header_bytes:
                raise ValueError("HTTP headers too large")
            return False

        header_bytes = bytes(self.request_buffer[:self.header_end])
        lines = header_bytes.split(b"\r\n")
        if not lines:
            raise ValueError("Invalid HTTP request")

        request_line = lines[0].decode("utf-8", "ignore")
        parts = request_line.split(" ")
        if len(parts) < 2:
            raise ValueError("Invalid HTTP request line")

        self.request_method = parts[0].upper()
        self.request_target = parts[1]

        headers = {}
        for line in lines[1:]:
            colon = line.find(b":")
            if colon > 0:
                key = line[:colon].decode("utf-8", "ignore").strip().lower()
                value = line[colon + 1:].decode("utf-8", "ignore").strip()
                headers[key] = value

        try:
            self.content_length = int(headers.get("content-length", "0"))
        except Exception:
            self.content_length = 0

        if self.content_length < 0 or self.content_length > self.max_body_bytes:
            raise ValueError("Request body too large")

        self.headers_parsed = True
        return True

    def _request_complete(self):
        if not self._parse_headers_if_ready():
            return False

        body_start = self.header_end + 4
        received_body = len(self.request_buffer) - body_start
        return received_body >= self.content_length

    def _make_header(self, status, content_type, content_length):
        return (
            "HTTP/1.1 {} {}\r\n"
            "Content-Type: {}\r\n"
            "Content-Length: {}\r\n"
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Access-Control-Allow-Methods: GET,POST,DELETE,OPTIONS\r\n"
            "\r\n"
        ).format(
            int(status),
            self.status_reason(status),
            content_type,
            int(content_length),
        ).encode("utf-8")

    def _prepare_body_response(self, status, content_type, body):
        if isinstance(body, str):
            body = body.encode("utf-8")
        elif body is None:
            body = b""

        self.send_buffer = self._make_header(
            status, content_type, len(body)
        ) + body
        self.send_offset = 0
        self.response_active = True
        self.file_done = True

    def _prepare_file_response(self, path, content_type):
        try:
            size = os.stat(path)[6]
            self.file_handle = open(path, "rb")
        except Exception:
            self._prepare_body_response(404, "text/plain", "File not found")
            return

        self.send_buffer = self._make_header(200, content_type, size)
        self.send_offset = 0
        self.response_active = True
        self.file_done = False

    def _dispatch_request(self):
        body_start = self.header_end + 4
        body = bytes(
            self.request_buffer[
                body_start:body_start + self.content_length
            ]
        )

        if self.request_method == "OPTIONS":
            self._prepare_body_response(204, "text/plain", b"")
            return

        target_path = self.request_target.split("?", 1)[0]
        if self.request_method == "GET" and target_path == "/":
            self._prepare_file_response(
                self.root_file,
                "text/html; charset=utf-8",
            )
            return

        try:
            status, content_type, response_body = self.route_callback(
                self.request_method,
                self.request_target,
                body,
            )
        except Exception as e:
            status = 500
            content_type = "application/json"
            response_body = '{"ok":false,"error":"%s"}' % repr(e).replace('"', "'")

        self._prepare_body_response(status, content_type, response_body)

    def _receive_one_chunk(self):
        if self.client is None or self.response_active:
            return

        try:
            chunk = self.client.recv(self.recv_chunk_size)
        except OSError:
            return
        except Exception:
            self._close_client()
            return

        if not chunk:
            self._close_client()
            return

        self.request_buffer.extend(chunk)
        self.last_activity_ms = time.ticks_ms()

        try:
            if self._request_complete():
                self._dispatch_request()
        except Exception as e:
            self._prepare_body_response(
                400,
                "application/json",
                '{"ok":false,"error":"%s"}' % repr(e).replace('"', "'"),
            )

    def _send_one_chunk(self):
        if self.client is None or not self.response_active:
            return

        # If the current buffer has been completely sent and this is a file
        # response, fetch only one next chunk from flash this update cycle.
        if self.send_offset >= len(self.send_buffer):
            if self.file_handle is not None and not self.file_done:
                try:
                    chunk = self.file_handle.read(self.file_chunk_size)
                except Exception:
                    chunk = b""

                if chunk:
                    self.send_buffer = chunk
                    self.send_offset = 0
                else:
                    self.file_done = True
                    try:
                        self.file_handle.close()
                    except Exception:
                        pass
                    self.file_handle = None
                    self._close_client()
                    return
            else:
                self._close_client()
                return

        remaining = len(self.send_buffer) - self.send_offset
        if remaining <= 0:
            return

        count = min(self.send_chunk_size, remaining)
        view = memoryview(self.send_buffer)[
            self.send_offset:self.send_offset + count
        ]

        try:
            sent = self.client.send(view)
        except OSError:
            return
        except Exception:
            self._close_client()
            return

        if sent is None:
            sent = count
        if sent <= 0:
            return

        self.send_offset += sent
        self.last_activity_ms = time.ticks_ms()

    def update(self):
        """Service a tiny amount of HTTP work and immediately return."""
        if not self.ready:
            return

        self._accept_if_needed()
        if self.client is None:
            return

        if time.ticks_diff(time.ticks_ms(), self.last_activity_ms) > self.client_timeout_ms:
            self._close_client()
            return

        if self.response_active:
            self._send_one_chunk()
        else:
            self._receive_one_chunk()
