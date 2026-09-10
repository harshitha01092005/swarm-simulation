"""Small same-origin HTTP adapter for the desktop and browser clients."""

from concurrent.futures import TimeoutError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import threading
from urllib.parse import urlsplit


class SwarmHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, state, submit, web_root=None, sample_path=None):
        self.state = state
        self.submit = submit
        self.web_root = Path(web_root or Path(__file__).parent / "web").resolve()
        self.sample_path = Path(sample_path) if sample_path else None
        super().__init__(address, SwarmHandler)

    def start(self):
        thread = threading.Thread(target=self.serve_forever, name="swarm-http", daemon=True)
        thread.start()
        return thread


class SwarmHandler(BaseHTTPRequestHandler):
    server_version = "DroneSwarm/1.0"

    def log_message(self, format, *args):
        # ROS logs command outcomes; polling should not flood the terminal.
        pass

    def _allowed(self):
        host = self.headers.get("Host", "")
        try:
            hostname = urlsplit("http://" + host).hostname
        except ValueError:
            hostname = None
        if hostname not in {"localhost", "127.0.0.1", "::1"}:
            self._json(403, {"error": "Use localhost or 127.0.0.1 to access this application"})
            return False
        origin = self.headers.get("Origin")
        if origin and origin != "http://" + host:
            self._json(403, {"error": "Cross-origin commands are not permitted"})
            return False
        return True

    def _send(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, status, value):
        self._send(status, json.dumps(value, allow_nan=False).encode(), "application/json")

    def do_GET(self):
        if not self._allowed():
            return
        route = urlsplit(self.path).path
        if route == "/api/state":
            self._json(200, self.server.state())
            return
        if route == "/api/sample":
            path = self.server.sample_path
        else:
            path = (self.server.web_root / ("index.html" if route == "/" else route.lstrip("/"))).resolve()
            if self.server.web_root not in path.parents:
                self._json(404, {"error": "Not found"})
                return
        if path is None or not path.is_file():
            self._json(404, {"error": "Not found"})
            return
        mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
        if path.suffix in (".js", ".mjs"):
            mime = "text/javascript"
        self._send(200, path.read_bytes(), mime)

    def do_POST(self):
        if not self._allowed():
            return
        if urlsplit(self.path).path != "/api/command":
            self._json(404, {"error": "Not found"})
            return
        try:
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("Send an application/json command")
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 12 * 1024 * 1024:
                self._json(413, {"error": "Command exceeds the 12 MiB limit"})
                return
            self.connection.settimeout(10)
            data = self.rfile.read(length)
            if len(data) != length:
                raise ValueError("Incomplete request body")
            command = json.loads(data)
            if not isinstance(command, dict) or not isinstance(command.get("action"), str):
                raise ValueError("Command must be an object with an action")
            future = self.server.submit(command)
            try:
                result = future.result(timeout=15)
            except TimeoutError:
                future.cancel()
                self._json(504, {"error": "Controller did not respond; check simulation status"})
                return
            self._json(200, result)
        except (ValueError, TypeError, RuntimeError, KeyError) as error:
            self._json(400, {"error": str(error)})
        except (OSError, OverflowError):
            self._json(400, {"error": "Invalid or interrupted request"})
