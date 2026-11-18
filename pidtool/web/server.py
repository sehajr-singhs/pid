"""Tiny local web UI: stdlib http.server, no extra dependencies."""

from __future__ import annotations

import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from importlib.resources import files

ROOT = Path("out/fatman")


def _index_html() -> str:
    return files("pidtool.web").joinpath("index.html").read_text(encoding="utf-8")


class Handler(BaseHTTPRequestHandler):
    out_dir: Path = ROOT

    def log_message(self, fmt, *args):  # quieter than the default
        print(f"[pid web] {fmt % args}")

    def _send(self, code: int, body: bytes, ctype: str = "application/json") -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _exports(self) -> bytes:
        """Every generated export file, for the download buttons."""
        base = self.out_dir.resolve()
        root = base / "exports"
        items = []
        if root.exists():
            for f in sorted(root.rglob("*")):
                if f.is_file():
                    items.append({
                        "group": f.parent.name,
                        "name": f.name,
                        "path": "/" + f.relative_to(base).as_posix(),
                        "bytes": f.stat().st_size,
                    })
        return json.dumps(items).encode()

    def _json_file(self, name: str) -> None:
        path = self.out_dir / name
        if not path.exists():
            self._send(404, json.dumps({"error": f"{name} not found - run 'pid fatman'"}).encode())
            return
        self._send(200, path.read_bytes())

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = unquote(parsed.path)

        if path in ("/", "/index.html"):
            self._send(200, _index_html().encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/api/robot":
            self._json_file("robot.json")
            return
        if path == "/api/gains":
            self._json_file("gains.json")
            return
        if path == "/api/exports":
            self._send(200, self._exports())
            return
        if path == "/api/summary":
            p = self.out_dir / "SUMMARY.md"
            if not p.exists():
                self._send(404, json.dumps({"error": "SUMMARY.md not found"}).encode())
            else:
                self._send(200, p.read_bytes(), "text/markdown; charset=utf-8")
            return
        if path.startswith("/plots/"):
            f = self.out_dir / "plots" / Path(path).name
            if f.exists():
                ctype = mimetypes.guess_type(str(f))[0] or "application/octet-stream"
                self._send(200, f.read_bytes(), ctype)
            else:
                self._send(404, b"no such plot", "text/plain")
            return
        if path.startswith("/exports/"):
            base = self.out_dir.resolve()
            f = (base / path.lstrip("/")).resolve()
            if f.is_file() and base in f.parents:
                self._send(200, f.read_bytes(), "text/plain; charset=utf-8")
            else:
                self._send(404, b"no such export", "text/plain")
            return
        self._send(404, b"not found", "text/plain")

    def do_POST(self) -> None:
        if urlparse(self.path).path != "/api/run":
            self._send(404, b"not found", "text/plain")
            return
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            body = {}
        from ..cli import run_fatman

        try:
            result = run_fatman(
                root=body.get("root", "C:/Users/sehaj/ftc-cad/robots/fatman"),
                out=str(self.out_dir),
                seed=int(body.get("seed", 0)),
            )
            self._send(200, json.dumps({
                "ok": True,
                "elapsed_s": result["elapsed_s"],
                "robot": result["robot"]["name"],
            }).encode())
        except Exception as exc:  # surface the message in the UI instead of a blank 500
            self._send(500, json.dumps({"ok": False, "error": str(exc)}).encode())


def serve(out_dir: str = "out/fatman", port: int = 8000) -> None:
    Handler.out_dir = Path(out_dir)
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"tuning... serving {Handler.out_dir} at http://127.0.0.1:{port}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        httpd.server_close()
