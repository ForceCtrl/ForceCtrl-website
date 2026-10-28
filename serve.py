#!/usr/bin/env python3
"""Static server for local preview: no caching, the right MIME types for the
demos, and HTTP Range requests so that videos can be scrubbed (the stock
http.server ignores Range, so seeking does nothing). GitHub Pages serves ranges
itself, so none of this is needed once deployed."""
import http.server, socketserver, sys, mimetypes, os, re

mimetypes.add_type("application/wasm", ".wasm")
mimetypes.add_type("text/javascript", ".mjs")

class H(http.server.SimpleHTTPRequestHandler):
  def end_headers(self):
    self.send_header("Cache-Control", "no-cache")
    self.send_header("Accept-Ranges", "bytes")
    super().end_headers()
  def log_message(self, *a): pass

  def send_head(self):
    m = re.match(r"bytes=(\d*)-(\d*)$", self.headers.get("Range", ""))
    path = self.translate_path(self.path)
    if not m or os.path.isdir(path) or not os.path.isfile(path):
      return super().send_head()
    size = os.path.getsize(path)
    start, end = m.groups()
    if start == "":  # suffix range: last N bytes
      start, end = max(size - int(end), 0), size - 1
    else:
      start, end = int(start), (int(end) if end else size - 1)
    end = min(end, size - 1)
    if start > end or start >= size:
      self.send_response(416); self.send_header("Content-Range", f"bytes */{size}"); self.end_headers(); return None
    self.send_response(206)
    self.send_header("Content-Type", self.guess_type(path))
    self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
    self.send_header("Content-Length", str(end - start + 1))
    self.end_headers()
    f = open(path, "rb"); f.seek(start)
    self._range_left = end - start + 1
    return f

  def copyfile(self, src, dst):
    left = getattr(self, "_range_left", None)
    if left is None:
      return super().copyfile(src, dst)
    while left > 0:
      chunk = src.read(min(left, 1 << 16))
      if not chunk: break
      dst.write(chunk); left -= len(chunk)
    self._range_left = None

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
socketserver.TCPServer.allow_reuse_address = True
with socketserver.ThreadingTCPServer(("127.0.0.1", port), H) as httpd:
  print("serving on", port, flush=True)
  httpd.serve_forever()
