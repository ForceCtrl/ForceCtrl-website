#!/usr/bin/env python3
"""Static server with the cross-origin isolation headers the WASM build needs.

MuJoCo's WebAssembly build uses pthreads, which need SharedArrayBuffer, which
browsers only enable on cross-origin-isolated pages. GitHub Pages cannot send
these headers, so the deployed page uses coi-serviceworker.min.js instead; this
server sets them directly for local work.
"""
import http.server, socketserver, sys, mimetypes

mimetypes.add_type("application/wasm", ".wasm")
mimetypes.add_type("text/javascript", ".mjs")

class H(http.server.SimpleHTTPRequestHandler):
  def end_headers(self):
    self.send_header("Cross-Origin-Opener-Policy", "same-origin")
    self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
    self.send_header("Cross-Origin-Resource-Policy", "cross-origin")
    self.send_header("Cache-Control", "no-cache")
    super().end_headers()
  def log_message(self, *a): pass

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
socketserver.TCPServer.allow_reuse_address = True
with socketserver.ThreadingTCPServer(("127.0.0.1", port), H) as httpd:
  print("serving on", port, flush=True)
  httpd.serve_forever()
