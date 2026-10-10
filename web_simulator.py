#!/usr/bin/env python3
"""
Web Simulator Server for STM32 Edge-AI Underground Drainage Hazard System
Serves index.html on http://localhost:8080
"""

import http.server
import socketserver
import os
import sys

PORT = 8080
DIRECTORY = "/home/senju/Documents/drainage-gas-hazard-system"

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

def main():
    os.chdir(DIRECTORY)
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print("=" * 70)
        print(" STM32 Drainage Hazard Interactive Web Simulator Server")
        print(f" Serving at: http://localhost:{PORT}/")
        print(" Open http://localhost:8080/ in your browser to view and interact.")
        print(" Press Ctrl+C to stop.")
        print("=" * 70)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")

if __name__ == "__main__":
    main()
