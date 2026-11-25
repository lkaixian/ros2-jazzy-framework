"""Fire-and-forget UDP telemetry from the arm tools to the GUI (127.0.0.1:8089). Never blocks, never raises, nothing listening is fine."""
import json
import socket

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(kind, **data):
    try:
        _sock.sendto(json.dumps({'k': kind, **data}).encode(), ('127.0.0.1', 8089))
    except (OSError, TypeError, ValueError):
        pass
