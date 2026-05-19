#!/usr/bin/env python3
import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

PORT = int(os.environ.get("PORT", "8090"))
TELEMETRY_PORT = int(os.environ.get("TELEMETRY_PORT", "8081"))
DISCOVERY_INTERVAL_MS = int(os.environ.get("DISCOVERY_INTERVAL_MS", "5000"))
MEDIAMTX_API_URL = os.environ.get("MEDIAMTX_API_URL", "http://127.0.0.1:9997").rstrip("/")
MEDIAMTX_WEBRTC_URL = os.environ.get("MEDIAMTX_WEBRTC_URL", "http://127.0.0.1:8889").rstrip("/")
LOG_DIR = Path(os.environ.get("LOG_DIR", "/logs"))
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"
DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"
DISCOVERY_PORT = 30000
MULTICAST_GROUP = "239.255.42.99"
MULTICAST_PORT = 30001
TELEMETRY_IDLE_TIMEOUT_S = float(os.environ.get("TELEMETRY_IDLE_TIMEOUT_S", "5"))
BASE_DIR = Path(__file__).resolve().parent
PUBLIC_DIR = BASE_DIR / "public"

DRONE_NAMES = [name.strip() for name in os.environ.get("DRONE_NAMES", "").split(",") if name.strip()]
FALLBACK_IPS = {}
for item in os.environ.get("DRONE_FALLBACKS", "").split(","):
    if "=" in item:
        name, ip = item.split("=", 1)
        FALLBACK_IPS[name.strip()] = ip.strip()

LOG_DIR.mkdir(parents=True, exist_ok=True)
EVENT_LOG = LOG_DIR / f"video-connection-test-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.ndjson"

lock = threading.RLock()
sse_clients = []
telemetry_threads = {}
telemetry_stop_events = {}

def utc_now():
    return datetime.now(timezone.utc).isoformat()


def make_drone(name, ip=None):
    ip = ip or FALLBACK_IPS.get(name)
    return {
        "name": name,
        "streamName": name,
        "ip": ip,
        "discoveredIp": None,
        "status": "fallback" if ip else "missing",
        "telemetryConnected": False,
        "telemetryPackets": 0,
        "telemetryReconnects": 0,
        "lastTelemetryAt": None,
        "lastDiscoveryAt": None,
        "lastError": None,
        "lastTelemetry": None,
        "mediaMtx": None,
        "browserStats": None,
        "ignored": False,
    }


drones = {name: make_drone(name) for name in DRONE_NAMES}
for fallback_name in FALLBACK_IPS:
    drones.setdefault(fallback_name, make_drone(fallback_name))


def write_sse(payload):
    data = f"data: {json.dumps(payload)}\n\n".encode("utf-8")
    dead = []
    for handler in list(sse_clients):
        try:
            handler.wfile.write(data)
            handler.wfile.flush()
        except Exception:
            dead.append(handler)
    for handler in dead:
        if handler in sse_clients:
            sse_clients.remove(handler)


def public_state():
    with lock:
        return {
            "generatedAt": utc_now(),
            "mediamtxWebrtcUrl": MEDIAMTX_WEBRTC_URL,
            "logFile": str(EVENT_LOG),
            "dynamicDiscovery": not bool(DRONE_NAMES),
            "drones": [dict(drone) for drone in drones.values()],
        }


def emit_state():
    write_sse({"type": "state", "payload": public_state()})


def log_event(event_type, **payload):
    entry = {"ts": utc_now(), "type": event_type, **payload}
    with EVENT_LOG.open("a", encoding="utf-8") as file:
        file.write(json.dumps(entry, separators=(",", ":")) + "\n")
    write_sse({"type": event_type, "payload": entry})


def parse_discovery_response(message, remote_ip):
    if not message.startswith(DISCOVERY_RESPONSE_PREFIX):
        return None
    payload = message[len(DISCOVERY_RESPONSE_PREFIX):]
    if ":" not in payload:
        return {"ip": remote_ip, "name": payload or remote_ip}
    ip, name = payload.split(":", 1)
    return {"ip": ip or remote_ip, "name": name.strip() or remote_ip}


def upsert_discovered(found):
    with lock:
        name = found["name"] if found["name"] in drones else None
        if name is None:
            for candidate, drone in drones.items():
                if drone.get("ip") == found["ip"] or drone.get("discoveredIp") == found["ip"]:
                    name = candidate
                    break
        if name is None:
            if DRONE_NAMES and found["name"] not in DRONE_NAMES:
                log_event("discovery_ignored", ip=found["ip"], name=found["name"], reason="not in DRONE_NAMES")
                return
            name = found["name"]
            drones[name] = make_drone(name, found["ip"])
        drone = drones[name]
        old_ip = drone.get("ip")
        drone["discoveredIp"] = found["ip"]
        drone["ip"] = found["ip"]
        if not drone.get("ignored"):
            drone["status"] = "discovered"
        drone["lastDiscoveryAt"] = utc_now()
        drone["lastError"] = None
    if old_ip != found["ip"]:
        log_event("drone_discovered", drone=name, reportedName=found["name"], ip=found["ip"], previousIp=old_ip)
        stop_telemetry(name)
    connect_telemetry(name)
    emit_state()


def run_discovery_socket(sock, target):
    sock.settimeout(1.5)
    try:
        sock.sendto(DISCOVERY_MSG, target)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout:
                break
            found = parse_discovery_response(data.decode("utf-8", errors="ignore").strip(), address[0])
            if found:
                upsert_discovered(found)
    finally:
        sock.close()


def discover_now():
    log_event("discovery_started", drones=DRONE_NAMES or "any")
    try:
        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        threading.Thread(target=run_discovery_socket, args=(broadcast_sock, ("255.255.255.255", DISCOVERY_PORT)), daemon=True).start()
    except Exception as exc:
        log_event("discovery_error", transport="broadcast", error=str(exc))

    try:
        multicast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        multicast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        multicast_sock.bind(("", MULTICAST_PORT))
        membership = socket.inet_aton(MULTICAST_GROUP) + socket.inet_aton("0.0.0.0")
        multicast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        threading.Thread(target=run_discovery_socket, args=(multicast_sock, (MULTICAST_GROUP, MULTICAST_PORT)), daemon=True).start()
    except Exception as exc:
        log_event("discovery_error", transport="multicast", error=str(exc))

    for name in list(drones.keys()):
        connect_telemetry(name)
    emit_state()


def stop_telemetry(name):
    event = telemetry_stop_events.pop(name, None)
    if event:
        event.set()
    telemetry_threads.pop(name, None)


def connect_telemetry(name):
    with lock:
        drone = drones.get(name)
        if not drone or drone.get("ignored") or not drone.get("ip") or name in telemetry_threads:
            return
        stop_event = threading.Event()
        telemetry_stop_events[name] = stop_event
    thread = threading.Thread(target=telemetry_loop, args=(name, stop_event), daemon=True)
    telemetry_threads[name] = thread
    thread.start()


def telemetry_loop(name, stop_event):
    while not stop_event.is_set():
        with lock:
            drone = drones[name]
            ip = drone.get("ip")
        try:
            with socket.create_connection((ip, TELEMETRY_PORT), timeout=5) as sock:
                sock.settimeout(2)
                with lock:
                    drone = drones[name]
                    drone["telemetryConnected"] = True
                    drone["status"] = "telemetry_connected"
                    drone["lastError"] = None
                log_event("telemetry_connected", drone=name, ip=ip, port=TELEMETRY_PORT)
                emit_state()
                buffer = ""
                last_sample_log = 0
                last_rx = time.time()
                while not stop_event.is_set():
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        if time.time() - last_rx > TELEMETRY_IDLE_TIMEOUT_S:
                            raise TimeoutError(f"no telemetry for {TELEMETRY_IDLE_TIMEOUT_S:.1f}s")
                        continue
                    if not chunk:
                        break
                    last_rx = time.time()
                    buffer += chunk.decode("utf-8", errors="ignore")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        with lock:
                            drone = drones[name]
                            drone["telemetryPackets"] += 1
                            drone["lastTelemetryAt"] = utc_now()
                            packet_count = drone["telemetryPackets"]
                        try:
                            telemetry = json.loads(line)
                            with lock:
                                drones[name]["lastTelemetry"] = telemetry
                        except json.JSONDecodeError as exc:
                            log_event("telemetry_parse_error", drone=name, error=str(exc), sample=line[:200])
                            continue
                        now = time.time()
                        if now - last_sample_log > 1:
                            last_sample_log = now
                            log_event(
                                "telemetry_sample",
                                drone=name,
                                packets=packet_count,
                                batteryLevel=telemetry.get("batteryLevel"),
                                flightMode=telemetry.get("flightMode"),
                            )
        except Exception as exc:
            with lock:
                drone = drones[name]
                drone["lastError"] = str(exc)
            log_event("telemetry_error", drone=name, ip=ip, error=str(exc))
        finally:
            with lock:
                drone = drones[name]
                drone["telemetryConnected"] = False
                drone["telemetryReconnects"] += 1
                if drone.get("ignored"):
                    drone["status"] = "ignored"
                elif drone["status"] != "missing":
                    drone["status"] = "telemetry_disconnected"
            log_event("telemetry_disconnected", drone=name, ip=ip, reconnects=drones[name]["telemetryReconnects"])
            emit_state()
        stop_event.wait(2)
    with lock:
        telemetry_threads.pop(name, None)


def poll_mediamtx_loop():
    while True:
        try:
            with urllib.request.urlopen(f"{MEDIAMTX_API_URL}/v3/paths/list", timeout=2) as response:
                body = json.loads(response.read().decode("utf-8"))
            items = body.get("items") or []
            with lock:
                for drone in drones.values():
                    drone["mediaMtx"] = next((item for item in items if item.get("name") == drone["streamName"]), None)
            log_event(
                "mediamtx_paths",
                paths=[{"name": item.get("name"), "ready": item.get("ready"), "readers": len(item.get("readers") or [])} for item in items],
            )
            emit_state()
        except Exception as exc:
            log_event("mediamtx_api_error", error=str(exc))
        time.sleep(2)


def set_drone_ignored(name, ignored):
    with lock:
        drone = drones.get(name)
        if not drone:
            return False
        drone["ignored"] = ignored
        if ignored:
            drone["status"] = "ignored"
        elif drone.get("ip"):
            drone["status"] = "discovered"
    if ignored:
        stop_telemetry(name)
        log_event("drone_ignored", drone=name)
    else:
        log_event("drone_unignored", drone=name)
        connect_telemetry(name)
    emit_state()
    return True


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(PUBLIC_DIR), **kwargs)

    def log_message(self, fmt, *args):
        return

    def send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/events":
            self.send_response(200)
            self.send_header("content-type", "text/event-stream")
            self.send_header("cache-control", "no-cache")
            self.send_header("connection", "keep-alive")
            self.end_headers()
            sse_clients.append(self)
            try:
                self.wfile.write(f"data: {json.dumps({'type': 'state', 'payload': public_state()})}\n\n".encode("utf-8"))
                self.wfile.flush()
                while self in sse_clients:
                    time.sleep(1)
            finally:
                if self in sse_clients:
                    sse_clients.remove(self)
            return
        if path == "/api/drones":
            self.send_json(200, public_state())
            return
        if path == "/api/logs":
            self.send_json(200, {"eventLog": str(EVENT_LOG)})
            return
        super().do_GET()

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/discover":
            discover_now()
            self.send_json(202, {"ok": True})
            return
        if path.startswith("/api/drones/") and path.endswith("/ignore"):
            parts = [part for part in path.split("/") if part]
            if len(parts) == 4:
                name = unquote(parts[2])
                length = int(self.headers.get("content-length", "0"))
                body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                if set_drone_ignored(name, bool(body.get("ignored"))):
                    self.send_json(200, public_state())
                else:
                    self.send_json(404, {"error": "Drone not found"})
                return
        if path == "/api/client-stats":
            length = int(self.headers.get("content-length", "0"))
            try:
                body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                drone_name = body.get("drone")
                with lock:
                    if drone_name in drones:
                        drones[drone_name]["browserStats"] = body
                log_event("browser_stats", **body)
                self.send_response(204)
                self.end_headers()
            except Exception as exc:
                self.send_json(400, {"error": str(exc)})
            return
        self.send_error(404)


def discovery_loop():
    discover_now()
    while True:
        time.sleep(DISCOVERY_INTERVAL_MS / 1000)
        discover_now()


if __name__ == "__main__":
    log_event("video_grid_started", port=PORT, logFile=str(EVENT_LOG), drones=DRONE_NAMES or "any")
    threading.Thread(target=discovery_loop, daemon=True).start()
    threading.Thread(target=poll_mediamtx_loop, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"WildBridge video grid listening on http://localhost:{PORT}")
    print(f"Logging diagnostics to {EVENT_LOG}")
    server.serve_forever()
