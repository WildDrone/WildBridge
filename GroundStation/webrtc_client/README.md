# WebRTC Drone Video Client

Python client for receiving video streams from the DJI drone via WebRTC.

## Installation

```bash
cd webrtc_client
pip install -r requirements.txt
```

## Usage

### Basic Usage (Display Video)
```bash
python webrtc_drone_viewer.py --server ws://192.168.1.100:8081
```

### Save Video to File
```bash
python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --save-video output.mp4
```

### Save Frames to Directory
```bash
python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --save-frames ./frames/
```

### Headless Mode (No Display)
```bash
python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --headless --save-video output.mp4
```

### Debug Mode
```bash
python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --debug
```

## Controls

When displaying video:
- **Q** - Quit the application
- **S** - Save a snapshot of the current frame

## Arguments

| Argument | Short | Description |
|----------|-------|-------------|
| `--server` | `-s` | WebSocket server URL (required) |
| `--headless` | | Run without displaying video |
| `--save-video` | `-v` | Save video to file (e.g., output.mp4) |
| `--save-frames` | `-f` | Save frames to directory |
| `--debug` | | Enable debug logging |

## How It Works

1. Connects to the WebRTC signaling server via WebSocket
2. Receives an SDP offer from the drone
3. Creates an SDP answer and exchanges ICE candidates
4. Establishes a direct peer-to-peer connection
5. Receives and displays/saves the video stream

## Troubleshooting

### "Connection refused"
- Make sure the drone app is running and the WebRTC server is started
- Verify you're on the same network as the drone controller
- Check the IP address and port are correct

### "No video displayed"
- The drone camera may not be active
- Try selecting a different camera in the app
- Check the debug output for errors

### High latency
- WebRTC over local network should have <100ms latency
- If latency is high, check network congestion
- Try connecting to a less congested WiFi channel
