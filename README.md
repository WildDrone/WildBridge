<div align="center">
  <img src="WildBridge_icon.png" alt="WildBridge App Icon" width="300" height="300">
  
  # WildBridge
  
  ![WildBridge Diagram](https://github.com/WildDrone/WildBridge/blob/main/WildBridgeDiagram.png "WildBridge System Architecture")
</div>

> **WildBridge: Ground Station Interface for Lightweight Multi-Drone Control and Telemetry on DJI Platforms**  
> Part of the [WildDrone Project](https://wilddrone.eu) - European Union's Horizon Europe Research Program

## Overview

WildBridge is an open-source Android application that extends DJI's Mobile SDK V5 to provide accessible telemetry, video streaming, and low-level control for scientific research applications. Running directly on the DJI remote controller, it exposes network interfaces (HTTP and RTSP) over a local area network, enabling seamless integration with ground stations and external research tools.

### Key Features

- **Real-time Telemetry**: HTTP-based access to flight data, sensor readings, and drone status
- **Live Video Streaming**: RTSP video feed compatible with OpenCV, FFmpeg, and VLC
- **Multi-drone Coordination**: Support for up to 10 concurrent drones with sub-100ms latency
- **Wildlife Monitoring**: Integrated YOLO-based object detection and geolocation
- **Scientific Applications**: Proven in conservation, wildfire detection, and atmospheric research
- **Cross-platform Integration**: Compatible with Python, ROS 2, and standard HTTP clients

## Supported Hardware

### DJI Drones (Mobile SDK V5 Compatible)
- **DJI Mini 3/Mini 3 Pro** - Consumer-grade platforms
- **DJI Mini 4 Pro** - Enhanced imaging capabilities
- **DJI Mavic 3 Enterprise Series** - Professional applications with thermal imaging
- **DJI Matrice 30 Series (M30/M30T)** - Industrial platforms
- **DJI Matrice 300 RTK** - High-precision surveying
- **DJI Matrice 350 RTK** - Latest enterprise platform

### Remote Controllers
- **DJI RC Pro** - Primary supported controller
- **DJI RC Plus** - Enterprise compatibility
- **DJI RC-N3** - Standard controller (tested with smartphones)

### System Requirements

#### Ground Station
- **OS**: Ubuntu 24.04+ / Windows 10+ / macOS 12+
- **Python**: 3.8 or higher
- **Network**: Wi-Fi 5 (802.11ac) or better
- **Hardware**: Intel Tiger Lake CPU or equivalent

#### Android App
- **Android**: 7.0 (API level 24) or higher
- **Development**: Android Studio Koala 2024.1.1
- **RAM**: 4GB minimum, 8GB recommended
- **Storage**: 2GB available space

## Performance Characteristics

Based on controlled experiments with consumer-grade hardware:

### Telemetry Performance
- **Latency**: <113ms mean, <290ms 90th percentile (up to 10 drones at 32Hz)
- **Scalability**: Optimal performance with up to 7 concurrent drones
- **Payload**: ~500 bytes per `/aircraft/allStates` request

### Video Streaming Performance
- **Latency**: 1.4-1.6s (1-4 drones), 1.8-1.9s (5-6 drones)
- **Scalability Limit**: 6 concurrent video streams before degradation
- **Format**: Standard Definition via RTSP
- **Compatibility**: FFmpeg, OpenCV, VLC

## Quick Start

### Prerequisites

1. **Hardware Setup**
   - DJI drone and compatible remote controller
   - Local Wi-Fi network (5GHz recommended)
   - Ground station computer

2. **Software Installation**
   ```bash
   git clone https://github.com/WildDrone/WildBridge.git
   cd WildBridge
   ```

3. **Python Dependencies**
   ```bash
   cd GroundStation
   pip install ultralytics opencv-python matplotlib requests nicegui numpy pillow
   ```

### Basic Usage

#### 1. Remote Controller Setup
- Connect RC to local Wi-Fi network
- Note the RC's IP address from network settings
- Install and launch WildBridge app
- Enable "Virtual Stick" mode

#### 2. Ground Station Connection

**Telemetry Access** (Python):
```python
import requests

rc_ip = "192.168.1.100"  # Your RC IP
response = requests.get(f"http://{rc_ip}:8080/aircraft/allStates")
print(response.json())
```

**Video Streaming** (OpenCV):
```python
import cv2

rtsp_url = f"rtsp://aaa:aaa@{rc_ip}:8554/streaming/live/1"
cap = cv2.VideoCapture(rtsp_url)
ret, frame = cap.read()
```

**Control Commands**:
```python
# Takeoff
requests.post(f"http://{rc_ip}:8080/send/takeoff")

# Navigate to waypoint
data = "49.306254,4.593728,20"  # lat,lon,alt
requests.post(f"http://{rc_ip}:8080/send/gotoWP", data=data)
```

#### 3. Launch Ground Control Interface
```bash
python GroundStation/mainLive.py
```

## API Reference

### Telemetry Endpoints (HTTP GET)

| Endpoint | Description | Response |
|----------|-------------|----------|
| `/aircraft/location` | GPS coordinates and altitude | JSON |
| `/aircraft/attitude` | Pitch, roll, yaw values | JSON |
| `/aircraft/speed` | Current velocity | JSON |
| `/aircraft/heading` | Current heading | JSON |
| `/aircraft/allStates` | Complete telemetry package | JSON |
| `/aircraft/battery` | Battery level and status | JSON |

### Control Endpoints (HTTP POST)

| Endpoint | Description | Parameters |
|----------|-------------|------------|
| `/send/takeoff` | Initiate takeoff | None |
| `/send/land` | Initiate landing | None |
| `/send/RTH` | Return to home | None |
| `/send/gotoWP` | Navigate to waypoint | `lat,lon,alt` |
| `/send/gotoYaw` | Rotate to heading | `yaw_angle` |
| `/send/stick` | Virtual stick input | `pitch,roll,yaw,throttle` |
| `/send/camera/zoom` | Camera zoom control | `zoom_ratio` |
| `/send/gimbal/pitch` | Gimbal pitch control | `pitch_angle` |

### Video Streaming
- **RTSP URL**: `rtsp://aaa:aaa@{RC_IP}:8554/streaming/live/1`
- **Format**: H.264, Standard Definition
- **Latency**: 1.4-1.9 seconds (depending on network)

## Project Structure

```
WildBridge/
├── GroundStation/                  # Python ground control system
│   ├── djiInterface.py            # Full DJI communication API
│   ├── djiInterfaceLite.py        # Lightweight interface
│   ├── mainLive.py               # Live mission control
│   ├── objectDetection.py        # YOLO-based detection
│   ├── objectPosition.py         # 3D position estimation
│   ├── dataLogger.py             # Flight data logging
│   └── utils/                    # Utility scripts
├── SampleCode-V5/                 # Android application
│   ├── android-sdk-v5-as/        # Main app project
│   ├── android-sdk-v5-sample/    # Sample implementations
│   └── android-sdk-v5-uxsdk/     # UI components
├── localisation_data/            # Research datasets
└── paper.tex                     # Academic paper source
```

## Development Guide

### Android App Development

**Development Environment**:
- Android Studio Koala 2024.1.1
- DJI Mobile SDK V5
- Kotlin/Java

**Key Implementation**: [`VirtualStickFragment.kt`](SampleCode-V5/android-sdk-v5-as/src/main/java/dji/sampleV5/moduleaircraft/pages/VirtualStickFragment.kt)

**Building from Source**:
1. Clone repository
2. Open `SampleCode-V5/android-sdk-v5-as` in Android Studio
3. Configure DJI SDK keys in `AndroidManifest.xml`
4. Build and deploy to RC Pro

### Ground Station Development

**Core APIs**:
```python
from GroundStation.djiInterfaceLite import DJIInterfaceLite

# Initialize connection
dji = DJIInterfaceLite("192.168.1.100")

# Mission execution
dji.requestSendTakeOff()
waypoints = [(lat1, lon1, alt1), (lat2, lon2, alt2)]
dji.requestSendNavigateTrajectory(waypoints, final_yaw)
```

**Adding Custom Detection**:
```python
from GroundStation.objectDetection import ObjectDetectionWorld

detector = ObjectDetectionWorld(
    droneInterface, 
    classes=["elephant", "rhino", "bird"]
)
```

## Scientific Applications

WildBridge has been validated in multiple research domains:

- **Wildlife Conservation**: Real-time animal detection and geolocation
- **Wildfire Detection**: Early fire detection and mapping
- **Atmospheric Research**: Wind field profiling and measurement
- **Multi-drone Coordination**: Swarm-based data collection
- **Conservation Monitoring**: Long-term ecosystem studies

## Limitations and Considerations

### Technical Limitations
- **Video Scalability**: Maximum 6 concurrent video streams
- **Telemetry Rate**: Optimal performance up to 32Hz request rate
- **Synchronization**: Video and telemetry streams are not synchronized
- **SDK Dependency**: Relies on DJI Mobile SDK V5 evolution

### Operational Considerations
- **Setup Time**: Multi-drone configurations require network setup
- **Environmental Factors**: Performance affected by Wi-Fi interference
- **Hardware Heating**: Extended operations may cause overheating
- **Data Synchronization**: Post-mission data alignment requires planning

## Troubleshooting

### Common Issues

**Connection Problems**:
- Verify RC IP address in network settings
- Check Wi-Fi network compatibility (5GHz preferred)
- Ensure WildBridge app is running and Virtual Stick enabled

**Video Stream Issues**:
- Test RTSP URL in VLC: `rtsp://aaa:aaa@{RC_IP}:8554/streaming/live/1`
- Check network bandwidth for multiple streams
- Verify firewall settings on ground station

**Performance Issues**:
- Reduce telemetry request frequency below 32Hz
- Limit concurrent video streams to 6 or fewer
- Use dedicated 5GHz Wi-Fi network when possible

### Debug Commands
```bash
# Test connectivity
ping {RC_IP}

# Test video stream
ffplay rtsp://aaa:aaa@{RC_IP}:8554/streaming/live/1

# Monitor telemetry
curl http://{RC_IP}:8080/aircraft/allStates
```

## Research and Citation

This work is part of the WildDrone project, funded by the European Union's Horizon Europe Research Program (Grant Agreement No. 101071224).

**Academic Papers**:
```bibtex
@inproceedings{rolland2025wildbridge,
    title={WildBridge: Ground Station Interface for Lightweight Multi-Drone Control and Telemetry on DJI Platforms},
    author={Rolland, Edouard G. A. and Meier, Kilian and Bronz, Murat and Shrikhande, Aditya M. and Richardson, Tom and Lundquist, Ulrik P. S. and Christensen, Anders L.},
    year={2025},
    note={Submitted for publication}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for details.

## Acknowledgments

- **WildDrone Consortium** - European research collaboration
- **University of Bristol** - Lead development institution  
- **University of Southern Denmark** - Multi-drone coordination
- **DJI** - Mobile SDK V5 platform support
- **Ultralytics** - YOLO detection models

## Contributing

Contributions are welcome! Please see our guidelines:

1. **Bug Reports**: Use GitHub issues with reproduction steps
2. **Feature Requests**: Describe use case and scientific application
3. **Code Standards**: Follow PEP 8 (Python) and Kotlin conventions
4. **Testing**: Include unit tests for new features
5. **Documentation**: Update README and code comments

For questions or collaboration inquiries, please contact the WildDrone consortium at [https://wilddrone.eu](https://wilddrone.eu).
