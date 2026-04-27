#!/usr/bin/env python3
"""
WebRTC Drone Video Viewer

This script connects to the DJI drone's WebRTC server and displays the video feed.
It can also save frames or the video stream to disk.

Requirements:
    pip install aiortc aiohttp opencv-python numpy websockets

Usage:
    python webrtc_drone_viewer.py --server ws://192.168.1.100:8081
    python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --save-video output.mp4
    python webrtc_drone_viewer.py --server ws://192.168.1.100:8081 --headless --save-frames ./frames/
"""

import argparse
import asyncio
import json
import logging
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRecorder
from av import VideoFrame

# Add parent directory to path to import djiInterface
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Python'))
try:
    from djiInterface import discover_drone
except ImportError:
    discover_drone = None

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class DroneVideoReceiver:
    """Handles WebRTC connection and video frame processing."""
    
    def __init__(self, server_url: str, headless: bool = False, 
                 save_video: str = None, save_frames: str = None):
        self.server_url = server_url
        self.headless = headless
        self.save_video = save_video
        self.save_frames = save_frames
        
        self.pc = None
        self.websocket = None
        self.client_id = None
        self.connected = False
        self.frame_count = 0
        self.start_time = None
        self.latest_frame = None
        self.recorder = None
        
        # Telemetry data from data channel
        self.latest_metadata = None
        self.metadata_lock = asyncio.Lock()
        
        # Track for keeping connection alive
        self.video_track_task = None
        self.running = True
        
        # Create frames directory if saving frames
        if self.save_frames and not os.path.exists(self.save_frames):
            os.makedirs(self.save_frames)
    
    async def connect(self):
        """Connect to the signaling server and establish WebRTC connection."""
        logger.info(f"Connecting to signaling server: {self.server_url}")
        
        try:
            self.websocket = await websockets.connect(self.server_url)
            logger.info("WebSocket connected")
            
            # Wait for welcome message with timeout
            try:
                welcome = await asyncio.wait_for(self.websocket.recv(), timeout=5.0)
                welcome_data = json.loads(welcome)
                if welcome_data.get('type') == 'welcome':
                    self.client_id = welcome_data.get('clientId')
                    logger.info(f"Assigned client ID: {self.client_id}")
            except asyncio.TimeoutError:
                logger.error("Timeout waiting for welcome message")
                raise
            
            # Register as viewer
            await self.websocket.send(json.dumps({
                'type': 'register',
                'role': 'viewer'
            }))
            logger.info("Registered as viewer")
            
            # Create peer connection
            self.pc = RTCPeerConnection()
            logger.info("RTCPeerConnection created")
            
            # Setup event handlers BEFORE sending register message
            # Handle incoming tracks
            @self.pc.on("track")
            def on_track(track):
                logger.info(f"Received track: {track.kind}")
                if track.kind == "video":
                    logger.info("Starting video track handler...")
                    # Store the task so it stays alive
                    self.video_track_task = asyncio.ensure_future(self._handle_video_track(track))
                elif track.kind == "audio":
                    logger.info("Received audio track (ignoring)")
            
            # Handle connection state changes
            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"Connection state: {self.pc.connectionState}")
                if self.pc.connectionState == "connected":
                    self.connected = True
                    self.start_time = time.time()
                    logger.info("WebRTC connection established!")
                elif self.pc.connectionState in ["failed", "closed", "disconnected"]:
                    self.connected = False
                    logger.warning(f"WebRTC connection {self.pc.connectionState}")
            
            # Handle ICE connection state
            @self.pc.on("iceconnectionstatechange")
            async def on_iceconnectionstatechange():
                logger.info(f"ICE connection state: {self.pc.iceConnectionState}")
            
            # Handle ICE candidates
            @self.pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    logger.debug(f"Sending ICE candidate: {candidate.candidate[:50]}...")
                    await self._send_ice_candidate(candidate)
            
            # Create data channel for metadata with same settings as Android side
            try:
                # Use negotiated mode with specific ID to match Android side
                self.metadata_channel = self.pc.createDataChannel(
                    "telemetry", 
                    negotiated=True,
                    id=0,
                    ordered=True
                )
                logger.info("Created metadata data channel (negotiated mode)")
                
                @self.metadata_channel.on("message")
                def on_metadata_message(message):
                    try:
                        if not message:
                            logger.debug("Received empty metadata message")
                            return
                        self.latest_metadata = json.loads(message)
                        logger.debug(f"Received metadata: frame {self.latest_metadata.get('frameNumber', 'N/A')}")
                    except json.JSONDecodeError as e:
                        logger.error(f"Failed to parse metadata: {e}")
                    except Exception as e:
                        logger.error(f"Error handling metadata message: {e}")
                        
                @self.metadata_channel.on("close")
                def on_metadata_channel_close():
                    logger.warning("Metadata channel closed")
                    self.metadata_channel = None
                    
                @self.metadata_channel.on("error")
                def on_metadata_channel_error(error):
                    logger.error(f"Metadata channel error: {error}")
                    self.metadata_channel = None
                    
            except Exception as e:
                logger.error(f"Error creating metadata channel: {e}")
                self.metadata_channel = None
            
            # Handle incoming data channel for metadata (if Android creates it differently)
            @self.pc.on("datachannel")
            def on_datachannel(channel):
                try:
                    if not channel:
                        logger.warning("Received null data channel")
                        return
                    logger.info(f"Received data channel: {channel.label}")
                    if channel.label == "telemetry":
                        logger.info("Metadata data channel received from peer")
                        self.metadata_channel = channel
                        
                        @channel.on("message")
                        def on_message(message):
                            try:
                                if not message:
                                    logger.debug("Received empty message from peer channel")
                                    return
                                self.latest_metadata = json.loads(message)
                                logger.debug(f"Received metadata from peer: frame {self.latest_metadata.get('frameNumber', 'N/A')}")
                            except json.JSONDecodeError as e:
                                logger.error(f"Failed to parse metadata from peer: {e}")
                            except Exception as e:
                                logger.error(f"Error handling peer metadata message: {e}")
                        
                        @channel.on("close")
                        def on_peer_channel_close():
                            logger.warning("Peer metadata channel closed")
                            
                        @channel.on("error")
                        def on_peer_channel_error(error):
                            logger.error(f"Peer metadata channel error: {error}")
                except Exception as e:
                    logger.error(f"Error handling incoming data channel: {e}")
            
            # Start listening for signaling messages
            logger.info("Waiting for SDP offer from server...")
            await self._handle_signaling()
            
        except Exception as e:
            logger.error(f"Connection error: {e}")
            raise
    
    async def _handle_signaling(self):
        """Handle incoming signaling messages and keep connection alive."""
        try:
            # Create a task for receiving messages
            async def receive_messages():
                async for message in self.websocket:
                    try:
                        data = json.loads(message)
                        msg_type = data.get('type')
                        
                        logger.debug(f"Received signaling message: {msg_type}")
                        
                        if msg_type == 'offer':
                            logger.info("Received offer from server")
                            await self._handle_offer(data)
                        elif msg_type == 'answer':
                            logger.warning("Received unexpected answer (viewer should receive offers)")
                            await self._handle_answer(data)
                        elif msg_type == 'candidate':
                            await self._handle_ice_candidate(data)
                        elif msg_type == 'ping':
                            # Respond to keep-alive pings
                            await self.websocket.send(json.dumps({'type': 'pong'}))
                    except Exception as e:
                        logger.error(f"Error processing signaling message: {e}")
            
            # Run message handler and keep connection alive
            receive_task = asyncio.ensure_future(receive_messages())
            
            # Keep the connection alive while running
            while self.running:
                try:
                    # Just check periodically if we're still supposed to be running
                    await asyncio.sleep(1.0)
                except asyncio.CancelledError:
                    logger.info("Signaling handler cancelled")
                    break
                except Exception as e:
                    logger.error(f"Error in signaling loop: {e}")
                    break
            
            # Clean up
            receive_task.cancel()
            try:
                await receive_task
            except asyncio.CancelledError:
                pass
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("WebSocket connection closed")
        except Exception as e:
            logger.error(f"Signaling error: {e}")
    
    async def _handle_offer(self, data: dict):
        """Handle incoming SDP offer."""
        logger.info("Processing offer...")
        
        offer = RTCSessionDescription(sdp=data['sdp'], type='offer')
        await self.pc.setRemoteDescription(offer)
        
        # Create answer
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        # Send answer
        await self.websocket.send(json.dumps({
            'type': 'answer',
            'sdp': self.pc.localDescription.sdp
        }))
        logger.info("Answer sent")
    
    async def _handle_answer(self, data: dict):
        """Handle incoming SDP answer."""
        logger.info("Processing answer...")
        answer = RTCSessionDescription(sdp=data['sdp'], type='answer')
        await self.pc.setRemoteDescription(answer)
    
    async def _handle_ice_candidate(self, data: dict):
        """Handle incoming ICE candidate.
        
        Note: aiortc doesn't expose a public RTCIceCandidate constructor.
        It handles ICE candidates internally during SDP negotiation.
        For trickle ICE, we would need lower-level access.
        Since aiortc gathers candidates automatically, remote candidates
        received here are logged but the library manages connectivity internally.
        """
        candidate_str = data.get('candidate', '')
        sdp_mid = data.get('id')
        logger.debug(f"Received ICE candidate for mid={sdp_mid}: {candidate_str[:60] if candidate_str else 'empty'}...")
        # aiortc handles ICE gathering and connectivity internally
        # Remote candidates are applied through the SDP exchange
    
    async def _send_ice_candidate(self, candidate):
        """Send ICE candidate to peer."""
        await self.websocket.send(json.dumps({
            'type': 'candidate',
            'id': candidate.sdpMid,
            'label': candidate.sdpMLineIndex,
            'candidate': candidate.candidate
        }))
    
    async def _handle_video_track(self, track):
        """Process incoming video frames."""
        logger.info("Starting video track processing...")
        
        # Log track info safely (RemoteStreamTrack doesn't have codec attribute)
        track_info = f"kind={track.kind}"
        if hasattr(track, 'width') and track.width:
            track_info += f", dimensions={track.width}x{getattr(track, 'height', 'unknown')}"
        logger.info(f"Track info: {track_info}")
        
        # Setup video recorder if saving video
        if self.save_video and self.save_video.strip():
            self.recorder = MediaRecorder(self.save_video)
            self.recorder.addTrack(track)
            await self.recorder.start()
            logger.info(f"Recording video to: {self.save_video}")
        
        frame_warnings = 0
        while self.running:
            try:
                frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                self.frame_count += 1
                
                # Convert to numpy array for OpenCV
                img = frame.to_ndarray(format="bgr24")
                self.latest_frame = img
                
                # Reset frame warning counter on successful frame
                frame_warnings = 0
                
                # Calculate FPS
                if self.start_time and self.frame_count % 30 == 0:
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed
                    logger.info(f"Frames: {self.frame_count}, FPS: {fps:.2f}, Resolution: {img.shape[1]}x{img.shape[0]}")
                
                # Save individual frames if requested
                if self.save_frames and self.frame_count % 10 == 0:  # Save every 10th frame
                    filename = os.path.join(
                        self.save_frames, 
                        f"frame_{self.frame_count:06d}.jpg"
                    )
                    cv2.imwrite(filename, img)
                    logger.debug(f"Saved frame: {filename}")
                    
                    # Save metadata alongside frame
                    if self.latest_metadata:
                        meta_filename = os.path.join(
                            self.save_frames,
                            f"frame_{self.frame_count:06d}_meta.json"
                        )
                        try:
                            with open(meta_filename, 'w') as f:
                                json.dump(self.latest_metadata, f, indent=2)
                            logger.debug(f"Saved metadata: {meta_filename}")
                        except Exception as e:
                            logger.error(f"Error saving metadata: {e}")
                
                # Display frame if not headless
                if not self.headless:
                    # Add overlay with stats
                    overlay = img.copy()
                    elapsed = time.time() - self.start_time if self.start_time else 0
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    
                    # Basic stats
                    cv2.putText(overlay, f"FPS: {fps:.1f}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Frames: {self.frame_count}", (10, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Resolution: {img.shape[1]}x{img.shape[0]}", (10, 110),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Telemetry overlay from metadata
                    if self.latest_metadata:
                        meta = self.latest_metadata
                        y_offset = 150
                        
                        # Frame number
                        frame_num = meta.get('frameNumber', 'N/A')
                        cv2.putText(overlay, f"Frame: {frame_num}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                        y_offset += 30
                        
                        # GPS data
                        lat = meta.get('latitude', 0)
                        lon = meta.get('longitude', 0)
                        alt_asl = meta.get('altitudeASL', 0)
                        alt_agl = meta.get('altitudeAGL', 0)
                        if lat != 0 or lon != 0:
                            cv2.putText(overlay, f"GPS: {lat:.6f}, {lon:.6f}", (10, y_offset),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            y_offset += 30
                            cv2.putText(overlay, f"Alt: ASL {alt_asl:.1f}m AGL {alt_agl:.1f}m", (10, y_offset),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            y_offset += 30
                        
                        # Gimbal data
                        gimbal_pitch = meta.get('gimbalPitch', 0)
                        gimbal_yaw = meta.get('gimbalYaw', 0)
                        gimbal_roll = meta.get('gimbalRoll', 0)
                        cv2.putText(overlay, f"Gimbal P:{gimbal_pitch:.1f} Y:{gimbal_yaw:.1f} R:{gimbal_roll:.1f}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                        y_offset += 30
                        
                        # Aircraft attitude
                        pitch = meta.get('aircraftPitch', 0)
                        yaw = meta.get('aircraftYaw', 0)
                        roll = meta.get('aircraftRoll', 0)
                        cv2.putText(overlay, f"Attitude P:{pitch:.1f} Y:{yaw:.1f} R:{roll:.1f}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                        y_offset += 30
                        
                        # Velocity data
                        vx = meta.get('velocityX', 0)
                        vy = meta.get('velocityY', 0)
                        vz = meta.get('velocityZ', 0)
                        speed = (vx**2 + vy**2 + vz**2)**0.5
                        cv2.putText(overlay, f"Speed: {speed:.1f} m/s", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                        y_offset += 30
                        
                        # Heading and battery
                        heading = meta.get('aircraftYaw', 0)  # Yaw is heading
                        battery = meta.get('batteryPercent', 0)
                        cv2.putText(overlay, f"Heading: {heading:.1f} deg", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                        y_offset += 30
                        
                        # Battery with color coding
                        battery_color = (0, 255, 0) if battery > 30 else (0, 165, 255) if battery > 15 else (0, 0, 255)
                        cv2.putText(overlay, f"Battery: {battery}%", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, battery_color, 2)
                        y_offset += 30
                        
                        # Satellite count
                        sats = meta.get('satelliteCount', 0)
                        gps_color = (0, 255, 0) if sats > 10 else (0, 165, 255) if sats > 5 else (0, 0, 255)
                        cv2.putText(overlay, f"Sats: {sats}", (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, gps_color, 2)
                    
                    cv2.imshow("DJI Drone Video Feed", overlay)
                    
                    # Check for quit key
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        logger.info("Quit requested")
                        break
                    elif key == ord('s'):
                        # Save current frame
                        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                        cv2.imwrite(filename, img)
                        logger.info(f"Saved snapshot: {filename}")
                        
            except asyncio.TimeoutError:
                frame_warnings += 1
                if frame_warnings % 5 == 0:  # Log every 5 timeouts
                    logger.warning(f"No frames received for 5 seconds (timeouts: {frame_warnings})")
                if frame_warnings > 12:  # More than 60 seconds
                    logger.error("No frames received for over 60 seconds, closing")
                    break
            except Exception as e:
                logger.error(f"Error processing frame: {e}")
                break
        
        # Cleanup
        if self.recorder:
            await self.recorder.stop()
        if not self.headless:
            cv2.destroyAllWindows()
        logger.info(f"Video track handler finished after {self.frame_count} frames")
    
    async def close(self):
        """Close all connections with proper null checks."""
        self.running = False
        try:
            if self.video_track_task and not self.video_track_task.done():
                try:
                    self.video_track_task.cancel()
                    await self.video_track_task
                except (asyncio.CancelledError, Exception) as e:
                    if not isinstance(e, asyncio.CancelledError):
                        logger.error(f"Error cancelling video task: {e}")
            
            if self.recorder:
                try:
                    await self.recorder.stop()
                except Exception as e:
                    logger.error(f"Error stopping recorder: {e}")
            
            if self.metadata_channel:
                try:
                    # Close data channel if still open
                    if hasattr(self.metadata_channel, 'close'):
                        self.metadata_channel.close()
                except Exception as e:
                    logger.error(f"Error closing metadata channel: {e}")
                finally:
                    self.metadata_channel = None
            
            if self.pc:
                try:
                    await self.pc.close()
                except Exception as e:
                    logger.error(f"Error closing peer connection: {e}")
                finally:
                    self.pc = None
            
            if self.websocket:
                try:
                    await self.websocket.close()
                except Exception as e:
                    logger.error(f"Error closing websocket: {e}")
                finally:
                    self.websocket = None
            
            if not self.headless:
                try:
                    cv2.destroyAllWindows()
                except Exception as e:
                    logger.error(f"Error destroying windows: {e}")
            
            logger.info("Connections closed successfully")
        except Exception as e:
            logger.error(f"Error in close(): {e}")


async def main():
    parser = argparse.ArgumentParser(description='WebRTC Drone Video Viewer')
    parser.add_argument('--server', '-s', required=False,
                       help='WebSocket server URL (e.g., ws://192.168.1.100:8082). If not provided, auto-discovery will be attempted.')
    parser.add_argument('--headless', action='store_true',
                       help='Run without displaying video (for servers)')
    parser.add_argument('--save-video', '-v', type=str,
                       help='Save video to file (e.g., output.mp4)')
    parser.add_argument('--save-frames', '-f', type=str,
                       help='Save frames to directory')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
    
    server_url = args.server
    if not server_url:
        if discover_drone:
            logger.info("No server URL provided. Attempting auto-discovery...")
            drone_ip = discover_drone(timeout=5.0)
            if drone_ip:
                server_url = f"ws://{drone_ip}:8082"
                logger.info(f"Discovered drone at {server_url}")
            else:
                logger.error("Failed to discover drone. Please provide --server URL.")
                sys.exit(1)
        else:
            logger.error("Auto-discovery not available (djiInterface module not found). Please provide --server URL.")
            sys.exit(1)
    
    logger.info(f"Connecting to: {server_url}")
    logger.info(f"Headless mode: {args.headless}")
    if args.save_video:
        logger.info(f"Saving video to: {args.save_video}")
    if args.save_frames:
        logger.info(f"Saving frames to: {args.save_frames}")
    
    receiver = DroneVideoReceiver(
        server_url=server_url,
        headless=args.headless,
        save_video=args.save_video,
        save_frames=args.save_frames
    )
    
    try:
        await receiver.connect()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        receiver.running = False
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        receiver.running = False
        await receiver.close()


if __name__ == '__main__':
    print("""
    ╔══════════════════════════════════════════════════════════════╗
    ║           DJI Drone WebRTC Video Viewer                      ║
    ║                                                              ║
    ║  Controls:                                                   ║
    ║    Q - Quit                                                  ║
    ║    S - Save snapshot                                         ║
    ║                                                              ║
    ╚══════════════════════════════════════════════════════════════╝
    """)
    
    asyncio.run(main())
