"""
WildBridge - DJI Interface Module

A Python interface for controlling DJI drones through HTTP requests, providing
seamless integration for drone operations, telemetry retrieval, and video streaming.

Authors: Kilian Meier, Edouard G.A. Rolland
Project: WildDrone - Autonomous Search and Rescue Operations
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import cv2
import requests
import ast
from datetime import datetime

# Aircraft state endpoint suffixes
# GETTER
EP_BASE = "/"
EP_SPEED = "/aircraft/speed"
EP_HEADING = "/aircraft/heading"
EP_ATTITUDE = "/aircraft/attitude"
EP_LOCATION = "/aircraft/location"
EP_GIMBAL_ATTITUDE = "/aircraft/gimbalAttitude"
EP_ALL_STATES = "/aircraft/allStates"
EP_STICK_VALUES = "/aircraft/rcStickValues"
EP_YAW_REACHED = "/status/yawReached"
EP_ALTITUDE_REACHED = "/status/altitudeReached"
EP_WP_REACHED = "/status/waypointReached"
EP_HOME_LOCATION = "/home/location"
EP_CAMERA_IS_RECORDING = "/status/camera/isRecording"

# SETTER
EP_STICK = "/send/stick" # expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"  # !!! This is the yaw joint angle !!!
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_INTERMEDIARY_WP_REACHED = "/status/intermediaryWaypointReached"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"

#PID Tuninng
EP_TUNING = "/send/gotoWPwithPIDtuning"


class DJIInterface:
    def __init__(self, IP_RC=""):
        self.IP_RC = IP_RC
        self.baseTelemUrl = f"http://{IP_RC}:8080"
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"

    def getVideoSource(self):
        if self.IP_RC == "":
            return ""
        return self.videoSource

    def requestGet(self, endPoint, verbose=False):
        if self.IP_RC == "":
            return ""
        response = requests.get(self.baseTelemUrl + endPoint)
        if verbose:
            print("EP : " + endPoint + "\t" +
                  str(response.content, encoding="utf-8"))
        return response.content.decode('utf-8')

    def requestSend(self, endPoint, data, verbose=False):
        if self.IP_RC == "":
            print(
                f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        response = requests.post(self.baseTelemUrl + endPoint, str(data))
        if verbose:
            print("EP : " + endPoint + "\t" +
                  str(response.content, encoding="utf-8"))
        return response.content.decode('utf-8')

    def requestAllStates(self, verbose=False):
        response = self.requestGet(EP_ALL_STATES, verbose)
        try:
            # TODO: probably very unsafe!!!
            states = ast.literal_eval(response)
            states["timestamp"] = datetime.now().strftime(
                "%Y-%m-%d_%H-%M-%S.%f")
            return states
        except:
            return {}

    def requestSendStick(self, leftX=0, leftY=0, rightX=0, rightY=0):
        # Saturate values such that they are in [-1;1]
        s = 0.3
        leftX = max(-s, min(s, leftX))
        leftY = max(-s, min(s, leftY))
        rightX = max(-s, min(s, rightX))
        rightY = max(-s, min(s, rightY))
        rep = self.requestSend(
            EP_STICK, f"{leftX:.4f},{leftY:.4f},{rightX:.4f},{rightY:.4f}")
        return rep

    def requestSendGimbalPitch(self, pitch=0):
        return self.requestSend(EP_GIMBAL_SET_PITCH, f"0,{pitch},0")

    def requestSendGimbalYaw(self, yaw=0):
        return self.requestSend(EP_GIMBAL_SET_YAW, f"0,0,{yaw}")

    def requestSendZoomRatio(self, zoomRatio=1):
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWP(self, latitude, longitude, altitude):
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw):
        return self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw}")
    
    def requestSendGoToWPwithPIDtuning(self, latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw):
        return self.requestSend(EP_TUNING, f"{latitude},{longitude},{altitude},{yaw},{kp_pos},{ki_pos},{kd_pos},{kp_yaw},{ki_yaw},{kd_yaw}")

    def requestSendNavigateTrajectory(self, waypoints, finalYaw):
        """
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param finalYaw: The final yaw angle at the last waypoint.
        :return: The response from the server.
        """
        self.requestSendEnableVirtualStick()
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message
        # All waypoints except the last: "lat,lon,alt"
        # Last waypoint: "lat,lon,alt,yaw"
        segments = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                # Intermediary waypoint: lat,lon,alt
                segments.append(f"{lat},{lon},{alt}")
            else:
                # Last waypoint: lat,lon,alt,yaw
                segments.append(f"{lat},{lon},{alt},{finalYaw}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY, message)
    
    def requestSendNavigateTrajectoryDJINative(self, waypoints):
        """
        Send waypoints to be executed using DJI's native waypoint mission system.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :return: The response from the server.
        """
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message format: "lat,lon,alt; lat,lon,alt; ..."
        segments = []
        for lat, lon, alt in waypoints:
            segments.append(f"{lat},{lon},{alt}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY_DJI_NATIVE, message)
    
    def requestAbortDJINativeMission(self):
        return self.requestSend(EP_ABORT_DJI_NATIVE_MISSION, "")

    def requestSticks(self):
        return self.requestGet(EP_STICK_VALUES, True)

    def requestAbortMission(self):
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestWaypointStatus(self):
        return self.requestGet(EP_WP_REACHED)

    def requestIntermediaryWaypointStatus(self):
        return self.requestGet(EP_INTERMEDIARY_WP_REACHED)

    def requestSendEnableVirtualStick(self):
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestYawStatus(self):
        return self.requestGet(EP_YAW_REACHED)

    def requestSendGotoYaw(self, yaw):
        return self.requestSend(EP_GOTO_YAW, f"{yaw}")

    def requestSendGotoAltitude(self, altitude):
        return self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}")

    def requestAltitudeStatus(self):
        return self.requestGet(EP_ALTITUDE_REACHED)

    def requestHomePosition(self):
        response = self.requestGet(EP_HOME_LOCATION, False)
        try:
            # TODO: probably very unsafe!!!
            home = ast.literal_eval(response)
            return home
        except:
            return {}

    def requestCameraStartRecording(self):
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")

    def requestCameraIsRecording(self):
        return self.requestGet(EP_CAMERA_IS_RECORDING) == "true"

if __name__ == '__main__':
    IP_RC = "10.100.67.235" # REPLACE WITH YOUR RC IP

    print(f"Connecting to {IP_RC}...")
    dji = DJIInterface(IP_RC)

    print(f"Requesting all states...")
    response = dji.requestAllStates()
    for key, value in response.items():
        print(f"{key}: {value}")

    print(f"Sending takeoff command...")
    dji.requestSendTakeOff()

    print(f"Saving frame...")
    cap = cv2.VideoCapture(dji.getVideoSource())
    ret, frame = cap.read()
    if ret:
        cv2.imwrite("frame.jpg", frame)
    cap.release()