import requests
import ast
import sys
from datetime import datetime
import time
import time
import matplotlib.pyplot as plt

# Aircraft state endpoints suffixes
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
EP_WP_REACHED = "/status/waypointReached"

# SETTER
# expects a sting formated as: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_STICK = "/send/stick"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_ALTITUDE_REACHED = "/status/altitudeReached"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"

EP_CAMERA_IS_RECORDING = "/status/camera/isRecording"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"



class DJIInterfaceLite:
    def __init__(self, IP_RC):
        self.IP_RC = IP_RC
        self.baseTelemUrl = f"http://{IP_RC}:8080"
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"

    def getVideoSource(self):
        return self.videoSource

    def requestGet(self, endPoint, verbose=False):
        response = requests.get(self.baseTelemUrl + endPoint)
        if verbose:
            print("EP : " + endPoint + "\t" +
                  str(response.content, encoding="utf-8"))
        return response.content.decode('utf-8')

    def requestSend(self, endPoint, data, verbose=False):
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

    def requestSendZoomRatio(self, zoomRatio=1):
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        self.requestAbortMission()
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWP(self, latitude, longitude, altitude):
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw):
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw}")

    def requestSendNavigateTrajectory(self, waypoints: dict):
        """
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param finalYaw: The final yaw angle at the last waypoint.
        :return: The response from the server.
        """

        self.requestSendEnableVirtualStick()

        if not waypoints:
            raise ValueError("No waypoints provided")

        if len(waypoints) == 1:
            waypoint_data = waypoints[0]
            self.requestSendGoToWPwithPID(waypoint_data.get('lat'), waypoint_data.get(
                'lng'), waypoint_data.get('y'), waypoint_data.get('final_heading'))

        else:
            # Build the message
            # All waypoints except the last: "lat,lon,alt"
            # Last waypoint: "lat,lon,alt,yaw"
            segments = []
            for i, waypoint in enumerate(waypoints):
                lat = waypoint.get('lat')
                lon = waypoint.get('lng')
                alt = waypoint.get('y')
                finalYaw = waypoint.get('final_heading')

                if i < len(waypoints) - 1:
                    # Intermediary waypoint: lat,lon,alt
                    segments.append(f"{lat},{lon},{alt}")
                else:
                    # Last waypoint: lat,lon,alt,yaw
                    segments.append(f"{lat},{lon},{alt},{finalYaw}")

            message = ";".join(segments)
            return self.requestSend(EP_GOTO_TRAJECTORY, message)

    def requestSticks(self):
        return self.requestGet(EP_STICK_VALUES, True)

    def requestAbortMission(self):
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestWaypointStatus(self):
        return self.requestGet(EP_WP_REACHED)

    def requestSendEnableVirtualStick(self):
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestYawStatus(self):
        return self.requestGet(EP_YAW_REACHED)

    def requestSendGotoYaw(self, yaw):
        return self.requestSend(EP_GOTO_YAW, f"{yaw}")

    def requestSendGotoAltitude(self, altitude):
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}")

    def requestAltitudeStatus(self):
        return self.requestGet(EP_ALTITUDE_REACHED)

    def requestCameraStartRecording(self):
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")

    def requestCameraIsRecording(self):
        return self.requestGet(EP_CAMERA_IS_RECORDING) == "true"


class DJIControllerLite():
    def __init__(self, droneInterface):
        self.droneInterface = droneInterface
        self.GAIN_P = 1e-2

    def center(self, xPixelOffset, yPixelOffset):
        # image x axis is going from left to right,
        # y is going from top to bottom,
        # origin is at the center of image
        ctrlX = xPixelOffset*self.GAIN_P
        ctrlY = -yPixelOffset*self.GAIN_P
        self.droneInterface.requestSendStick(rightX=ctrlX, rightY=ctrlY)

    def gotoYaw(self, targetYaw):
        if targetYaw > 179 or targetYaw < -179:
            raise ValueError("Yaw must be in the range [-179, 179]")
        self.droneInterface.requestSendGotoYaw(targetYaw)
        while dji.requestYawStatus() == "false":  # Wait for the yaw to be reached
            time.sleep(0.1)

    def waitGPSFix(self):
        # Wait for GPS fix before starting waypoint navigation
        while True:
            if self.droneInterface.requestAllStates(True)["location"]["latitude"] == 0:
                print("Waiting for GPS fix...")
            else:
                break

    def gotoWaypoint(self, latitude, longitude, altitude):
        self.droneInterface.requestSendGoToWP(latitude, longitude, altitude)
        while self.droneInterface.requestWaypointStatus() == "false":
            time.sleep(0.1)
        print("Waypoint reached")

    def gotoWaypointwithPID(self, latitude, longitude, altitude, yaw):
        self.droneInterface.requestSendGoToWPwithPID(
            latitude, longitude, altitude, yaw)
        while self.droneInterface.requestWaypointStatus() == "false":
            time.sleep(0.1)
        print("Waypoint reached")

    def gotoAltitude(self, altitude):
        self.droneInterface.requestSendGotoAltitude(altitude)
        while self.droneInterface.requestAltitudeStatus() == "false":
            time.sleep(0.1)
        print("Altitude reached")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python djiInterfaceLite.py <IP_RC>")
    else:
        IP_RC = sys.argv[1]
        dji = DJIInterfaceLite(IP_RC)
        dji.requestGet(EP_BASE, True)

        controller = DJIControllerLite(dji)

        while True:
            states = dji.requestAllStates(True)
