import requests
import ast
import sys
from datetime import datetime
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from objectDetection import ObjectDetection
import math

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
# expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_STICK = "/send/stick"
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

#PID Tuninng
EP_TUNING = "/send/gotoWPwithPIDtuning"


class DJIInterfaceLite:
    def __init__(self, IP_RC=""):
        self.IP_RC = IP_RC
        self.baseTelemUrl = f"http://{IP_RC}:8080"
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"

    def getVideoSource(self):
        if self.IP_RC == "":
            # print("No IP_RC provided, returning empty string")
            return ""
        return self.videoSource

    def requestGet(self, endPoint, verbose=False):
        if self.IP_RC == "":
            # print(f"No IP_RC provided, returning empty string for request at {endPoint}")
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


class DJIControllerLite():
    def __init__(self, droneInterface):
        self.droneInterface = droneInterface
        self.GAIN_P = 1e-2
        self.prevLat = None
        self.prevLon = None
        self.prevAlt = None
        self.prevYaw = None

    def center(self, xPixelOffset, yPixelOffset):
        # image x axis is going from left to right,
        # y is going from top to bottom,
        # origin is at the center of image
        ctrlX = xPixelOffset*self.GAIN_P
        ctrlY = -yPixelOffset*self.GAIN_P
        self.droneInterface.requestSendStick(rightX=ctrlX, rightY=ctrlY)

    def waitGPSFix(self):
        # Wait for GPS fix before starting waypoint navigation
        while True:
            if self.droneInterface.requestAllStates(True)["location"]["latitude"] == 0:
                print("Waiting for GPS fix...")
            else:
                break

    def gotoYaw(self, targetYaw):
        if targetYaw > 179 or targetYaw < -179:
            raise ValueError("Yaw must be in the range [-179, 179]")
        self.droneInterface.requestSendGotoYaw(targetYaw)
        while dji.requestYawStatus() == "false":  # Wait for the yaw to be reached
            time.sleep(0.1)

    def gotoWaypointPIDAsynch(self, latitude, longitude, altitude, yaw):
        if self.prevLat == latitude and self.prevLon == longitude and self.prevAlt == altitude and self.yaw == yaw:  # Not a new waypoint
            # Check if the waypoint has been reached
            return self.droneInterface.requestWaypointStatus() == "true"
        else:
            self.prevLat = latitude
            self.prevLon = longitude
            self.prevAlt = altitude
            self.yaw = yaw
            self.droneInterface.requestSendGoToWPwithPID(
                latitude, longitude, altitude, yaw)
            return False

    def gotoYawAsynch(self, targetYaw):
        if targetYaw > 179 or targetYaw < -179:
            raise ValueError("Yaw must be in the range [-179, 179]")

        if self.prevYaw == targetYaw:  # Not a new yaw
            # Check if the yaw has been reached
            return self.droneInterface.requestYawStatus() == "true"
        else:
            self.prevYaw = targetYaw
            self.droneInterface.requestSendGotoYaw(targetYaw)
            return False

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


def generate_spiral(center_lat, center_lon, radius_m, alt_start, alt_end, revolutions, points_per_rev):
    """
    Generates an ascending spiral around (center_lat, center_lon).
    - radius_m: max radius in meters
    - alt_start, alt_end: start and end altitude (m)
    - revolutions: number of full turns
    - points_per_rev: point density per turn
    """
    waypoints = []
    total_pts = revolutions * points_per_rev
    for i in range(total_pts + 1):
        # current angle
        theta = 2 * math.pi * (i / points_per_rev)
        # radius increases linearly
        r = radius_m * (i / total_pts)
        # offsets in meters
        dx = r * math.cos(theta)
        dy = r * math.sin(theta)
        # conversion to degrees
        dlat = dx / 111_320          # approx. 1° lat ~ 111.32 km
        dlon = dy / (111_320 * math.cos(math.radians(center_lat)))
        lat = center_lat + dlat
        lon = center_lon + dlon
        alt = alt_start + (alt_end - alt_start) * (i / total_pts)
        waypoints.append((lat, lon, alt))
    return waypoints

if __name__ == '__main__':

    # Get the RC IP (or default value)
    IP_RC = sys.argv[1] if len(sys.argv) == 2 else "10.100.67.235"
    dji = DJIInterfaceLite(IP_RC)

    # Spiral parameters
    center_lat = 55.367889
    center_lon = 10.435056
    radius_m = 40           # radius in meters
    alt_start = 10          # start altitude (m)
    alt_end = 60            # end altitude (m)
    revolutions = 6         # number of turns
    pts_per_rev = 10        # points per turn
    final_yaw = 180.0       # final heading (°)


    # Generate spiral waypoints
    waypoints = generate_spiral(
        center_lat, center_lon,
        radius_m, alt_start, alt_end,
        revolutions, pts_per_rev
    )

    # Save the trajectory to a txt file (including yaw)
    with open("spiral_trajectory.txt", "w") as f:
        for i, (lat, lon, alt) in enumerate(waypoints):
            # Interpolate yaw from 0 to final_yaw
            yaw = final_yaw * (i / (len(waypoints) - 1))
            f.write(f"{lat},{lon},{alt},{yaw}\n")
    print("Trajectory saved to spiral_trajectory.txt")

    # Prepare data for the target trajectory
    wp_lats = [wp[0] for wp in waypoints]
    wp_lons = [wp[1] for wp in waypoints]
    wp_alts = [wp[2] for wp in waypoints]

    # Enable Virtual Stick
    print("Enabling Virtual Stick…")
    dji.requestSendEnableVirtualStick()
    time.sleep(1)

    # Start the trajectory
    print("Starting ascending spiral trajectory…")
    resp = dji.requestSendNavigateTrajectory(waypoints, final_yaw)
    print("Response navigateTrajectory:", resp)

    # Prepare the plot
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(wp_lats, wp_lons, wp_alts, 'r--', label='Target trajectory')
    drone_lats, drone_lons, drone_alts = [], [], []
    drone_yaws = []
    drone_path, = ax.plot([], [], [], 'b-', label='Drone trajectory')
    ax.set_xlabel('Latitude')
    ax.set_ylabel('Longitude')
    ax.set_zlabel('Altitude (m)')
    ax.legend()
    plt.title("Drone trajectory vs target (spiral)")

    # Open log file for more frequent logging
    log_file = open("drone_spiral_flight.txt", "w")
    log_file.write("lat,lon,alt,yaw,timestamp\n")

    # Animation function
    def update(frame):
        states = dji.requestAllStates()
        loc = states.get("location", {})
        att = states.get("attitude", {})
        lat = loc.get("latitude", None)
        lon = loc.get("longitude", None)
        alt = loc.get("altitude", None)
        yaw = att.get("yaw", None)
        timestamp = states.get("timestamp", datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f"))
        if lat is not None and lon is not None and alt is not None:
            drone_lats.append(lat)
            drone_lons.append(lon)
            drone_alts.append(alt)
            if yaw is not None:
                drone_yaws.append(yaw)
            else:
                drone_yaws.append(float('nan'))
            # Log every datapoint with timestamp
            log_file.write(f"{lat},{lon},{alt},{yaw},{timestamp}\n")
            log_file.flush()
            drone_path.set_data(drone_lats, drone_lons)
            drone_path.set_3d_properties(drone_alts)
        ax.set_xlim(min(wp_lats)-0.0005, max(wp_lats)+0.0005)
        ax.set_ylim(min(wp_lons)-0.0005, max(wp_lons)+0.0005)
        ax.set_zlim(min(wp_alts)-5, max(wp_alts)+5)
        return drone_path,

    # Wait for the end of the trajectory with real-time display
    print("Waiting for the end of the trajectory…")
    ani = animation.FuncAnimation(fig, update, interval=200, blit=False)  # log every 200ms
    plt.show(block=False)

    """
    #Tuning
    dji.requestSendGoToWPwithPIDtuning(latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw)
    """ 
    
    while True:
        done = dji.requestWaypointStatus()
        print("WaypointReached =", done)
        if done == "true":
            print("Spiral finished ✅")
            log_file.close()
            print("Drone trajectory with yaw saved to drone_spiral_flight.txt")
            break
        plt.pause(0.1)

    # Save the animation as GIF
    ani.save("drone_spiral.gif", writer='pillow', fps=10)
    print("Animation saved as drone_spiral.gif")

    # Example post-mission action
    print("Landing…")
