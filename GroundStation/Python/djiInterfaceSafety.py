"""
WildBridge - DJI Safety Interface Module

Safety Computer variant of the DJI interface. It subclasses djiInterface.DJIInterface
WITHOUT modifying it, and injects the X-Safety-Token header into every command so the
Android app treats this client as the Safety Computer: its first command seizes
persistent control of the drone and locks out the Pilot Computer until
requestReleaseSafetyControl() is called.

Use djiInterface.DJIInterface for the Pilot Computer (no token) and
djiInterfaceSafety.DJIInterfaceSafety for the Safety Computer.

Authors: Edouard G.A. Rolland, Kilian Meier
Project: WildDrone
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import requests
from datetime import datetime

from djiInterface import DJIInterface, EP_CAPTURE_THERMAL_IMAGE

# Must match SAFETY_TOKEN hardcoded in the Android app (WildBridgeDefaultLayoutActivity).
SAFETY_TOKEN = "98"
SAFETY_TOKEN_HEADER = "X-Safety-Token"
EP_RELEASE_SAFETY_CONTROL = "/releaseSafetyControl"


class DJIInterfaceSafety(DJIInterface):
    """
    Safety Computer interface: a DJIInterface that always sends the Safety token.

    Same API as DJIInterface — every command (takeoff, RTH, waypoints, thermal capture,
    etc.) is automatically authenticated as the Safety Computer. The first command seizes
    control; call requestReleaseSafetyControl() to hand authority back to the Pilot.

    Nothing in djiInterface.py is modified: the token is injected by overriding the two
    methods that issue HTTP requests (requestSend and requestCaptureThermalImage).
    """

    def __init__(self, IP_RC="", safety_token=SAFETY_TOKEN):
        super().__init__(IP_RC)
        self.safety_token = safety_token

    def setSafetyToken(self, token):
        """Set (or clear, with None) the Safety Computer token sent on every command."""
        self.safety_token = token

    def _authHeaders(self):
        """Return the X-Safety-Token header dict when a token is configured."""
        if self.safety_token:
            return {SAFETY_TOKEN_HEADER: str(self.safety_token)}
        return {}

    # --- Overrides that inject the token (parent versions send no headers) ---

    def requestSend(self, endPoint, data, verbose=False):
        """Send a POST request to the drone, authenticated as the Safety Computer."""
        if self.IP_RC == "":
            print(f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        try:
            response = requests.post(
                self.baseCommandUrl + endPoint, str(data),
                headers=self._authHeaders(), timeout=5)
            if verbose:
                print("EP : " + endPoint + "\t" + str(response.content, encoding="utf-8"))
            return response.content.decode('utf-8')
        except requests.exceptions.RequestException as e:
            print(f"Request error at {endPoint}: {e}")
            return ""

    def requestCaptureThermalImage(self, save_path=None):
        """Capture a thermal image, authenticated as the Safety Computer.

        Mirrors DJIInterface.requestCaptureThermalImage but adds the X-Safety-Token header.
        """
        if self.IP_RC == "":
            print("No IP_RC provided, cannot capture thermal image")
            return False

        if save_path is None:
            save_path = f"thermal_image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"

        try:
            response = requests.post(
                self.baseCommandUrl + EP_CAPTURE_THERMAL_IMAGE, data="",
                headers=self._authHeaders(), timeout=90)

            content_type = response.headers.get("Content-Type", "")
            if response.status_code == 200 and content_type.startswith("image/"):
                with open(save_path, "wb") as f:
                    f.write(response.content)
                print(f"Thermal image saved to: {save_path}")
                return True

            print(f"Thermal capture failed: HTTP {response.status_code}, "
                  f"Content-Type={content_type!r}, body={response.text[:200]!r}")
            return False

        except requests.exceptions.RequestException as e:
            print(f"Error capturing thermal image: {e}")
            return False

    # --- Pilot / Safety authority ---

    def requestReleaseSafetyControl(self):
        """Return command authority to the Pilot Computer.

        After this call the Pilot Computer's commands are accepted again and this
        interface goes back to being a supervisor.
        """
        return self.requestSend(EP_RELEASE_SAFETY_CONTROL, "")


if __name__ == '__main__':
    import sys
    import time

    def _looks_rejected(response_text):
        """Heuristic checker for rejection-like responses from the Android bridge."""
        if response_text is None:
            return True
        text = str(response_text).strip().lower()
        if text == "":
            return True
        rejection_markers = (
            "reject", "rejected", "denied", "forbidden", "unauthorized",
            "not allowed", "not authorised", "401", "403", "safety"
        )
        return any(marker in text for marker in rejection_markers)

    IP_RC = "10.177.40.181"  # REPLACE WITH YOUR RC IP
    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]

    print(f"[SAFETY] Connecting to {IP_RC} with token {SAFETY_TOKEN!r}...")
    safety = DJIInterfaceSafety(IP_RC)
    pilot = DJIInterface(IP_RC)

    # The first command seizes control from the Pilot Computer (persistent).
    print("[SAFETY] Seizing control (RTH)...")
    print("  ->", safety.requestSendRTH())

    time.sleep(5)

    # Proof: once Safety has seized control, a Pilot command (no token) should be rejected.
    print("[PILOT] Sending command without token while Safety has control (RTH)...")
    pilot_reply = pilot.requestSendRTH()
    print("  ->", pilot_reply)
    print("[PROOF] Pilot command rejected:", _looks_rejected(pilot_reply))

    # Hand control back to the Pilot Computer.
    print("[SAFETY] Releasing control back to Pilot...")
    print("  ->", safety.requestReleaseSafetyControl())
