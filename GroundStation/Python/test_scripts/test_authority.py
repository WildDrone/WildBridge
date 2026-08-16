"""
WildBridge - Pilot/Safety authority single-computer test.

Runs BOTH a Pilot client (no token) and a Safety client (token "98") from one machine
against the same Android app, and checks the full takeover / release state machine.

The two clients are distinguished only by the X-Safety-Token header, not by IP, so they
can live in the same process on the same computer.

Uses /send/setRTHAltitude: a gated /send/ command that does NOT move the drone, so this
is safe to run on the bench (drone can even be on the ground / disarmed).

Usage:
    python3 test_authority.py <DRONE_IP>
"""

import sys

from djiInterface import DJIInterface
from djiInterfaceSafety import DJIInterfaceSafety


def expect(label, response, should_be_rejected):
    rejected = "REJECTED" in (response or "")
    ok = rejected == should_be_rejected
    verdict = "PASS" if ok else "FAIL"
    expected = "rejected" if should_be_rejected else "accepted"
    print(f"[{verdict}] {label}: expected {expected:8s} | got -> {response!r}")
    return ok


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_authority.py <DRONE_IP>")
        sys.exit(1)
    ip = sys.argv[1]

    pilot = DJIInterface(ip)
    safety = DJIInterfaceSafety(ip)  # token "98" baked in

    print(f"\nTesting Pilot/Safety authority against {ip}\n" + "=" * 60)
    results = []

    # 1) Initial state: Pilot holds control.
    results.append(expect("Pilot command (initial)",
                          pilot.requestSetRTHAltitude(30), should_be_rejected=False))

    # 2) Safety sends a command -> seizes persistent control.
    results.append(expect("Safety command (takeover)",
                          safety.requestSetRTHAltitude(40), should_be_rejected=False))

    # 3) Pilot is now locked out.
    results.append(expect("Pilot command (while Safety holds)",
                          pilot.requestSetRTHAltitude(30), should_be_rejected=True))

    # 4) Pilot CANNOT release (not the Safety Computer).
    pilot_release = pilot.requestSend("/releaseSafetyControl", "")
    results.append(expect("Pilot tries to release",
                          pilot_release, should_be_rejected=True))

    # 5) Pilot still locked out after its failed release attempt.
    results.append(expect("Pilot command (still locked)",
                          pilot.requestSetRTHAltitude(30), should_be_rejected=True))

    # 6) Safety releases control explicitly.
    print(f"[INFO] Safety release -> {safety.requestReleaseSafetyControl()!r}")

    # 7) Pilot regains control.
    results.append(expect("Pilot command (after release)",
                          pilot.requestSetRTHAltitude(30), should_be_rejected=False))

    print("=" * 60)
    print(f"{sum(results)}/{len(results)} checks passed.")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
