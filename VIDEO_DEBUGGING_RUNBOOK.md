# WildBridge Video Debugging Runbook

Updated: 2026-05-19

This document captures the working setup, the validated deployment flow, the current WebRTC tuning, and the main failure modes observed during multi-phone video testing.

## Core Findings

- The main recurring failure mode is not a full app crash.
- The common bad state is: phone app stays alive, encoder keeps producing frames, MediaMTX path stays ready, browser stays connected, but effective browser FPS collapses to `0-1`.
- In those bad windows, browser jitter rises sharply while `connectionState=connected` remains true.
- That pattern points to transport/uplink pressure and queueing, not a dead sender process.
- Recovery logic based only on local frame-source health is not sufficient when the sender is alive but outbound continuity has degraded.

## Validated Failure Patterns

### 1. Connected but unusable stream

Observed repeatedly on `mini2` and `mini5`.

Symptoms:

- Browser stats show `fps=0` or `fps=1`.
- `connectionState=connected` and `iceConnectionState=connected`.
- `packetsLost` may remain low or zero.
- Jitter rises significantly.
- Phone logs still show `HardwareVideoEncoder: Sync frame generated`.
- MediaMTX still shows the path as `ready`, `online`, with a reader attached.

Interpretation:

- The stream is alive at the session level.
- Delivery quality is bad enough that the browser cannot render smooth video.
- This is likely caused by sender bitrate pressure and/or unstable shared Wi-Fi uplink behavior.

### 2. No DJI frames available before WHIP offer

Observed on `mini5` after redeploy.

Symptoms:

- `WhipPublisher: No DJI video frames before WHIP offer; recovering capture`
- `Publish failed: No DJI video frames available for WHIP publishing`
- `Camera feed lost. The drone may have been idle too long or overheated`

Interpretation:

- This is a separate failure mode from the transport/jitter issue.
- The sender never receives usable DJI frames, so WHIP cannot complete useful publishing.
- In this state, bitrate tuning cannot be validated because the stream never reaches steady publishing.

## Current WebRTC / Sender Settings

The current fix applied in code is a conservative sender bitrate cap.

### Current intent

- Avoid aggressive static bitrates in multi-phone Wi-Fi conditions.
- Especially avoid native/source mode publishing at `6 Mbps`.
- Prefer lower, safer sender caps that reduce queue growth and downstream collapse.

### Current validated change

- Native/default mode sender cap reduced from `6_000_000` to `2_500_000` bps.
- Shared sender bitrate helper now drives both WHIP and direct WebRTC sender tuning.

### Current cap policy

- Native/source mode: `2_500_000`
- 1080p-class mode: `4_000_000`
- 720p-class mode: `2_000_000`
- Lower resolutions: `1_500_000`

### Runtime validation

Validated on `mini2` after rebuild and reinstall:

- `WhipPublisher: Sender params tuned: maxBitrate=2500000bps, maxFps=5, prefer=MAINTAIN_FRAMERATE`

This confirms the updated build reached the phone and replaced the old `6000000bps` sender path.

## Build / Validate / Package

The active Android build is driven from:

- `WildBridgeApp/android-sdk-v5-as`

The actual Gradle modules included by `settings.gradle` are:

- `:sample`
- `:uxsdk`

Do not use `:android-sdk-v5-sample`; that project path does not exist.

### Kotlin compile validation

```bash
cd /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-as
./gradlew :sample:compileDebugKotlin
```

### Build debug APK

```bash
cd /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-as
./gradlew :sample:assembleDebug
```

### Expected APK path

```bash
/home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
```

## Wireless Debugging / Phone Endpoints

Validated connected phones:

- `mini1` -> `192.168.50.172:5555`
- `mini2` -> `192.168.50.183:5555`
- `mini3` -> `192.168.50.224:5555`
- `mini4` -> `192.168.50.200:5555`
- `mini5` -> `192.168.50.92:5555`
- `mini6` -> `192.168.50.219:5555`
- `mini7` -> `192.168.50.42:5555`
- `mini8` -> `192.168.50.83:5555`

USB mapping used to enable wireless debugging:

- `mini7` -> USB serial `335c7c32`
- `mini8` -> USB serial `54af3c3a`

### Enable wireless ADB for the USB-only phones

```bash
adb -s 335c7c32 tcpip 5555
adb connect 192.168.50.42:5555

adb -s 54af3c3a tcpip 5555
adb connect 192.168.50.83:5555
```

### List current wireless-debugging devices

```bash
adb devices -l
```

## Install on Multiple Phones

### Single phone install

```bash
adb -s 192.168.50.183:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
```

### All six phones

```bash
adb -s 192.168.50.172:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
adb -s 192.168.50.183:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
adb -s 192.168.50.224:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
adb -s 192.168.50.200:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
adb -s 192.168.50.92:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
adb -s 192.168.50.219:5555 install -r /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
```

Note:

- Reinstall commonly drops the phones back to the launcher.
- The app usually needs to be relaunched manually or through ADB after install.

## How to Open the App and Reach WildBridge Default Layout

### App identity

- Application ID: `com.dji.sampleV5.aircraft`
- Launcher activity: `dji.sampleV5.aircraft.DJIAircraftMainActivity`
- WildBridge screen: `dji.sampleV5.aircraft.WildBridgeDefaultLayoutActivity`

### Important constraint

`WildBridgeDefaultLayoutActivity` is not exported.

That means this does **not** work from shell:

```bash
adb -s 192.168.50.183:5555 shell am start -n com.dji.sampleV5.aircraft/dji.sampleV5.aircraft.WildBridgeDefaultLayoutActivity
```

Android blocks it with a permission denial because the activity is not exported.

### Correct way to open the app

```bash
adb -s 192.168.50.183:5555 shell am start -n com.dji.sampleV5.aircraft/dji.sampleV5.aircraft.DJIAircraftMainActivity
```

This was validated on-device.

### How to enter the WildBridge default layout

The main activity wires its `DEFAULT LAYOUT` button to `WildBridgeDefaultLayoutActivity`.

Important behavior:

- The button may not be immediately active right after the main activity appears.
- If tapped too early, the activity may remain on the main screen.
- Once the activity finishes setup, the button path works.

### Validated fallback tap sequence on the current phones

After launching the main activity, this tap was validated to enter the WildBridge default layout on these devices:

```bash
adb -s 192.168.50.183:5555 shell input tap 375 700
```

This coordinate assumes the current validated landscape layout on the tested phones.

### Relaunch flow for one phone

```bash
adb -s 192.168.50.183:5555 shell am start -n com.dji.sampleV5.aircraft/dji.sampleV5.aircraft.DJIAircraftMainActivity
adb -s 192.168.50.183:5555 shell input tap 375 700
adb -s 192.168.50.183:5555 shell dumpsys activity activities | grep -i 'topResumedActivity' | tail -n 1
```

Expected final activity:

- `com.dji.sampleV5.aircraft/dji.sampleV5.aircraft.WildBridgeDefaultLayoutActivity`

## Video Test Stack

Current video test stack:

- Compose file: `compose.video-test.yaml`
- MediaMTX API port: `9997`
- WHIP/WHEP media service: `8889`
- Browser test UI: `http://localhost:8090`
- Browser logs written to: `GroundStation/video_test/logs/`

### Start the stack

```bash
docker compose -f compose.video-test.yaml up -d --build
```

## Diagnosis Commands

### Check MediaMTX path state

```bash
curl -s http://127.0.0.1:9997/v3/paths/list | jq '.items[] | select(.name=="mini2")'
```

Healthy enough to publish does not necessarily mean healthy enough to view smoothly.

Typical misleading good-looking state:

- `ready: true`
- `online: true`
- reader attached
- bytes increasing

while browser FPS is still near zero.

### Check browser-side stats log freshness

```bash
wc -l /home/alejp/dev/WildBridge/GroundStation/video_test/logs/video-connection-test-20260519T173619Z.ndjson
stat -c '%y %s' /home/alejp/dev/WildBridge/GroundStation/video_test/logs/video-connection-test-20260519T173619Z.ndjson
```

### Inspect latest browser stats

```bash
tail -n 40 /home/alejp/dev/WildBridge/GroundStation/video_test/logs/video-connection-test-20260519T173619Z.ndjson
```

### What current bad browser stats look like

Example `mini2` degradation pattern observed:

- `connectionState: connected`
- `iceConnectionState: connected`
- `fps: 1` then `0`
- `jitter: ~0.64-0.67`

This means the stream is logically connected but not delivering usable continuity.

### Inspect phone sender logs

```bash
adb -s 192.168.50.183:5555 logcat -d | grep -E 'WhipPublisher|WebRTCStreamer|Sender params tuned|maxBitrate'
```

### Live log filtering

Useful indicators:

- `HardwareVideoEncoder: Sync frame generated`
- `WhipPublisher: Sender params tuned:`
- `WHIP publishing from`
- `No DJI video frames available for WHIP publishing`
- `Camera feed lost`

## Practical Interpretation of the Current State

### What has been proven

- The new sender bitrate cap build compiles and installs successfully.
- The new `2500000bps` cap is active on at least one live phone (`mini2`).
- Some failures are not sender crashes; they are continuity failures under live load.
- `mini5` has also shown a distinct capture-source failure where no DJI frames arrive before WHIP publishing.

### What is still not solved

- Lowering the sender bitrate alone did not fully eliminate the `connected but ~0 fps` failure mode.
- `mini2` still showed browser-side degradation even after the new build was active.

### Likely next engineering direction

- Add transport-health-based recovery in addition to source-health recovery.
- Avoid relying only on local frame production metrics.
- Potentially step resolution/bitrate down more aggressively under sustained jitter.

## Short Session Notes

- `mini2` was the most consistent example of the transport-degradation failure mode.
- `mini5` was the clearest example of the separate `no DJI frames before WHIP offer` capture failure.
- The validated in-field workflow is now: build, install, relaunch main activity, tap into default layout, validate `Sender params tuned`, then compare phone logs, MediaMTX status, and browser stats together.
