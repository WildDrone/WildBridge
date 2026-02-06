# WildBridge App - Crash Debugging Guide

## Quick Start: Capture Crash Logs

### Method 1: Automated Script (Recommended)

I've created a script that will automatically install the app, launch it, and capture any crash logs.

```bash
cd /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-as
./capture_crash_log.sh
```

**What it does:**
1. Checks if your phone is connected
2. Installs the latest APK
3. Launches the app
4. Captures all crash logs in real-time
5. Saves the log to `crash_logs/crash_log_<timestamp>.txt`

**Requirements:**
- Phone connected via USB
- USB debugging enabled on your phone
- Computer authorized on your phone

---

### Method 2: Manual ADB Commands

If you prefer to capture logs manually:

#### Step 1: Connect your phone
```bash
adb devices
```
You should see your device listed.

#### Step 2: Clear previous logs
```bash
adb logcat -c
```

#### Step 3: Install the app
```bash
cd /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-as
adb install -r ../android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk
```

#### Step 4: Start capturing logs (in one terminal)
```bash
adb logcat -v time AndroidRuntime:E System.err:E *:E DJIAircraftApp:D DJIApplication:D DroneController:D *:W > crash_log.txt
```

#### Step 5: Launch the app (in another terminal)
```bash
adb shell am start -n dji.sampleV5.aircraft/.WildBridgeDefaultLayoutActivity
```

#### Step 6: View the crash log
```bash
cat crash_log.txt
```

---

### Method 3: Using Android Studio Logcat

1. Open Android Studio
2. Connect your phone via USB
3. Go to **View > Tool Windows > Logcat**
4. Install and launch the app
5. Look for red errors in the logcat window
6. Filter by "AndroidRuntime" to see crash stacktraces

---

## Interpreting Crash Logs

### Look for these key indicators:

1. **Fatal Exception:**
```
E AndroidRuntime: FATAL EXCEPTION: main
E AndroidRuntime: Process: dji.sampleV5.aircraft, PID: 12345
E AndroidRuntime: java.lang.RuntimeException: ...
```

2. **StackOverflowError:** (if recursion issue still exists)
```
E AndroidRuntime: java.lang.StackOverflowError
    at dji.sampleV5.aircraft.controller.DroneController.isWaypointReached(DroneController.kt:1276)
```

3. **NullPointerException:**
```
E AndroidRuntime: java.lang.NullPointerException: Attempt to invoke virtual method ... on a null object reference
```

4. **ClassNotFoundException:** (if missing dependencies)
```
E AndroidRuntime: java.lang.ClassNotFoundException: Didn't find class "..."
```

---

## Common Issues and Solutions

### Issue 1: StackOverflowError in DroneController

**Symptom:** App crashes immediately with StackOverflowError
**Solution:** This should be fixed now with the underscore-prefixed private properties

**Verify the fix:**
```bash
grep "_isWaypointReached" /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/controller/DroneController.kt
```

### Issue 2: IllegalAccessError (JDK 17 kapt issue)

**Symptom:** Build fails with module access errors
**Solution:** Already fixed in gradle.properties with --add-opens flags

### Issue 3: NullPointerException during initialization

**Symptom:** Crash in Application.onCreate()
**Solution:** Check the crash log for which object is null

---

## Added Logging

I've added detailed logging to help identify where the crash occurs:

### In DJIApplication.kt:
- Logs when onCreate() starts
- Logs when SDK initialization completes
- Logs any exceptions with full stacktrace

### In DJIAircraftApplication.kt:
- Logs attachBaseContext() lifecycle
- Logs Helper.install() completion
- Logs onCreate() completion

### Look for these log tags:
```bash
adb logcat | grep -E "DJIApplication|DJIAircraftApp|DroneController"
```

---

## Enable USB Debugging on Your Phone

If you haven't enabled USB debugging yet:

1. **Enable Developer Options:**
   - Go to Settings > About Phone
   - Tap "Build Number" 7 times
   - You'll see "You are now a developer!"

2. **Enable USB Debugging:**
   - Go to Settings > Developer Options
   - Turn on "USB Debugging"
   - Connect your phone via USB
   - Accept the authorization prompt on your phone

3. **Verify connection:**
   ```bash
   adb devices
   ```
   Should show: `<device-id>    device`

---

## Quick Diagnostic Commands

### Check if app is installed:
```bash
adb shell pm list packages | grep dji.sampleV5.aircraft
```

### Get app info:
```bash
adb shell dumpsys package dji.sampleV5.aircraft | grep -A 5 "versionName"
```

### Force stop the app:
```bash
adb shell am force-stop dji.sampleV5.aircraft
```

### Uninstall the app:
```bash
adb uninstall dji.sampleV5.aircraft
```

### Clear app data:
```bash
adb shell pm clear dji.sampleV5.aircraft
```

---

## Next Steps

1. **Run the capture script:**
   ```bash
   cd /home/alejp/dev/WildBridge/WildBridgeApp/android-sdk-v5-as
   ./capture_crash_log.sh
   ```

2. **Open the app on your phone** (or let the script launch it)

3. **If it crashes, the log will contain the stacktrace**

4. **Send me the crash log:**
   - The log file will be in `android-sdk-v5-as/crash_logs/`
   - Look for the most recent `crash_log_<timestamp>.txt` file
   - Share the relevant error section

---

## Contact

If you need help interpreting the crash logs, please share:
1. The complete error stacktrace from the log
2. The line numbers and file names mentioned in the error
3. What you were doing when the crash occurred

---

**Note:** The build is successful, so this is a runtime crash, not a compilation issue. The logs will tell us exactly what's happening at runtime.
