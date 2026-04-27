#!/bin/bash

# Script to capture crash logs when the app starts
# Usage: ./capture_crash_log.sh [device_id]

echo "========================================="
echo "WildBridge App Crash Log Capture Tool"
echo "========================================="
echo ""

# Check if device is connected
echo "Checking for connected devices..."
adb devices -l

DEVICE_COUNT=$(adb devices | grep -v "List" | grep "device" | wc -l)

if [ "$DEVICE_COUNT" -eq 0 ]; then
    echo ""
    echo "ERROR: No Android device found!"
    echo ""
    echo "Please ensure:"
    echo "1. Your phone is connected via USB"
    echo "2. USB debugging is enabled on your phone"
    echo "3. You've authorized this computer on your phone"
    echo ""
    echo "To enable USB debugging:"
    echo "- Go to Settings > About Phone"
    echo "- Tap 'Build Number' 7 times to enable Developer Options"
    echo "- Go to Settings > Developer Options"
    echo "- Enable 'USB Debugging'"
    echo ""
    exit 1
fi

# Handle multiple devices
DEVICE_ID="$1"
if [ "$DEVICE_COUNT" -gt 1 ]; then
    if [ -z "$DEVICE_ID" ]; then
        echo ""
        echo "Multiple devices detected. Please specify which device to use:"
        echo ""
        adb devices -l
        echo ""
        echo "Usage: $0 <device_id>"
        echo "Example: $0 emulator-5554"
        echo ""
        exit 1
    fi
    ADB_CMD="adb -s $DEVICE_ID"
    echo ""
    echo "Using device: $DEVICE_ID"
else
    ADB_CMD="adb"
    echo ""
    echo "Device connected successfully!"
fi
echo ""

# Create logs directory
LOGS_DIR="$(dirname "$0")/crash_logs"
mkdir -p "$LOGS_DIR"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$LOGS_DIR/crash_log_$TIMESTAMP.txt"

echo "Clearing previous logs..."
$ADB_CMD logcat -c

echo ""
echo "Installing and launching app..."
echo "Please wait while we install the APK..."

# Install the APK
APK_PATH="$(dirname "$0")/../android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk"

if [ ! -f "$APK_PATH" ]; then
    echo "ERROR: APK not found at $APK_PATH"
    echo "Please build the app first: ./gradlew :sample:assembleDebug"
    exit 1
fi

$ADB_CMD install -r "$APK_PATH"

echo ""
echo "Starting log capture..."
echo "Log file: $LOG_FILE"
echo ""
echo "==================== INSTRUCTIONS ===================="
echo "1. The app will now launch automatically"
echo "2. Logs are being captured in real-time"
echo "3. If the app crashes, the crash will be logged"
echo "4. Press Ctrl+C when you're done capturing"
echo "======================================================"
echo ""

# Launch the app and capture logs
$ADB_CMD shell am start -n dji.sampleV5.aircraft/.WildBridgeDefaultLayoutActivity

# Capture logs with filters for crash information
echo "Capturing logs... (Press Ctrl+C to stop)"
echo ""

$ADB_CMD logcat -v time \
    AndroidRuntime:E \
    System.err:E \
    *:E \
    DJIAircraftApplication:D \
    DroneController:D \
    WildBridge:D \
    *:W | tee "$LOG_FILE"

echo ""
echo "Log saved to: $LOG_FILE"
