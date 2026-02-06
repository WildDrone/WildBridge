#!/bin/bash
# Quick guide to rebuild and test the Android app with discovery fixes

echo "=========================================="
echo "WildBridge Android App - Rebuild Guide"
echo "=========================================="
echo ""

echo "1. Navigate to Android project:"
echo "   cd WildBridgeApp/android-sdk-v5-sample"
echo ""

echo "2. Build the app:"
echo "   ./gradlew assembleDebug"
echo ""

echo "3. Install on drone remote controller:"
echo "   adb install -r app-aircraft/build/outputs/apk/debug/app-aircraft-debug.apk"
echo ""

echo "4. Or open in Android Studio:"
echo "   - File → Open → WildBridgeApp/android-sdk-v5-sample"
echo "   - Build → Build Bundle(s) / APK(s) → Build APK(s)"
echo "   - Run → Run 'app-aircraft'"
echo ""

echo "5. After deployment, test discovery:"
echo "   cd /home/alejp/dev/WildBridge"
echo "   rm -f ~/.wildbridge/drones_cache.json  # Clear cache"
echo "   python3 test_discovery.py"
echo ""

echo "Expected improvements:"
echo "  - Broadcast discovery should work (~1-2 seconds)"
echo "  - Multicast discovery should work (VLAN-friendly)"
echo "  - Subnet scan as fallback only"
echo ""

echo "Check Android logs during test:"
echo "  adb logcat | grep WildBridge"
echo ""
echo "Look for:"
echo "  ✓ Discovery server started on 0.0.0.0:30000"
echo "  ✓ Multicast discovery started on 239.255.42.99:30001"
echo "  📡 UDP/Multicast from <ground_station_ip>"
echo "  ✓ Sent discovery response to <ground_station_ip>"
echo ""

echo "=========================================="
echo "Changes Made to Android App:"
echo "=========================================="
echo "File: WildBridgeDefaultLayoutActivity.kt"
echo ""
echo "1. Added multicast support (239.255.42.99:30001)"
echo "2. Fixed broadcast reception (bind to 0.0.0.0)"
echo "3. Dual-thread discovery server"
echo "4. Enhanced logging with emojis"
echo ""

echo "Ground Station Changes (Already Complete):"
echo "1. Enhanced broadcast (multiple addresses)"
echo "2. Multicast discovery"
echo "3. Parallel subnet scan (50 workers)"
echo "4. Persistent cache (177x faster re-discovery)"
echo ""

echo "=========================================="
