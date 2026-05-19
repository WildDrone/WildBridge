# ADB Wireless Setup — WildDrone_Ca_Biche Router

## Phone addresses (192.168.50.x subnet)

| # | IP | ADB endpoint |
|---|-----|--------------|
| 1 | 192.168.50.172 | `192.168.50.172:5555` |
| 2 | 192.168.50.183 | `192.168.50.183:5555` |
| 3 | 192.168.50.224 | `192.168.50.224:5555` |
| 4 | 192.168.50.200 | `192.168.50.200:5555` |
| 5 | 192.168.50.92  | `192.168.50.92:5555`  |
| 6 | 192.168.50.219 | `192.168.50.219:5555` |

> All phones are OPPO CPH2621. IPs may change on DHCP lease renewal — set static DHCP reservations on the router if needed.

---

## Reconnect all phones (no cable needed after first setup)

```bash
adb connect 192.168.50.172:5555
adb connect 192.168.50.183:5555
adb connect 192.168.50.224:5555
adb connect 192.168.50.200:5555
adb connect 192.168.50.92:5555
adb connect 192.168.50.219:5555
adb devices -l
```

---

## Install APK to all 6 phones simultaneously

```bash
APK=WildBridgeApp/android-sdk-v5-sample/build/outputs/apk/debug/app-debug.apk
for d in 192.168.50.172:5555 192.168.50.183:5555 192.168.50.224:5555 \
          192.168.50.200:5555 192.168.50.92:5555 192.168.50.219:5555; do
    adb -s $d install -r $APK &
done
wait && echo "Done — installed on all phones"
```

---

## First-time setup for a new phone (requires USB cable once)

1. Enable **Developer options** on the phone (tap Build number 7 times)
2. Enable **USB debugging** in Developer options
3. Plug in via USB and accept the "Allow USB debugging" prompt on screen
4. Run:
```bash
# Get the new phone's USB serial
adb devices

# Switch to TCP mode and get its Wi-Fi IP
adb -s <SERIAL> tcpip 5555
sleep 2
adb -s <SERIAL> shell ip addr show wlan0 | grep 'inet '
```
5. Unplug the cable
6. Run:
```bash
adb connect <PHONE_IP>:5555
```
7. Add the new IP to the reconnect script above

> **Note:** Make sure the phone is connected to **WildDrone_Ca_Biche** Wi-Fi (192.168.50.x), not any other network.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `No route to host` | Check phone is on WildDrone_Ca_Biche Wi-Fi |
| `adb mdns services` returns nothing | Router AP isolation is on, or Wireless debugging not enabled — use `adb tcpip 5555` method above |
| Phone IP changed after reboot | Reconnect via USB, re-run `tcpip 5555` + `shell ip addr`, or set DHCP reservation on router |
| `unauthorized` on USB connect | Accept the "Allow USB debugging" dialog on the phone screen |
