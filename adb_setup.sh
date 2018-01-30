#!/usr/bin/env bash

##
# Setup wireless ADB
#
# Instructions:
# 1. Connect to the phone's WiFi Direct connection
#     * The password can be found in the phone's WiFi settings.
#     * If you limited the connection to two devices when you created it, you will need to
#       reconfigure it to allow three.
# 2. Run this script
# 3. If this script fails, it's safe to run again.
## 

echo "Starting ADB Deamon on the Phone..."
adb tcpip 5555
echo "Disconnecting USB Connection (Feel free to unplug the phone now)..."
adb disconnect # This disconnects everything
echo "Waiting a bit, because that seems to make it work better..."
sleep 10
echo "Now, reconnecting over WiFi..."
adb connect 192.168.49.1
echo "Done! Wireless ADB is now set up! Use ./run to build and run the app."
