#!/bin/bash

UXRCE_PORT="/dev/ttyUSB0"
VN110_PORT="/dev/ttyUSB1"
RECORD_BAG="false"  # Set to "false" to disable rosbag recording

cleanup() {
    echo "Stopping all processes..."
    kill $UXRCE_PID $VN110_PID $FOXGLOVE_PID 2>/dev/null
    wait
    exit 0
}

trap cleanup SIGINT SIGTERM

ros2 daemon stop
ros2 daemon start

sleep 2

source ~/ros2_ws/install/setup.bash
MicroXRCEAgent serial --dev $UXRCE_PORT -b 1000000 &

UXRCE_PID=$!

sleep 10


ros2 launch vn110_bridge launch_with_recording.py port:=$VN110_PORT record:=$RECORD_BAG &
VN110_PID=$!

ros2 run foxglove_bridge foxglove_bridge &
FOXGLOVE_PID=$!

echo "All processes started:"
echo "  MicroXRCEAgent PID: $UXRCE_PID ($UXRCE_PORT)"
echo "  VN110 Bridge + DR PID: $VN110_PID ($VN110_PORT)"
echo "  Foxglove Bridge PID: $FOXGLOVE_PID"
echo "  Rosbag Recording: $RECORD_BAG"

wait
