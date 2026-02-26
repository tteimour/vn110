# VN-110 AHRS — ROS2 Bridge

ROS2 node for the VectorNav VN-110E AHRS sensor. Publishes IMU, attitude, and sensor status data at up to 400 Hz. Accepts PX4 airspeed for velocity aiding and supports real-time parameter tuning via Foxglove.

---

## Hardware Setup

### Components

- **VectorNav VN-110E** — AHRS sensor (serial, 921600 baud)
- **PX4 Flight Controller** — provides airspeed, attitude, and GPS data
- **NVIDIA Jetson** (or any Linux SBC) — runs ROS2 and this node

### Wiring

**VN-110E to Jetson** — USB-to-serial (FTDI or CP2102). Connect VN-110 TX/RX/GND to the TTL adapter, plug USB into the Jetson. Default port: `/dev/ttyUSB1`.

**PX4 to Jetson (uXRCE-DDS)** — Connect PX4 TELEM2 (UART) to a second USB-to-TTL adapter on the Jetson. This runs the Micro XRCE-DDS Agent for ROS2-PX4 communication. Default port: `/dev/ttyUSB0`, baud: `1000000`.

> Both connections use 3.3V TTL level. Make sure the TTL adapters match the voltage level of the devices. Do **not** use RS-232 level converters.

### Port Identification

After connecting both USB-to-serial adapters:

```bash
dmesg | grep ttyUSB
```

Identify which port maps to which device. Update the ports in `start_all.sh` or pass them as launch arguments.

### Serial Permissions

```bash
sudo usermod -aG dialout $USER
# Log out and back in, then verify:
ls -l /dev/ttyUSB*
```

---

## Prerequisites

- **ROS2 Humble** (desktop or base install)
- **Micro XRCE-DDS Agent** — for PX4-ROS2 bridge
- **Foxglove Bridge** (optional) — for web-based parameter tuning and visualization

Install the XRCE agent:

```bash
sudo apt install ros-humble-micro-xrce-dds-agent
```

---

## Build

```bash
cd vn110/vn110_node
source /opt/ros/humble/setup.bash
colcon build
```

The VectorNav SDK is compiled from `vnsdk/` automatically — no separate install needed. PX4 messages are pulled as a git submodule:

```bash
git submodule update --init --recursive
```

---

## Run

### Quick Start (All-in-One)

The `start_all.sh` script launches everything — XRCE agent, VN-110 bridge, and Foxglove bridge:

```bash
# Edit ports at the top of the script first
./start_all.sh
```

### Manual Launch

```bash
source /opt/ros/humble/setup.bash
source vn110_node/install/setup.bash

# Start XRCE agent for PX4
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 1000000 &

# Start VN-110 bridge
ros2 launch vn110_bridge launch_with_recording.py port:=/dev/ttyUSB1

# Optional: Foxglove bridge for web UI
ros2 run foxglove_bridge foxglove_bridge
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | VN-110 serial port |
| `record` | `true` | Enable rosbag recording (MCAP) |
| `bag_dir` | `~/rosbags` | Rosbag output directory |

---

## Configuration

All parameters live in `vn110_node/src/vn110_bridge/config/vn110_params.yaml` and can be changed at runtime via Foxglove or `ros2 param set`.

### Connection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `921600` | Serial baud rate |
| `ahrs_rate` | `200` | Output rate in Hz (1–400) |

### Attitude Estimation (VPE)

| Parameter | Values | Description |
|-----------|--------|-------------|
| `vpe_heading_mode` | 0=Absolute, 1=Relative, 2=Indoor | Heading source |
| `vpe_filtering_mode` | 0=Unfiltered, 1=AdaptivelyFiltered | Output filtering |
| `vpe_tuning_mode` | 0=Static, 1=Adaptive | Filter tuning |

### Velocity Aiding (PX4 Airspeed)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `velocity_aiding_enabled` | `true` | Feed PX4 airspeed into VN-110 |
| `velocity_uncertainty` | `0.1` | Measurement uncertainty (m/s) |
| `velocity_aiding_threshold` | `5.0` | Min airspeed to start aiding (m/s) |

### Hard/Soft Iron Calibration (HSI)

| Parameter | Values | Description |
|-----------|--------|-------------|
| `hsi_mode` | 0=Off, 1=Run, 2=Reset | Real-time magnetometer calibration |
| `hsi_apply_compensation` | true/false | Apply calibration results |
| `hsi_converge_rate` | 1–5 | Convergence speed (1=slow, 5=fast) |

### World Magnetic & Gravity Model

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_model_enabled` | `true` | Use reference magnetic/gravity model |
| `world_model_latitude` | `40.399` | Reference latitude (degrees) |
| `world_model_longitude` | `49.811` | Reference longitude (degrees) |
| `world_model_altitude` | `0.0` | Reference altitude (meters) |

### Reference Frame Rotation

9-element DCM matrix (`ref_frame_rot_00` through `ref_frame_rot_22`) for sensor-to-vehicle frame alignment. Set in the YAML based on how the VN-110 is physically mounted.

---

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/vn110/imu` | `sensor_msgs/Imu` | Quaternion, gyro, accelerometer |
| `/vn110/euler` | `geometry_msgs/Vector3Stamped` | Roll, pitch, yaw (degrees) |
| `/vn110/aided_velocity` | `geometry_msgs/Vector3Stamped` | Airspeed received from PX4 |
| `/vn110/aided_velocity_sent` | `geometry_msgs/Vector3Stamped` | Velocity actually sent to VN-110 |
| `/vn110/status/gyro` | `std_msgs/UInt8` | Gyro saturation (0–3) |
| `/vn110/status/accel` | `std_msgs/UInt8` | Accelerometer saturation (0–3) |
| `/vn110/status/mag` | `std_msgs/UInt8` | Magnetometer saturation (0–3) |
| `/vn110/status/pressure` | `std_msgs/UInt8` | Pressure sensor saturation (0–3) |
| `/vn110/status/any_saturation` | `std_msgs/Bool` | True if any sensor is saturated |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/out/airspeed_validated` | `px4_msgs/AirspeedValidated` | PX4 airspeed for velocity aiding |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | PX4 attitude (for comparison) |
| `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | PX4 raw IMU data |

---

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/vn110/write_settings` | `std_srvs/Trigger` | Save current settings to flash |
| `/vn110/reset_sensor` | `std_srvs/Trigger` | Reset the sensor |
| `/vn110/restore_factory` | `std_srvs/Trigger` | Restore factory defaults |
| `/vn110/known_mag_disturbance` | `std_srvs/SetBool` | Flag magnetic disturbance |
| `/vn110/known_accel_disturbance` | `std_srvs/SetBool` | Flag acceleration disturbance |
| `/vn110/read_hsi_results` | `std_srvs/Trigger` | Read HSI calibration state |

> Parameter changes are **not** saved to flash automatically. Call `/vn110/write_settings` to persist settings across power cycles.

---

## Troubleshooting

**Sensor not detected** — Check `dmesg | grep ttyUSB`, verify the port path, and ensure your user is in the `dialout` group.

**No PX4 data on `/fmu/out/*` topics** — Make sure the XRCE agent is running and connected to the correct serial port. Check PX4 `UXRCE_DDS_CFG` parameter is set to the right TELEM port.

**Parameter changes not taking effect** — Verify the node is running (`ros2 node list`). Check logs with `ros2 topic echo /rosout`.

**Settings lost after power cycle** — Call `ros2 service call /vn110/write_settings std_srvs/srv/Trigger` to save to flash before powering off.
