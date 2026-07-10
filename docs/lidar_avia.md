# Livox Avia — ROS2 driver setup (Ubuntu 22.04 / Humble)

The Avia uses Livox's **original SDK (Livox-SDK v1)** and the **`livox_ros2_driver`**
package. (The newer Mid-360 / HAP use Livox-SDK2 / `livox_ros_driver2` — a
different SDK; do not use that one for the Avia.)

The driver publishes a standard `sensor_msgs/PointCloud2` on `/livox/lidar`,
which is exactly what `livox_mapping` subscribes to — so no mapping-side changes
are needed.

## 1. Physical connection

- Power the Avia from its own supply (9–27 V).
- Connect the Avia to the mini PC by **Ethernet** (directly, or through a switch).

## 2. Host network config

Livox LiDARs use static IPs. The Avia's IP is derived from its broadcast code;
by default the host must be on the `192.168.1.x` subnet. Set the mini PC's wired
connection to a static IP:

```bash
# find your ethernet interface name (e.g. eth0, enp1s0)
ip link
# set a static IP on the Livox subnet (replace enp1s0 with yours)
sudo ip addr add 192.168.1.50/24 dev enp1s0
sudo ip link set enp1s0 up
```

(For a permanent setting, configure the same static IP in the Ubuntu network
settings GUI.) Verify you can reach the Avia once it is powered:

```bash
ping 192.168.1.1XX      # the Avia's IP; if unknown, the driver still finds it by broadcast code
```

## 3. Install the driver

The one-shot `scripts/setup_minipc.sh` does this automatically. To do it by hand:

```bash
# Livox-SDK (v1)
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK.git
# GCC 11 (Ubuntu 22.04) fix: the 2019 SDK uses std::shared_ptr without including
# <memory>. Without this one line the build fails with
# "'shared_ptr' in namespace 'std' does not name a type". Add the missing include:
sed -i '/#include "noncopyable.h"/a #include <memory>' \
    ~/Livox-SDK/sdk_core/src/base/thread_base.h
mkdir -p ~/Livox-SDK/build && cd ~/Livox-SDK/build
cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig

# livox_ros2_driver, into this workspace
cd ~/livox_high_precision_mapping_ros2/ws/src
git clone https://github.com/Livox-SDK/livox_ros2_driver.git
cd ~/livox_high_precision_mapping_ros2/ws
colcon build --symlink-install
source install/setup.bash
```

## 4. Set your Avia's broadcast code

Every Avia has a ~14–15 character **broadcast code** printed on the device (near
the QR/SN label). Put it in the driver's config so it connects to your unit:

```
~/livox_high_precision_mapping_ros2/ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json
```

```json
{
  "lidar_config": [
    {
      "broadcast_code": "REPLACE_WITH_YOUR_AVIA_CODE",
      "enable_connect": true,
      "enable_fan": true,
      "return_mode": 0,
      "coordinate": 0,
      "imu_rate": 0,
      "extrinsic_parameter_source": 0
    }
  ],
  "timesync_config": {
    "enable_timesync": false,
    "device_name": "/dev/ttyUSB0",
    "comm_device_type": 0,
    "baudrate_index": 2,
    "parity_index": 0
  }
}
```

`imu_rate: 0` disables the Avia's own IMU (we use the IM10A instead). Leave
`coordinate: 0` (cartesian) and `return_mode: 0` (single-return, matches the
Avia's "Single First"). `enable_timesync` stays `false` because the Avia is
already fed 1PPS in hardware (its Viewer shows "PPS Sync"). Rebuild is not needed
for a config change when built with `--symlink-install`.

## 5. Run and verify the point cloud

```bash
source ~/livox_high_precision_mapping_ros2/ws/install/setup.bash
ros2 launch livox_ros2_driver livox_lidar_launch.py
```

In another terminal, confirm points are arriving:

```bash
ros2 topic list                 # expect /livox/lidar
ros2 topic hz /livox/lidar      # ~10 Hz
ros2 topic echo /livox/lidar --field width   # non-zero point count
```

The provided `livox_lidar_launch.py` publishes `PointCloud2` (`xfer_format: 0`)
on `/livox/lidar` at 10 Hz — which is what the mapping node expects. To see the
raw cloud in RViz, use `livox_lidar_rviz_launch.py` instead.

## 6. Feed it into the map

Once `/livox/lidar` is publishing, the full pipeline can run. **Match the
mapping `lidar_delta_time` to the LiDAR publish rate** for correct per-point
motion compensation: at the default 10 Hz, use `0.1`.

```bash
ros2 launch livox_hp_mapping_bringup mapping_online.launch.py \
    lidar_delta_time:=0.1 use_gnss_heading:=true
```

(The bringup already starts the UM982 and IM10A drivers on `/dev/gps` and
`/dev/imu`. Start the Avia driver separately as in step 5, or add it to the
bringup once confirmed working.)

## Notes / troubleshooting

- **No `/livox/lidar`**: check the broadcast code, that the Avia is powered, and
  that the host IP is on the Avia's subnet (`192.168.1.50/24`). `sudo ufw disable`
  temporarily if a firewall blocks the UDP data.
- **Topic name differs**: if the driver publishes on another topic, remap it, e.g.
  `ros2 launch ... --remap /livox/lidar:=<actual_topic>`.
- **Frame rate**: if you change `publish_freq` in the launch, set
  `lidar_delta_time` to `1 / publish_freq`.
