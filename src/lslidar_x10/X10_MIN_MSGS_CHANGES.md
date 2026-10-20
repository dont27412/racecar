# X10 minimal lslidar_msgs build

Target launch command:

```bash
ros2 launch lslidar_driver lslidar_x10_launch.py
```

For this launch path, the X10 driver only uses:

- `lslidar_msgs/msg/LslidarPacket.msg`

The following interfaces are not generated in this build:

- `msg/LslidarInformation.msg`
- all `srv/*.srv`

To make that possible, the X10 driver build no longer compiles:

- `src/lslidar_services.cpp`
- `src/lslidar_device_info.cpp`

and the common base header no longer includes or declares the service/device-info members that are unused by `LslidarX10Driver`.

The unused `.msg`/`.srv` source definition files are intentionally left in the source tree for reference, but `rosidl_generate_interfaces()` does not generate code for them.
