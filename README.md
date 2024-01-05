# Ardupilot ROS 2 Software-in-the-Loop (SITL)

This repository includes Dockerfile and run script for testing MAVROS code.

## Quick Start

Starting the container will mount the directory `mhsboat` into the container at `/root/ros_ws/src/mhsboat`. All your
code should be placed within that `mhsboat` directory.

Use `run.sh` to automatically create, start, or attaching to the container.

### Terminal 1: Start SITL with Gazebo

```
ros2 launch ardupilot_gz_bringup wildthumper_playpen.launch.py
```

### Terminal 2: Start MAVROS

```
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14555
```

### Terminal 3: Build your own ROS 2 package

```
cd ${ROS_WS} && colcon build --symlink-install --packages-select mhsboat
```

### Terminal 3: Arming and Controlling

You can arm the rover with the service `/mavros/cmd/arming` and set mode to `GUIDED` through the service
`/mavros/set_mode`. This will allow you to manually drive the robot via `/mavros/setpoint_velocity/cmd_vel` by
specifying linear (XYZ) and rotational (RPY) velocity vectors as a tuple of 3 values.
