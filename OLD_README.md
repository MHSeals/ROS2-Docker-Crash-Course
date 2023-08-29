# Roboboat Dockerized Environment

## Development Status

| Operating System | Status |
|------------------|:------:|
| Windows          |   ✅   |
| MacOS/Apple      |   ❌   |
| Ubuntu           |   ✅   |
| Fedora*          |   ✅   |
| Debian           |   ❓   |
| Arch             |   ❓   |

\* See #1 by Alec. You will need to modify the run script to mount the entire `/dev/` directory onto the container for it to work.

🐳 Docker Hub Image Push: **COMPLETED!**

## Quick Start

1. Setup Docker. See [comprehensive Docker guide](./DOCKER.md).
2. Clone this repository.

```
git clone git@github.com:MHSeals/ROS2-Docker-Crash-Course.git
```
3. Generate the appropriate build and run files

```shell
./generate_config.py --os [your_os] --gpu [your_gpu]
```
OS options are currently `linux` or `windows`. GPU options are `intel` or `nvidia`.

This command should spawn `build.sh`, `pull.sh`, `test.sh`, and `run.sh` in the repository directory.

4. Pull the Docker image from Docker Hub.

```shell
bash pull.sh
```

Wait for the pull to complete. Wait time may varies based on your internet speed.

5. (Optional) Verify the container will be using your specified GPU when running. For examle, `bash test.sh` should return NVIDIA GPU as your OpenGL renderer if you selected `nvidia` during step 3.

```
$ bash test.sh
access control disabled, clients can connect from any host
OpenGL vendor string: NVIDIA Corporation
OpenGL renderer string: Quadro P4000/PCIe/SSE2
OpenGL core profile version string: 4.6.0 NVIDIA 520.61.05
OpenGL core profile shading language version string: 4.60 NVIDIA
OpenGL core profile context flags: (none)
OpenGL core profile profile mask: core profile
OpenGL core profile extensions:
OpenGL version string: 4.6.0 NVIDIA 520.61.05
OpenGL shading language version string: 4.60 NVIDIA
OpenGL context flags: (none)
OpenGL profile mask: (none)
OpenGL extensions:
OpenGL ES profile version string: OpenGL ES 3.2 NVIDIA 520.61.05
OpenGL ES profile shading language version string: OpenGL ES GLSL ES 3.20
OpenGL ES profile extensions:
access control enabled, only authorized clients can connect
$
```

6. Launch the container using `bash run.sh`. You will need to run this twice, the first time will create a container named `roboboat-container` which will persist after reboot, the second run will do two things:

   1. If `roboboat-container` is not started, start it, then attach to it.
   2. If `roboboat-container` is already running, just attach to it.

At this point, you should be able to open as many terminal as needed, and simply run `bash run.sh` to connect your terminal so it can run commands inside containers.

7. (Optional) Verify you can spawn a GUI window.

```
root@[container_id]$ glxgears
```

You should now see a small windows with 3 colored gears spinning. Congratulations! 🎊 You are now ready to do some simulations.

8. (Optional) Use Visual Studio Code

VSCode + Remote Development extension will allow you to have a full-featured editor capable of accessing directory and code files within the container. This is highly recommended, but not entirely necessary.

## Boat Simulation

Once inside your container, you may work as if you are on Ubuntu. By default, you should be in `~/roboboat_ws` when you attach to the container.

>:warning: Within the container you will be the user `root`, which means you do not need to use `sudo`. Do **be very careful** running around with the root user...

1. Clean up `turtlebot3_simulation`, which will conflict with our boat sim.

```
cd $BOAT_WS && rm -rf build/ log/ install/ src/turtlebot3_simulations
```

2. Clone the VRX Repository

```
git clone https://github.com/osrf/vrx.git $BOAT_SRC/vrx
```

3. Install recommended Gazebo and tools

```
apt update && apt -y install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro gz-garden
```
4. Install additional packages dependencies using `rosdep`

```
cd $BOAT_WS && rosdep install --from-paths src -y --ignore-src
```

5. Build the VRX package.

```
colcon build --merge-install
```

6. Source the setup files

```
source $BOAT_WS/install/setup.bash
```

7. Run the simulation

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

First startup will take some times, but you should eventually see a boat (WAMV) floating on the water with some obstacles.

Continue through the tutorial [at VRX Official Guide](https://github.com/osrf/vrx/wiki/getting_around_tutorial).
