# Roboboat Dockerized Environment

## Development Status

| Operating System | Status |
|------------------|:------:|
| Windows          |   ✅   |
| MacOS/Apple      |   ❌   |
| Ubuntu           |   ✅   |
| Fedora           |   ✅   |
| Debian           |   ❓   |
| Arch             |   ❌   |

## Quick Start

1. Setup Docker. See [comprehensive Docker guide](./DOCKER.md).
2. Clone this repository.

With SSH

```
git clone git@github.com:MHSeals/ROS2-Docker-Crash-Course.git
```

With HTTPS

```
git clone https://github.com/MHSeals/ROS2-Docker-Crash-Course.git
```

3. Run the corresponding script

For example, if you are using Ubuntu, and have NVIDIA GPU, you should run

```
./run_container_ubuntu_nvidia.sh
```

4. Run the simulation

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

You should see a boat (WAMV) floating on the water with some obstacles.

Continue through the tutorial [at VRX Official
Guide](https://github.com/osrf/vrx/wiki/getting_around_tutorial).

5. Code your project

The folder `mhs_roboboat` will be mounted into the container for you. To build
it, use the command

```
cd $COLCON_WS && colcon build --symlink-install --packages-select mhs_roboboat && . install/setup.bash
```

Ideally, you only need to do this once, as it will create symbolic links to your
Python files. Any changes to those files will be updated on the fly. However,
should some error occurs where ROS complains it could not find something,
rebuilding and resourcing the setup files will most likely solves the problem.
