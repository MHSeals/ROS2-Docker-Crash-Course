
This is a slightly more verbose, and it is technically the old guide.

## Install Docker

For Windows, use Docker Desktop. Download at https://www.docker.com/products/docker-desktop/.

For Linux, use `sudo apt install docker.io` and complete the following:

1. Post Installation steps for Linux. See https://docs.docker.com/engine/install/linux-postinstall/.
2. (Optional for NVIDIA) Install NVIDIA Container Toolkit. See https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#setting-up-nvidia-container-toolkit.

Test your installation with `docker run --rm hello-world`. You should see:

```
mqt00xx@RVL-PRECISION-001:~$ docker run --rm hello-world

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/

mqt00xx@RVL-PRECISION-001:~$
```

## Clone this Repo

Setup SSH Key if you haven't already. See https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh.

Boils down to:

1. Create a key

> ⚠️ If you already have a public-private key pair and your SSH Agent is aware of it, skip to step 3

```
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Just hit Enter to continue with default values.

2. Add key to SSH Agent

You will need to start an agent first

```
eval "$(ssh-agent -s)"
```

Then add your private key

```
ssh-add ~/.ssh/id_ed25519
```

3. Add your public key to GitHub using instructions at https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account.

4. Clone the repository using SSH instead of HTTPS

```
git clone git@github.com:MHSeals/ROS2-Docker-Crash-Course.git
```

<!-- ### Prepare your Docker Image

The included `generate_config.py` will produce all the scripts you need. You should run

```bash
python3 generate_config.py --help
# or ./generate_config.py --help
```

to see the available customization.

For example, Windows 11 host with an NVIDIA GPU should run

```
python3 generate_config.py --os windows --gpu nvidia
```

This will also spawn `pull.sh`, `build.sh`, `test.sh`, and `run.sh`.

At this point you'll probably need to build your own image for now. It may take awhile.

```
bash build.sh
```

## Testing the Image

Verify that the rendering string matches your GPU. In my case, that's NVIDIA Quadro P4000.

```
mqt00xx@RVL-PRECISION-001:~/Workspace/Roboboat-Tutorial$ bash test.sh
OpenGL vendor string: Microsoft Corporation
OpenGL renderer string: D3D12 (NVIDIA Quadro P4000)
OpenGL core profile version string: 4.2 (Core Profile) Mesa 23.1.3 - kisak-mesa PPA
OpenGL core profile shading language version string: 4.20
OpenGL core profile context flags: (none)
OpenGL core profile profile mask: core profile
OpenGL core profile extensions:
OpenGL version string: 4.2 (Compatibility Profile) Mesa 23.1.3 - kisak-mesa PPA
OpenGL shading language version string: 4.20
OpenGL context flags: (none)
OpenGL profile mask: compatibility profile
OpenGL extensions:
OpenGL ES profile version string: OpenGL ES 3.1 Mesa 23.1.3 - kisak-mesa PPA
OpenGL ES profile shading language version string: OpenGL ES GLSL ES 3.10
OpenGL ES profile extensions:
```

### Running the Container

The `run.sh` will spawn a *persistent* container, and subsequent execution of `run.sh` will start and attach to the container for you.

```
$ bash run.sh
5e329ff82aea6dd82c7001501e11980be574d3e52fb4ff098bfc2ce9aa25f628

$ bash run.sh
root@5e329ff82aea:~/roboboat_ws#
```

And you're in!

## Testing GUI Spawning and Performance

You should be able to spawn GUI windows. You can run a simple test case using `glxgears`. It should spawn a windows with colored gears.

You can also start a simulation with Turtlebot 3 using

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Turtlebot 4 is also included, but at this time **Windows will crash when trying to use Ignition Gazebo.**

```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
``` -->
