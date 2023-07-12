# ROS and Docker Crash Course

TL;DR: Mac users are borked and limited to CLI tools only. Windows users are semi-borked because for some reason on some machines it renders and runs well, on others it's just a black screen. Linux (Ubuntu) users are in the clear, although other distro will need more testing.

> ⚠️ For now, proceed to [The Construct](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/) and learn more about ROS there until I figure out some simulation stuff for your boat.

Consequently, I will use Linux and Ubuntu interchangably. I know some of you Fedora/Arch/Debian folks will be mildly (or not so mildly) triggered. You're going to have to live with it, because officially ROS only supports Ubuntu, and all of their instructions assumes you are on the latest LTS version of Ubuntu.

For now markdown is the quickest to draft up a doc. Yes it is ugly and horribly unreadable considering how all the headings are omega similar, at least to me. See Issues for further planned improvements.

## Initial Preparation 🔧

It is probably a good idea to use the latest graphics driver available at the time of reading this, whether it is Intel, NVIDIA, or AMD. Different OS do this differently, so I won't include it here.

### Windows 

1. Install WSL 2 using [official instructions from Microsoft](https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command), or just open an elevated terminal/powershell and run

```
C:> wsl --install -d Ubuntu-22.04
```

and let it run. You will need to approve the installation at some point if you have User Account Control (UAC) enabled. After a required restart, you will be prompt for a username and password for your WSL 2.

2. (Optional) You may want to limit WSL 2 resources consumption by hard-limiting it via [.wslconfig](https://learn.microsoft.com/en-us/windows/wsl/wsl-config#configuration-setting-for-wslconfig). To do this, you need to create a `.wslconfig` file at your user directory, which is usually `C:\Users\your_username`, and put the following content in it

```coffee
# Settings apply across all Linux distros running on WSL 2
[wsl2]

# Limits VM memory to use no more than 4 GB, this can be set as whole numbers using GB or MB
memory=4GB 

# Sets the VM to use two virtual processors
processors=2
```

You can change this to whatever you're comfortable with or doesn't slow your workstation down. However, if possible, the lowest I would personally go is 4 processors and 8GB of RAM.

### MacOS

Similar to Windows, you may want to limit your resource usage, but instead of doing it through a file, you have the option to do it in the Docker Desktop GUI.

TODO: Borrow someone's Macbook to get a picture of the screen.

### Linux

TODO: Find out if limiting resources through UI is an option. It is only CLI for now.

## Docker Installation 🐋

I mulled over the use of distribution based Docker installation (through apt, etc.), but decided against it for the sake of uniformity. Aside from Linux, MacOS, and Ubuntu, you will need to find your instructions on how to install Docker Desktop on your machine.

See [Official Docker Desktop Installation Instruction](https://docs.docker.com/desktop/) for more details.

### Windows

Download the installer executable, run it, and restart your machine. Once you're back, check within WSL 2 if Docker is accessible. Be patient, Docker Desktop will take awhile to come online. Sample output from your WSL may look like this

![](Images/windows_docker.png)

If you get to this screen, you're ready to rock!

### MacOS

Download the installer and do your Mac Application drag and drop thing then start it. Confirm your installation upon gaining access to Docker through your terminal. MacOS permission is handled however it is handled, but it seems to work right out of the box.

TODO: Borrow someone's Macbook to get a picture of the screen.

### Linux

Mildly more complicated, but not horrible. It boils down to some security stuff, if you're interested, but otherwise it just mean that Docker will run as root 100% of the time and there are some implications, see [Docker Daemon Attack Surface](https://docs.docker.com/engine/security/#docker-daemon-attack-surface) for more details.

You can follow [official instructions](https://docs.docker.com/engine/install/linux-postinstall/) or just copy-paste the tl;dr commands below

```
$ sudo usermod -aG docker $USER
```

Restart or Relogin your account at this point. Once back in, open a new terminal and run

```
$ newgrp docker
```

You should now be able to run docker without the need of `sudo` to access the backend stuff (daemon, socket, etc.). Test it with

```
$ docker run --rm hello-world
```

The output should match exactly as shown in the picture above.

## Docker 101 📝

The starting point of all Docker container is a Docker Image. But in order to obtain or create an image, we need a Dockerfile. While you do not need to be comfortable with writing your own Dockerfile, I highly recommend that you do learn how to use them.

![](Images/docker1.png)

### But, why? 🤔

- Uniformity: Environment in which apps are deployed will be virtually identical. This solves a lot of headache in trying to recreate a similar, if not identical, working setup on someone else's computer.
- Isolation: If you do mess up, it's contained. Your host system remains unaffected (mostly) by whatever is going on in the container. Caveat: this relies heavily on how you control what container can and cannot do, but it is quite isolated by default.
- Flexibility: You may break a large application into smaller applications across different containers, so if any of those break, they are readily replaceable without intefering with other services.
- Scalability: And because you can easily replace them, you can also easily add more.

In our case, Docker unifies the deployment of ROS across all of our members' PCs, including the ones that will be on the boat itself. From there, we can leverage ROS to do more complicated things without having to worry about some interfacing stuff.

### Dockerfile

This is the blueprint that will define how a Docker image is constructed. I have included several to try and cover a wide range of OS. They may look similar at first glance, but they are non-trivially different.

The only thing you need to learn here, for now, is how to build an image from a Dockerfile, which is simply

```
$ docker build -t <image_name>:<image_tag> -f /path/to/Dockerfile .
```

`<image_name>` and `<image_tag>` can be mostly whatever you want, and `/path/to/Dockerfile` can be relative. Notice the dot at the end of the command. That is telling Docker that the build context is current directory. For now, you don't need to worry about it.

If the build is successful, you will have an image with given name and tag.

### Running Docker Container

For very simple cases, `docker run` would be a very simple command. However, unfortunately, it is not so for our case.

I will provide all necessary Dockerfile, build, test, and run script at a later time.

## Robot Operating System (ROS) 🤖

### What is it?

It is a standardized communication interface. That's it.

What that means is hardware abstraction is taken care of for you ahead of time, either by the hardware manufacturer or some good people of the free and open source community, and all you need to do is process a predefined message format.

### Why do we need it?

For our case, trying to write a program from scratch to talk to various sensors (RealSense, Velodyne, etc.), computing units (Jetson, Raspberry Pi, etc.), and control hardware (Pixhawk, motors, etc.) is tedious, difficult, and requires extremely high level of understanding and coordination to do.

ROS allows us to use a simple Python program to gain access and control to all of that, with the only tradeoff being we have to follow the ROS communication standard.

### But how can it (or I) do stuff like navigation?

While ROS is practically a communication layer, what you do with the information across such a network is up to your imagination. Whenever you hear a ROS package to do something, say image recognition, what it means is the package itself is an image recognition package, it just so happen that it receives images and response with information through ROS.

### Where can I learn more?

YouTube, unfortunately, is a hit-or-miss. It is absurdly hard to find a proper tutorial.

I recommend signing up and accessing free contents available at [The Construct](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/). They cover a wide range of topics from basic to fairly advance difficulty, and best of all, they have a web-based simulator ready for you to follow.

I suggest briefly go over some of the basic ones until you are comfortable with how ROS is organized, then moving on to some more advanced topic like distributed ROS over multiple machines. Being able to understand those concepts will help you figure out how to use ROS to solve your boat problem.