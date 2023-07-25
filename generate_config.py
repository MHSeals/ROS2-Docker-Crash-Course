#!/usr/bin/env python3

import argparse
import pathlib
import os
import stat

current_path = str(pathlib.Path(__file__).parent.resolve())

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-o', '--os', type=str, default='linux', choices=['windows', 'linux'], help="Specify the OS of HOST machine")
parser.add_argument('-g', '--gpu', type=str, default='intel', choices=['intel', 'nvidia', 'amd'], help="Specify the available GPU of HOST machine")
parser.add_argument('-v', '--volume', type=str, default=None, help="Specify a HOST folder to mount to workspace as roboboat_package")
parser.add_argument('--fast', action='store_true', help='Use fast mirror relative to Dallas, Texas')
parser.add_argument('--mirror', type=str, default='http://mirror.us-tx.kamatera.com', help='URL to apt mirror, only in effect if --fast is used')
parser.add_argument('--extended-testing', action='store_true', help="Generate additional test scripts for debugging and verification")
parser.add_argument('--clean', action='store_true', help='Clean up the working directory')

args = parser.parse_args()

image = f'roboboat:2023-{args.os}-{args.gpu}'

def generate_test_script(test_command, test_name:str = '') -> None:
    if test_name != '': test_name = f'_{test_name}'
    with open(current_path + f'/test{test_name}.sh', 'w') as file:
        file.write('#!/usr/bin/env bash \n\n')
        if args.os == 'linux': file.write('xhost +\n\n')
        file.write('docker run \\\n')
        file.write('    --tty \\\n')
        file.write('    --rm \\\n')
        file.write('    --privileged \\\n')
        file.write('    --volume /tmp/.X11-unix:/tmp/.X11-unix \\\n')
        file.write('    --env DISPLAY=$DISPLAY \\\n')
        if args.os == 'linux' and args.gpu == 'nvidia':
            file.write('    --runtime nvidia \\\n')
        elif args.os == 'linux' and args.gpu == 'intel':
            file.write('    --device /dev/dri/card0 \\\n')
            file.write('    --device /dev/dri/renderD128 \\\n')
        elif args.os == 'linux' and args.gpu == 'amd':
            raise NotImplementedError
        elif args.os == 'windows':
            file.write('    --device /dev/dxg \\\n')
            file.write('    --device /dev/dri/card0 \\\n')
            file.write('    --device /dev/dri/renderD128 \\\n')
            file.write('    --volume /mnt/wslg:/mnt/wslg \\\n')
            file.write('    --volume /usr/lib/wsl:/usr/lib/wsl \\\n')
            file.write('    --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \\\n')
            file.write('    --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \\\n')
            file.write('    --env PULSE_SERVER=$PULSE_SERVER \\\n')
        file.write(f'    {image} \\\n')
        file.write(f'    {test_command}\n\n')
        if args.os == 'linux': file.write('xhost -\n')

if args.clean:
    files = os.listdir(current_path)
    for file in files:
        if file == 'Dockerfile' or file.endswith('.sh'):
            os.remove(current_path + f'/{file}')
else:
    lines = ["FROM osrf/ros:humble-desktop-full-jammy",
            "ARG DEBIAN_FRONTEND=noninteractive",
            "SHELL [ \"/bin/bash\", \"-c\" ]",
            f"RUN cp /etc/apt/sources.list /etc/apt/sources.list.original && \\\n    sed -i -r 's,http://(.*).ubuntu.com,{args.mirror},' /etc/apt/sources.list" if args.fast else None,
            "RUN apt-get update && apt-get -y --no-install-recommends install \\\n    git \\\n    curl \\\n    wget \\\n    build-essential \\\n    cmake \\\n    lsb-release \\\n    gnupg \\\n    gnupg2 \\\n    locales \\\n    net-tools \\\n    iputils-ping \\\n    netcat \\\n    software-properties-common \\\n    python3-dev \\\n    python3-pip \\\n    python-is-python3 \\\n    libxext6 \\\n    libx11-6 \\\n    libglvnd0 \\\n    libgl1 \\\n    libglx0 \\\n    libegl1 \\\n    freeglut3-dev \\\n    mesa-utils \\\n    mesa-utils-extra \\\n    libgl1-mesa-glx \\\n    libgl1-mesa-dri \\\n    && apt-get -y autoremove \\\n    && apt-get clean",
            "RUN add-apt-repository ppa:kisak/kisak-mesa && apt-get update && apt-get -y upgrade",
            "RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \\\n    echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main\" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \\\n    apt-get update && \\\n    apt-get -y install \\\n    ignition-fortress \\\n    ros-humble-turtlebot4-simulator \\\n    ros-humble-gazebo-* \\\n    ros-humble-cartographer \\\n    ros-humble-cartographer-ros \\\n    ros-humble-navigation2 \\\n    ros-humble-nav2-bringup \\\n    ros-humble-dynamixel-sdk \\\n    ros-humble-turtlebot3-msgs \\\n    ros-humble-turtlebot3 \\\n    ros-humble-velodyne \\\n    ros-humble-velodyne-simulator",
            "ENV LIBVA_DRIVER_NAME=d3d12" if args.os == 'windows' else None,
            "ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib" if args.os == 'windows' else None,
            "ENV MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel" if args.gpu == 'intel' and args.os == 'windows' else None,
            "ENV MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" if args.gpu == 'nvidia' and args.os == 'windows' else None,
            "ENV NVIDIA_DRIVER_CAPABILITIES all" if args.gpu == 'nvidia' else None,
            "ENV ROS_DISTRO humble",
            "ENV TURTLEBOT3_MODEL burger",
            "ENV QT_X11_NO_MITSHM 1",
            "ENV TERM xterm-256color",
            "ENV HOME /root",
            "ENV BOAT_WS /root/roboboat_ws",
            "ENV BOAT_SRC /root/roboboat_ws/src",
            "RUN mkdir -p ${BOAT_SRC}",
            "RUN git clone --progress -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git ${BOAT_SRC}/turtlebot3_simulations",
            "RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \\\n    cd ${BOAT_WS} && \\\n    rosdep update --include-eol-distros && \\\n    rosdep install --from-paths src -y --ignore-src && \\\n    colcon build --symlink-install",
            "RUN echo \"source /opt/ros/${ROS_DISTRO}/setup.bash\" >> ${HOME}/.bashrc && \\\n    echo \"source ${BOAT_WS}/install/setup.bash\" >> ${HOME}/.bashrc && \\\n    echo \"cd ${BOAT_WS}\" >> ${HOME}/.bashrc"]

    # generate Dockerfile
    with open(current_path + '/Dockerfile', 'w') as file:
        for line in lines:
            if line is not None:
                file.write(line + '\n\n')

    # generate build script
    with open(current_path + '/build.sh', 'w') as file:
        file.write('#!/usr/bin/env bash \n\n')
        file.write(f'docker build -t {image} .')

    # generate pull script
    with open(current_path + '/pull.sh', 'w') as file:
        file.write('#!/usr/bin/env bash \n\n')
        file.write(f'docker pull {image}')

    # generate test script(s)
    generate_test_script('glxinfo | grep "OpenGL"')

    if args.extended_testing:
        generate_test_script('glxgears', 'glxgears')
        generate_test_script('gazebo', 'gazebo_classic')
        generate_test_script('ign gazebo --render-engine ogre', 'gazebo')

    # generate run script
    with open(current_path + '/run.sh', 'w') as file:
        file.write('#!/usr/bin/env bash\n\n')
        file.write('CONTAINER_NAME=roboboat-container\n\n')
        file.write('CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`\n\n')
        file.write('if [ -z "${CONTAINER_ID}" ]; then\n\n')
        file.write('    docker run \\\n')
        file.write('        --tty \\\n')
        file.write('        --detach \\\n')
        file.write('        --name ${CONTAINER_NAME} \\\n')
        file.write('        --privileged \\\n')
        file.write('        --volume /tmp/.X11-unix:/tmp/.X11-unix \\\n')
        file.write('        --env DISPLAY=$DISPLAY \\\n')
        if args.os == 'linux' and args.gpu == 'nvidia':
            file.write('        --runtime nvidia \\\n')
        elif args.os == 'linux' and args.gpu == 'intel':
            file.write('        --device /dev/dri/card0 \\\n')
            file.write('        --device /dev/dri/renderD128 \\\n')
        elif args.os == 'linux' and args.gpu == 'amd':
            raise NotImplementedError
        elif args.os == 'windows':
            file.write('        --device /dev/dxg \\\n')
            file.write('        --device /dev/dri/card0 \\\n')
            file.write('        --device /dev/dri/renderD128 \\\n')
            file.write('        --volume /mnt/wslg:/mnt/wslg \\\n')
            file.write('        --volume /usr/lib/wsl:/usr/lib/wsl \\\n')
            file.write('        --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \\\n')
            file.write('        --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \\\n')
            file.write('        --env PULSE_SERVER=$PULSE_SERVER \\\n')
        file.write(f'        {image} \n\n')
        file.write('else\n\n')
        if args.os == 'linux': file.write("    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`\n\n")
        file.write('    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then\n')
        file.write('        docker start ${CONTAINER_ID}\n')
        file.write('    fi\n\n')
        file.write('    docker exec -it ${CONTAINER_ID} bash\n\n')
        if args.os == 'linux': file.write("    xhost -local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`\n\n")
        file.write('fi\n')
