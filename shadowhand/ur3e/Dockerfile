# Use an official ROS Kinetic runtime as a parent image
FROM ros:kinetic-ros-base-xenial

LABEL maintainer Cristian Beltran "beltran@hlab.sys.es.osaka-u.ac.jp"

# install universal robot ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-universal-robot \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-gripper-action-controller \
    ros-kinetic-orocos-kdl \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-rosserial \
    ros-kinetic-rosserial-arduino \
    # install catkin
    ros-kinetic-catkin \
    python-catkin-tools \
    # Install Numpy Boost
    libboost-dev \
    libboost-python-dev \
    libboost-system-dev \
    libboost-all-dev \
    libatlas-dev \
    libprotobuf-dev \
    protobuf-compiler \
    # python dependencies
    python-setuptools \
    python-pip \
    python-tk \
    # utils
    locate \
    aptitude \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
                && mkdir -p ~/ros_ws/src \
                && cd ~/ros_ws/ \
                && catkin init \
                && echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc "

# Installing repo required for homework
RUN /bin/bash -c "cd ~/ros_ws/src \
               && git clone https://github.com/cambel/ur3.git ur3"


# # Updating ROSDEP and installing dependencies
RUN cd ~/ros_ws \
    && rosinstall ~/ros_ws/src /opt/ros/kinetic src/ur3/dependencies.rosinstall \
    && apt-get update \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "cd ~/ros_ws/src \
                  && git clone https://github.com/shadow-robot/sr-teleop.git \
                  && git clone https://github.com/shadow-robot/sr_common \
                  && git clone https://github.com/shadow-robot/sr_core \
                  && git clone https://github.com/shadow-robot/ros_ethercat \
                  && git clone https://github.com/shadow-robot/sr-config"

# # Updating ROSDEP and installing dependencies
RUN cd ~/ros_ws \
    && rosinstall ~/ros_ws/src /opt/ros/kinetic src/sr-teleop/repository.rosinstall \
    && apt-get update \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y \
    && rm -rf /var/lib/apt/lists/*

RUN cd /usr/lib/x86_64-linux-gnu/ \
    && cd \
    && git clone https://github.com/ndarray/Boost.NumPy.git \
    && cd Boost.NumPy \
    && cmake . \
    && make -j4 \
    && make install

# Sourcing
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
               && cd ~/ros_ws \
               && rm -rf build \
               && catkin build"

RUN python -m pip install pip --upgrade && \
    pip install matplotlib==2.2.3 spicy protobuf pyyaml pyquaternion rospkg lxml tqdm

# Gazebo
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

ENV DEBIAN_FRONTEND noninteractive

# install gazebo packages
RUN apt-get update && apt-get install -q -y \
    binutils \
    mesa-utils \
    module-init-tools \
    x-window-system \
    gazebo7=7.15.0-1* \
    libgazebo7-dev=7.15.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup environment
EXPOSE 11345

# Fix for Boost Numpy
ENV LD_LIBRARY_PATH /usr/local/lib64:$LD_LIBRARY_PATH

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/lib/nvidia-418/bin:$PATH
# Fix for nvidia drivers
ENV LD_LIBRARY_PATH /usr/lib/nvidia-418:/usr/lib32/nvidia-418:$LD_LIBRARY_PATH

RUN apt-get update && apt-get install -y ros-kinetic-industrial-core \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "cd ~/ros_ws/src \
               && git clone https://github.com/ros-industrial/ur_modern_driver.git \
               && cd ur_modern_driver \
               && git fetch origin pull/310/head:ur-e && git checkout ur-e \
               && catkin build ur_modern_driver "

CMD ["bash"]

# docker create -it \
#     --env="DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --device=/dev/dri:/dev/dri \
#     --privileged -e DISPLAY=unix$DISPLAY \
#     -v /usr/lib/nvidia-390:/usr/lib/nvidia-390 \
#     -v /usr/lib32/nvidia-390:/usr/lib32/nvidia-390 \
#     --name=ros-gps \
#     registry.gitlab.com/cambel/gps && \
#  export containerId=$(docker ps -l -q) && \
#  xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId` && \
