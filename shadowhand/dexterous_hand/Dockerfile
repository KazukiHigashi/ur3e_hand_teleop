FROM shadowrobot/dexterous-hand:kinetic-night-build

RUN apt-get update \
&& apt-get install -y --no-install-recommends pkg-config libxau-dev libxdmcp-dev libxcb1-dev libxext-dev libx11-dev \
ros-kinetic-rosserial ros-kinetic-rosserial-arduino \
&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-kinetic-universal-robot \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-gripper-action-controller \
    ros-kinetic-orocos-kdl \
    ros-kinetic-gazebo-ros-pkgs \
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
# RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
#                && mkdir -p ~/projects/shadow_robot/base/src \
#                && cd /home/user/projects/shadow_robot/base/ \
#                && catkin init \
#                && echo 'source ~/projects/shadow_robot/base/devel/setup.bash' >> ~/.bashrc "

# Installing repo required for homework
RUN /bin/bash -c "cd /home/user/projects/shadow_robot/base/src \
               && git clone https://github.com/cambel/ur3.git ur3"

# # Updating ROSDEP and installing dependencies
RUN cd /home/user/projects/shadow_robot/base \
    && rosinstall /home/user/projects/shadow_robot/base/src /opt/ros/kinetic src/ur3/dependencies.rosinstall \
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
               && cd /home/user/projects/shadow_robot/base/ \
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

RUN /bin/bash -c "cd /home/user/projects/shadow_robot/base/src \
               && git clone https://github.com/ros-industrial/ur_modern_driver.git \
               && cd ur_modern_driver \
               && git fetch origin pull/310/head:ur-e && git checkout ur-e \
               && catkin build ur_modern_driver "


# replace with other Ubuntu version if desired
# see: https://hub.docker.com/r/nvidia/opengl/
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 /usr/local/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu

# replace with other Ubuntu version if desired
# see: https://hub.docker.com/r/nvidia/opengl/
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf \
&& ldconfig \
&& echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload \
&& echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


