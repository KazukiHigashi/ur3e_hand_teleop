FROM shadowrobot/dexterous-hand:kinetic-night-build

LABEL Description="This ROS Kinetic image contains Shadow's dexterous hand software with build tools. It includes IDE environments. Nvidia compatible" Vendor="Shadow Robot" Version="1.0"


# python-vcstools : to use uptodate vcstools in rosinstall
# python-catkin-tools : to use catkin command from this dockerfile
RUN apt-get update \
&& apt-get install -y --no-install-recommends pkg-config libxau-dev libxdmcp-dev libxcb1-dev libxext-dev libx11-dev python-catkin-tools python-vcstools\
&& rm -rf /var/lib/apt/lists/*

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

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"

ENV PATH /usr/local/nvidia/bin:${PATH}

ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN cd /usr/lib/x86_64-linux-gnu/ \
    && cd \
    && git clone https://github.com/ndarray/Boost.NumPy.git \
    && cd Boost.NumPy \
    && cmake . \
    && make -j4 \
    && make install

RUN python -m pip install pip --upgrade && \
    pip install matplotlib==2.2.3 spicy protobuf pyyaml pyquaternion rospkg lxml tqdm sklearn

# Fix for Boost Numpy
ENV LD_LIBRARY_PATH /usr/local/lib64:$LD_LIBRARY_PATH

# Git Cloning
RUN cd /home/user/projects/shadow_robot/base/src \
    && git clone https://github.com/cambel/ur3.git ur3 \
    && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver \
    && git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot \
    && git clone https://github.com/ozymandium/cereal-port.git \
    && git clone https://github.com/shadow-robot/sr_utils.git

RUN pip install Cython

# Updating ROSDEP and installing dependencies
RUN cd /home/user/projects/shadow_robot/base \
    && rosinstall /home/user/projects/shadow_robot/base/src /opt/ros/kinetic src/ur3/dependencies.rosinstall\
    && apt-get update \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y \
    && rm -rf /var/lib/apt/lists/*

RUN cd /home/user/projects/shadow_robot/base/src \
    && git clone https://github.com/cambel/ur_ikfast.git \
    && cd ur_ikfast \
    && pip install -e .

RUN cd /home/user/projects/shadow_robot/base/ \
    && source devel/setup.bash \
    && catkin_make
