sudo apt-get update && sudo apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release

# setup keys
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# install bootstrap tools
sudo apt-get update && sudo apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools

# bootstrap rosdep
sudo rosdep init \
    && sudo rosdep update

# install ros packages
export ROS_DISTRO=kinetic
sudo apt-get update && sudo apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0*

# install universal robot ros packages
sudo apt-get update \
&& sudo apt-get install -y \
    ros-kinetic-universal-robot \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-gripper-action-controller \
    ros-kinetic-orocos-kdl \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-catkin \
    ros-kinetic-industrial-core \
    ros-kinetic-rqt \
    ros-kinetic-rqt-common-plugins \
    python-catkin-tools \
    libboost-dev \
    libboost-python-dev \
    libboost-system-dev \
    libboost-all-dev \
    libatlas-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python-setuptools \
    python-pip \
    python-tk \
    python-qt4

# Set up the workspace
source /opt/ros/kinetic/setup.bash \
&& mkdir -p ~/ros_ws/src \
&& cd ~/ros_ws/ \
&& catkin init \
&& echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc

# Installing repo required for homework
cd ~/ros_ws/src \
&& git clone https://github.com/cambel/ur3.git ur3

# # Updating ROSDEP and installing dependencies
cd ~/ros_ws \
&& rosinstall ~/ros_ws/src /opt/ros/kinetic src/ur3/dependencies.rosinstall \
&& sudo apt-get update \
&& sudo rosdep fix-permissions \
&& sudo rosdep update \
&& sudo rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

# Numpy Boost
cd /usr/lib/x86_64-linux-gnu/ \
&& cd \
&& git clone https://github.com/ndarray/Boost.NumPy.git \
&& cd Boost.NumPy \
&& cmake . \
&& make -j4 \
&& sudo make install

# Sourcing
source /opt/ros/kinetic/setup.bash \
&& cd ~/ros_ws/ \
&& rm -rf build \
&& catkin build

python -m pip install pip --upgrade && \
pip install matplotlib==2.2.3 spicy protobuf pyyaml pyquaternion rospkg lxml tqdm tensorflow-gpu==1.4.0 pillow

# Installing repo required for homework
cd ~/ \
&& git clone --branch ros_agent_interface https://oath2:wzjvUHnkVWsdQxEyygVc@gitlab.com/cambel/gps.git gps

cd ~/gps \
&& ./compile_proto.sh

# Gazebo
# setup keys
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

cat /etc/apt/sources.list.d/gazebo-stable.list
deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

export DEBIAN_FRONTEND=noninteractive

# install gazebo packages
sudo apt-get update \
&& sudo apt-get install -q -y \
    binutils \
    mesa-utils \
    module-init-tools \
    x-window-system \
    gazebo7=7.15.0-1* \
    libgazebo7-dev=7.15.0-1*

# Fix for Boost Numpy
echo 'export LD_LIBRARY_PATH=/usr/local/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:~/gps' >> ~/.bashrc
