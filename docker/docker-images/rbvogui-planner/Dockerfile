# Build:
#  docker build --rm -f Dockerfile -t augmentedfabricationlab/ros-abb-fabtory-planner .
#
# Usage:
#  docker pull augmentedfabricationlab/ros-abb-fabtory-planner

FROM augmentedfabricationlab/ros-melodic-moveit
LABEL maintainer "Gido Dielemans <gido.dielemans@tum.de>"

SHELL ["/bin/bash","-c"]

ENV TERM xterm

ENV CATKIN_WS=/root/catkin_ws

RUN sudo apt install ros-melodic-moveit

# Add robot packages to local catkin workspace
RUN echo "Sourcing.." \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get update && apt-get -y upgrade \
    && apt-get install -y libserial-dev \
    # && apt-get install -y ros-melodic-descartes \
    # && apt-get update && apt-get -y upgrade \
    && git clone -b melodic-devel https://github.com/ros/xacro.git \
    # && git clone -b melodic-devel https://github.com/RobotnikAutomation/rbvogui_common.git \
    && git clone -b melodic-devel-desc-only https://github.com/gidodielemans/rbvogui_common \
    && git clone -b melodic-devel https://github.com/RobotnikAutomation/robotnik_sensors.git \
    # && git clone https://github.com/RobotnikAutomation/velodyne_simulator.git \
    # && git clone https://github.com/RobotnikAutomation/robotnik_msgs.git \
    && git clone https://github.com/RobotnikAutomation/ewellix_description.git \
    && git clone https://github.com/augmentedfabricationlab/robotic_setups_description.git 
    # && git clone https://github.com/ros-industrial-consortium/descartes.git \
    # && git clone -b melodic-collision-minimum https://github.com/PickNikRobotics/descartes.git \
    # && git clone -b melodic-dev https://github.com/gidodielemans/descartes_capability.git 
    
RUN echo "Building tools..." \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*