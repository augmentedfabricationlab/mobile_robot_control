# Build:
#  docker build --rm -f Dockerfile -t augmentedfabricationlab/ros-abb-fabtory-planner .
#
# Usage:
#  docker pull augmentedfabricationlab/ros-abb-fabtory-planner

FROM augmentedfabricationlab/ros-melodic-moveit
LABEL maintainer "Gido Dielemans <gido.dielemans@tum.de>"

SHELL ["/bin/bash","-c"]
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src

COPY . /root/catkin_ws/src

# Add robot packages to local catkin workspace
RUN echo "Building tools for ROS" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get update \
    # && apt-get update && apt-get -y upgrade \
    && git clone https://github.com/RobotnikAutomation/joint_read_command_controller.git \
    && git clone -b melodic-devel https://github.com/RobotnikAutomation/rbvogui_common.git \
    && git clone -b melodic-1.0.0 https://github.com/RobotnikAutomation/robotnik_msgs.git \
    && git clone -b melodic-devel https://github.com/RobotnikAutomation/robotnik_sensors.git \
    && git clone https://github.com/RobotnikAutomation/robotnik_pad.git \
    && git clone -b melodic-devel https://github.com/RobotnikAutomation/rbvogui_sim.git \
    && git clone https://github.com/RobotnikAutomation/rcomponent.git \
    && git clone https://github.com/RobotnikAutomation/teleop_panel.git \
    && git clone https://github.com/RobotnikAutomation/ewellix_description.git \
    && git clone https://github.com/RobotnikAutomation/velodyne_simulator.git \
    && git clone -b melodic-devel https://github.com/ros/xacro.git \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}\
    # Build catkin workspace
    && catkin_make \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*
