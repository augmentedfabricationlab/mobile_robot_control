# Build:
#  docker build --rm -f Dockerfile -t daricht/rbvogui:latest .
#
# Usage:
#  docker pull augmentedfabricationlab/ros-abb-fabtory-planner

FROM augmentedfabricationlab/ros-noetic-moveit
LABEL maintainer "Gido Dielemans <gido.dielemans@tum.de>"

SHELL ["/bin/bash","-c"]

ENV CATKIN_WS=/root/catkin_ws
WORKDIR $CATKIN_WS/src
# RUN sudo apt install ros-noetic-moveit  # this is already done in the parent image

# Add robot packages to local catkin workspace
RUN echo "Sourcing.." \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get update && apt-get -y upgrade \
    && apt-get install -y libserial-dev \
    && git clone -b noetic-devel https://github.com/ros/xacro.git \
    # && git clone https://github.com/RobotnikAutomation/joint_read_command_controller.git \
    && git clone -b noetic-devel-desc-only https://github.com/gidodielemans/rbvogui_common.git \
    # && git clone -b melodic-1.0.0 https://github.com/RobotnikAutomation/robotnik_msgs.git \
    && git clone -b noetic-devel https://github.com/RobotnikAutomation/robotnik_sensors.git \
    # && git clone https://github.com/RobotnikAutomation/robotnik_pad.git \
    # && git clone -b melodic-devel https://github.com/RobotnikAutomation/rbvogui_sim.git \
    # && git clone https://github.com/RobotnikAutomation/rcomponent.git \
    # && git clone https://github.com/RobotnikAutomation/teleop_panel.git \
    && git clone https://github.com/RobotnikAutomation/ewellix_description.git \
    # && git clone https://github.com/generationrobots-lab/ewellix_tlt.git \
    # && git clone https://github.com/RobotnikAutomation/velodyne_simulator.git \
    && git clone -b no_dae_files_for_macintosh https://github.com/augmentedfabricationlab/robotic_setups_description.git \
    && git clone https://github.com/RobotWebTools/tf2_web_republisher \
    && echo "Building tools..." \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*
    