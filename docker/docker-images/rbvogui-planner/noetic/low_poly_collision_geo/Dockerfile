# Build:
#  docker build --rm -f Dockerfile -t daricht/rbvogui:latest .
#
# Usage:
#  docker pull augmentedfabricationlab/ros-abb-fabtory-planner

FROM augmentedfabricationlab/ros-noetic-moveit
LABEL maintainer "David Richter <david.richter@tum.de>"

SHELL ["/bin/bash","-c"]

ENV TERM xterm

ENV CATKIN_WS=/root/catkin_ws

# RUN sudo apt install ros-noetic-moveit  # this is already done in the parent image

# Add robot packages to local catkin workspace
RUN echo "Sourcing.." \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get -y update && apt-get -y upgrade \
    && apt-get install -y libserial-dev wget \
    # && apt-get install -y ros-noetic-descartes \
    # && apt-get update && apt-get -y upgrade \
    && git clone -b noetic-devel https://github.com/ros/xacro.git \
    # && git clone -b noetic-devel https://github.com/RobotnikAutomation/rbvogui_common.git \
    && git clone -b melodic-devel-desc-only https://github.com/gidodielemans/rbvogui_common \
    && git clone -b melodic-devel https://github.com/RobotnikAutomation/robotnik_sensors.git \
    # && git clone https://github.com/RobotnikAutomation/velodyne_simulator.git \
    && git clone https://github.com/RobotnikAutomation/robotnik_msgs.git \
    && git clone https://github.com/RobotnikAutomation/ewellix_description.git \
    && git clone https://github.com/augmentedfabricationlab/robotic_setups_description.git
   # && git clone https://github.com/utexas-bwi/eband_local_planner.git

#update collision geometry
RUN rm -rf /root/catkin_ws/src/robotnik_sensors/meshes/rs_bpearl.stl 
RUN rm -rf /root/catkin_ws/src/robotnik_sensors/meshes/antenna_ANN_MB_mast.stl
RUN rm -rf /root/catkin_ws/src/ewellix_description/meshes/ewellix_lift_base_700mm.stl
RUN rm -rf /root/catkin_ws/src/ewellix_description/meshes/ewellix_lift_top_700mm.stl

RUN wget -O /root/catkin_ws/src/robotnik_sensors/meshes/antenna_ANN_MB_mast.stl https://github.com/daricht/collision_geometry/blob/main/antenna_ANN_MB_mast.stl?raw=true
RUN wget -O /root/catkin_ws/src/robotnik_sensors/meshes/rs_bpearl.stl https://github.com/daricht/collision_geometry/raw/main/rs_bpearl.stl?raw=true
RUN wget -O /root/catkin_ws/src/ewellix_description/meshes/ewellix_lift_base_700mm.stl https://github.com/daricht/collision_geometry/blob/main/ewellix_lift_base_700mm.stl?raw=true
RUN wget -O /root/catkin_ws/src/ewellix_description/meshes/ewellix_lift_top_700mm.stl https://github.com/daricht/collision_geometry/blob/main/ewellix_lift_top_700mm.stl?raw=true

    
RUN echo "Building tools..." \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*