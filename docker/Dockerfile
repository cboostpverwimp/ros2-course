FROM ros:jazzy-ros-base

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    wget \
    x11-xserver-utils \
    software-properties-common \
    python3-pip \
    xauth \
    ros-${ROS_DISTRO}-cartographer-ros \ 
    ros-${ROS_DISTRO}-can-msgs \ 
    ros-${ROS_DISTRO}-controller-manager \ 
    ros-${ROS_DISTRO}-control-msgs \ 
    ros-${ROS_DISTRO}-demo-nodes-cpp \ 
    ros-${ROS_DISTRO}-demo-nodes-py \ 
    ros-${ROS_DISTRO}-gripper-controllers \ 
    ros-${ROS_DISTRO}-image-transport \ 
    ros-${ROS_DISTRO}-image-tools \ 
    ros-${ROS_DISTRO}-image-view \ 
    ros-${ROS_DISTRO}-interactive-marker-twist-server \ 
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joy-linux \ 
    ros-${ROS_DISTRO}-kinematics-interface \ 
    ros-${ROS_DISTRO}-moveit \ 
    ros-${ROS_DISTRO}-moveit-msgs \ 
    ros-${ROS_DISTRO}-moveit-py \ 
    ros-${ROS_DISTRO}-moveit-planners-chomp \ 
    ros-${ROS_DISTRO}-ompl \ 
    ros-${ROS_DISTRO}-plotjuggler-ros \ 
    ros-${ROS_DISTRO}-realtime-tools \ 
    ros-${ROS_DISTRO}-ros2-control \ 
    ros-${ROS_DISTRO}-ros2-controllers \ 
    ros-${ROS_DISTRO}-ros-gz \ 
    ros-${ROS_DISTRO}-rqt-console \ 
    ros-${ROS_DISTRO}-rqt-graph \ 
    ros-${ROS_DISTRO}-rqt-service-caller \ 
    ros-${ROS_DISTRO}-srdfdom \ 
    ros-${ROS_DISTRO}-teleop-twist-joy \ 
    ros-${ROS_DISTRO}-teleop-twist-keyboard \ 
    ros-${ROS_DISTRO}-test-msgs \ 
    ros-${ROS_DISTRO}-turtlesim \ 
    ros-${ROS_DISTRO}-twist-mux \
    '~nros-${ROS_DISTRO}-rqt*' \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -G video -m $USERNAME \
    # Add sudo support
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Gazebo Sim dependencies, Python support
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget -O- https://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
    libgz-transport14-dev \
    python3-gz-transport14

ENV SHELL=/bin/bash

USER $USERNAME

# Download and install Tiago Harmonic
RUN mkdir -p /home/${USERNAME}/tiago_ws/src
WORKDIR /home/${USERNAME}/tiago_ws/src
RUN git clone https://github.com/Tiago-Harmonic/tiago_harmonic.git -b ${ROS_DISTRO}
RUN vcs import . < tiago_harmonic/dependencies.repos
RUN wget https://raw.githubusercontent.com/ymollard/pal_gripper/6b51bf9fa56864fb03ef06400a580592c31f8794/pal_gripper_description/urdf/gripper.urdf.xacro -O /home/${USERNAME}/tiago_ws/src/pal_gripper/pal_gripper_description/urdf/gripper.urdf.xacro
RUN rm -rf /home/${USERNAME}/tiago_ws/src/br2_gazebo_worlds/
RUN git clone https://github.com/ymollard/br2_gazebo_worlds.git  # Replace br2_gazebo_worlds with customized worlds
WORKDIR /home/${USERNAME}/tiago_ws
RUN sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src -y -r
RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon build --symlink-install"

# Create workspace so that user owns this directory
RUN mkdir -p /home/${USERNAME}/ros2_ws/src
WORKDIR /home/${USERNAME}/ros2_ws
# Install rosdeps from packages
RUN sudo apt update && rosdep update \
    && rosdep install --from-paths src --ignore-src -y -r

# Add colcon mixin default
RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; colcon mixin update default"

# Build Workspace
WORKDIR /home/${USERNAME}/ros2_ws/
RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon build --symlink-install"

CMD ["bash"]
