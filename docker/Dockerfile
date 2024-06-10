# Use the base ROS Noetic core image
FROM ros:noetic-ros-core-focal

# Install required dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-pip \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \
    wget \
    git \
    nano \
    vim \
    gedit \
    iproute2 \
    net-tools \
    libeigen3-dev \
    libgoogle-glog-dev \
    linux-tools-common \
    linux-tools-generic \
    linux-tools-$(uname -r) \
    && rm -rf /var/lib/apt/lists/*

RUN pip install pynput

# Set up catkin workspace and clone repositories
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone https://github.com/ethz-asl/mav_comm.git && \
    git clone https://github.com/achilleas2942/summer_school_controller.git

# Initialize rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-tf \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Build catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws/src && \
    catkin_init_workspace && \
    cd /root/catkin_ws/ && \
    catkin_make && \
    source /root/catkin_ws/devel/setup.bash"

# Copy the entrypoint script
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Source ROS setup scripts
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Update and upgrade the system
RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

# Set the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["roslaunch", "summer_school_controller", "udp_tunnels.launch"]
