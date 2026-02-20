ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=jazzy_ws
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
        libcap-dev


# Upgrade installed packages
RUN apt update && apt upgrade -y && apt clean

# Setting up locale stuff
RUN apt update && apt install locales

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8


RUN wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && apt-key add ros.asc
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Additional dependencies needed for rosidl_generator_c
RUN apt update && apt install -y \
    pkg-config \
    python3-yaml \
    cmake-extras

# Install Boost libraries needed for OMPL
RUN apt update && apt install -y \
    libboost-all-dev \
    libboost-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-serialization-dev \
    libboost-date-time-dev \
    libboost-regex-dev \
    libboost-python-dev \
    libfmt-dev

# Install dependencies for geometric_shapes and other packages
RUN apt update && apt install -y \
    libqhull-dev \
    libassimp-dev \
    liboctomap-dev \
    libconsole-bridge-dev \
    libfcl-dev

# Install Eigen3 needed for OMPL and MoveIt
RUN apt update && apt install -y \
    libeigen3-dev

# Install X11 and graphics dependencies needed for OGRE (RViz)
RUN apt update && apt install -y \
    libx11-dev \
    libxaw7-dev \
    libxrandr-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
    libgles2-mesa-dev \
    libopengl-dev \
    libfreetype-dev \
    libfreetype6-dev \
    libfontconfig1-dev \
    libfmt-dev

# Install Qt5 and additional dependencies for RViz
RUN apt update && apt install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5core5a \
    libqt5gui5 \
    libqt5opengl5 \
    libqt5widgets5 \
    libxcursor-dev \
    libxinerama-dev \
    libxi-dev \
    libyaml-cpp-dev \
    libassimp-dev \
    libzzip-dev \
    freeglut3-dev \
    libogre-1.9-dev \
    libpng-dev \
    libjpeg-dev \
    python3-pyqt5.qtwebengine

RUN apt update && apt install -q -y \
  python3-pip \
  python3-pytest-cov \
  python3-rosinstall-generator \
  ros-dev-tools \
  libbullet-dev \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev \
  libacl1-dev \
  python3-empy \
  libpython3-dev \
  liblttng-ust-dev

# BUG: when build this gets overwritten to a newer verison which messes up with colcon build --symlink-install
RUN pip3 install --break-system-packages setuptools==79.0.1

# Install the correct version of empy that is compatible with ROS 2 jazzy
# Uninstall any existing empy first, then install version 3.3.4 specifically
RUN python3 -m pip uninstall -y em empy || true
RUN python3 -m pip install --break-system-packages empy==3.3.4

RUN python3 -m pip install --break-system-packages -U --ignore-installed \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  rospkg \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  lark

RUN python3 -m pip uninstall numpy -y || true
RUN python3 -m pip install --break-system-packages --ignore-installed --upgrade pip
RUN python3 -m pip install --break-system-packages --ignore-installed numpy pybind11 PyYAML

# Fix paths for pybind11
RUN python3 -m pip install --break-system-packages --ignore-installed "pybind11[global]"

RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} rosidl_runtime_c rcutils rcl rmw tf2 tf2_msgs common_interfaces geometry_msgs nav_msgs std_msgs rosgraph_msgs sensor_msgs vision_msgs rclpy ros2topic ros2pkg ros2doctor ros2run ros2node ros_environment ackermann_msgs example_interfaces angles rclcpp hardware_interface > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

RUN rosdep init && rosdep update

# Use logging to help debug build issues
RUN cd ${ROS_ROOT} && colcon build --merge-install

# Need these to maintain compatibility on non 20.04 systems
RUN cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* /workspace/jazzy_ws/install/lib/ || true
RUN cp /usr/lib/x86_64-linux-gnu/libssl.so* /workspace/jazzy_ws/install/lib/ || true
RUN cp /usr/lib/x86_64-linux-gnu/libcrypto.so* /workspace/jazzy_ws/install/lib/ || true

# Make sure we're in the right directory
WORKDIR /workspace

COPY jazzy_ws/src /workspace/jazzy_ws/src


RUN git clone https://github.com/sergey-khl/bear_ide.git \
    && cd bear_ide \
    && ./install.sh

# RUN apt-get update && \
#     rosdep install -y -r --from-paths ${ROS_ROOT}/src --ignore-src --rosdistro ${ROS_DISTRO} && \
#     apt-get install -y ros-${ROS_DISTRO}-moveit
RUN /bin/bash -c "source ${ROS_ROOT}/install/setup.sh && apt-get update && rosdep install -y -r --from-paths jazzy_ws/src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys 'rti-connext-dds-6.0.1' && apt-get install -y ros-${ROS_DISTRO}-moveit"

# # 5. Build the single workspace (sourcing the underlay so it finds MoveIt)
# RUN /bin/bash -c "if [ -f /opt/ros/${ROS_DISTRO}/setup.sh ]; then source /opt/ros/${ROS_DISTRO}/setup.sh; fi && \
#     cd /workspace/${ROS_ROOT} && \
#     colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && source ${ROS_ROOT}/install/setup.sh && cd jazzy_ws && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF"


RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/${ROS_ROOT}/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc


# # install everything we need
# RUN /bin/bash -c "source ${ROS_ROOT}/install/setup.sh && apt-get update && rosdep install -y -r --from-paths jazzy_ws/src --ignore-src --rosdistro ${ROS_DISTRO} && apt-get install -y ros-${ROS_DISTRO}-moveit"
