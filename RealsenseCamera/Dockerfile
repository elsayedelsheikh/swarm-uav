ARG BASE_IMAGE=librealsense
#################################
#   Librealsense Builder Stage  #
#################################
FROM $BASE_IMAGE as realsenseros

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Install ROS
RUN apt-get update && apt-get install -y \
    curl \
    dirmngr \
    gnupg2 \
    lsb-release \
    sudo \
  && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noeitc-realsense2-camera \
    ros-noetic-realsense2-description \
  && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=1
ENV PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
ENV PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic
ENV DEBIAN_FRONTEND=

# Shows a list of connected Realsense devices
CMD [ "rs-enumerate-devices", "--compact" ]