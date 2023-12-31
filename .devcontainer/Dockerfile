FROM althack/ros:noetic-gazebo-nvidia

ENV DEBIAN_FRONTEND=noninteractive
#################################################################################################
### Install ROS packages
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
        ros-noetic-pcl*  \
        ros-noetic-teleop*\
        ros-noetic-opencv* \
        ros-noetic-depth-image-proc \
   # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
#################################################################################################
### Setup Ardupilot Dev Environment
RUN apt-get update \
     && apt-get -y install --no-install-recommends \
        build-essential ccache g++ gawk git make wget \
        python-is-python3 \
        libtool libxml2-dev libxslt1-dev python3-dev python3-pip python3-setuptools python3-matplotlib python3-serial python3-scipy python3-opencv python3-numpy python3-pyparsing xterm lcov gcovr \
     && apt-get remove modemmanager -y \
   # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade pip 
RUN pip install future lxml pymavlink MAVProxy pexpect 

## Install MAVROS
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
        ros-noetic-mavros ros-noetic-mavros-extras \
   # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod a+x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh

## QGround Control and afew other tools
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
        fuse libfuse2 python3-vcstool python3-wxgtk4.0 python3-lxml python3-pygame liburdfdom-tools\
   # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*

## Variables for simulator
ENV INSTANCE 0
ENV LAT 42.3898
ENV LON -71.1476
ENV ALT 14
ENV DIR 270
ENV MODEL +
ENV SPEEDUP 1
ENV VEHICLE ArduCopter

# TCP 5760 is what the sim exposes by default
EXPOSE 5760/tcp
#################################################################################################
## Pangolin
# WORKDIR /pangolin
# RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.5.zip -O Pangolin.zip \
#     && unzip Pangolin.zip \
#     && cd Pangolin-0.5 \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && cmake --build . \
#     && make install
#################################################################################################
# ### Install OpenCV
# ARG OPENCV_VERSION="3.4.16" 
# ## Install build dependencies
# RUN apt-get update \
#     && apt-get install -y --no-install-recommends \
#         libavdevice-dev libavfilter-dev libc++-10-dev libc++-dev libc++1-10 libc++abi1-10 ninja-build \
#         build-essential \
#         gfortran \
#         cmake \
#         git \
#         file \
#         wget \
#         zip \
#         unzip \
#         tar \
#         ffmpeg \
#         libatlas-base-dev \
#         libavcodec-dev \
#         libavformat-dev \
#         libcanberra-gtk3-module \
#         libeigen3-dev \
#         libglew-dev \
#         libgstreamer-plugins-base1.0-dev \
#         libgstreamer-plugins-good1.0-dev \
#         libgstreamer1.0-dev \
#         libgtk-3-dev \
#         libjpeg-dev \
#         libjpeg8-dev \
#         libjpeg-turbo8-dev \
#         liblapack-dev \
#         liblapacke-dev \
#         libopenblas-dev \
#         libpng-dev \
#         libpostproc-dev \
#         libswscale-dev \
#         libtbb-dev \
#         libtbb2 \
#         libtesseract-dev \
#         libtiff-dev \
#         libv4l-dev \
#         libxine2-dev \
#         libxvidcore-dev \
#         libx264-dev \
#         libgtkglext1 \
#         libgtkglext1-dev \
#         pkg-config \
#         qv4l2 \
#         v4l-utils \
#         zlib1g-dev \
#         locales \
#         && locale-gen en_US.UTF-8 \
#         && LC_ALL=en_US.UTF-8 \
#         && rm -rf /var/lib/apt/lists/* \
#         && apt-get clean

# ## Download OpenCV
# WORKDIR /opencv
# RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
#     && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
#     && unzip opencv.zip \
#     && unzip opencv_contrib.zip \
#     && mv opencv-${OPENCV_VERSION} opencv \
#     && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

# ## Build
# RUN mkdir /opencv/opencv/build
# WORKDIR /opencv/opencv/build
# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
#  -D CMAKE_INSTALL_PREFIX=/usr/local \
#  -D INSTALL_PYTHON_EXAMPLES=ON \
#  -D INSTALL_C_EXAMPLES=ON \
#  -D OPENCV_GENERATE_PKGCONFIG=ON \
#  -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules \
#  -D LAPACKE_H_PATH=/usr/include \
#  -D BUILD_EXAMPLES=ON .. \
#     && make -j$(nproc) && make install && ldconfig
#################################################################################################
### Install WSLg dependencies and vGPU acceleration 
## RUN apt-get update && apt install -y software-properties-common
## RUN add-apt-repository ppa:oibaf/graphics-drivers -y
### For Windows, uncomment the following lines
# RUN apt-get update \
#    && apt-get -y install \
#           vainfo \
#           mesa-va-drivers \
#           mesa-utils \
#    # Clean up
#         && apt-get autoremove -y \
#         && apt-get clean -y \
#         && rm -rf /var/lib/apt/lists/*
# ENV LIBVA_DRIVER_NAME=d3d12
# ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
# CMD vainfo --display drm --device /dev/dri/card0
#################################################################################################
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/devel/setup.bash ]; then source ${WORKSPACE}/devel/setup.bash; fi" >> /home/ros/.bashrc