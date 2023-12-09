ARG ROS_DISTRO=noetic
FROM althack/ros:$ROS_DISTRO-full

ENV DEBIAN_FRONTEND=noninteractive
#################################################################################################
### Install OpenCV
ARG OPENCV_VERSION="3.4.16" 
## Install build dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libavdevice-dev libavfilter-dev libc++-10-dev libc++-dev libc++1-10 libc++abi1-10 ninja-build \
        build-essential \
        gfortran \
        cmake \
        git \
        file \
        wget \
        zip \
        unzip \
        tar \
        ffmpeg \
        libatlas-base-dev \
        libavcodec-dev \
        libavformat-dev \
        libcanberra-gtk3-module \
        libeigen3-dev \
        libglew-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        libgstreamer1.0-dev \
        libgtk-3-dev \
        libjpeg-dev \
        libjpeg8-dev \
        libjpeg-turbo8-dev \
        liblapack-dev \
        liblapacke-dev \
        libopenblas-dev \
        libpng-dev \
        libpostproc-dev \
        libswscale-dev \
        libtbb-dev \
        libtbb2 \
        libtesseract-dev \
        libtiff-dev \
        libv4l-dev \
        libxine2-dev \
        libxvidcore-dev \
        libx264-dev \
        libgtkglext1 \
        libgtkglext1-dev \
        pkg-config \
        qv4l2 \
        v4l-utils \
        zlib1g-dev \
        locales \
        && locale-gen en_US.UTF-8 \
        && LC_ALL=en_US.UTF-8 \
        && rm -rf /var/lib/apt/lists/* \
        && apt-get clean

## Download OpenCV
WORKDIR /opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
    && unzip opencv.zip \
    && unzip opencv_contrib.zip \
    && mv opencv-${OPENCV_VERSION} opencv \
    && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

## Build
RUN mkdir /opencv/opencv/build
WORKDIR /opencv/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
 -D CMAKE_INSTALL_PREFIX=/usr/local \
 -D INSTALL_PYTHON_EXAMPLES=ON \
 -D INSTALL_C_EXAMPLES=ON \
 -D OPENCV_GENERATE_PKGCONFIG=ON \
 -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules \
 -D LAPACKE_H_PATH=/usr/include \
 -D BUILD_EXAMPLES=ON .. \
    && make -j$(nproc) && make install && ldconfig
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
### Install ROS packages
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
        ros-$ROS_DISTRO-pcl*  \
        ros-$ROS_DISTRO-opencv* \
   # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
#################################################################################################
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
# ARG WORKSPACE
# RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# WORKDIR $WORKSPACE
