FROM althack/ros:noetic-full

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
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends \
#         fuse libfuse2 python3-vcstool python3-wxgtk4.0 python3-lxml python3-pygame liburdfdom-tools\
#    # Clean up
#         && apt-get autoremove -y \
#         && apt-get clean -y \
#         && rm -rf /var/lib/apt/lists/*
#################################################################################################
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/devel/setup.bash ]; then source ${WORKSPACE}/devel/setup.bash; fi" >> /home/ros/.bashrc