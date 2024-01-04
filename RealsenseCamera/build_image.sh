#! /bin/sh
echo "Copy Dockerfile to librealsense/scripts/Docker"
cp Dockerfile ../ThirdParty/librealsense/scripts/Docker
echo "Build docker image for RealsenseCamera"
bash ../ThirdParty/librealsense/scripts/Docker/build_image.sh
echo "Build finished"