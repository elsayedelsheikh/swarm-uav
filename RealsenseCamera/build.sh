#! /bin/sh
echo "Copy Dockerfile"
cp Dockerfile ../ThirdParty/librealsense/scripts/Docker

echo "Build Docker image"
cd ../ThirdParty/librealsense/scripts/Docker
./build_image.sh
echo "Build finished"