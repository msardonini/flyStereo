#!/bin/bash

xhost +local:root

docker run \
  --env DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/storage/fly_stereo/:/mnt/storage/fly_stereo:ro \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rmw' \
  --device /dev/dri \
  --env XAUTHORITY=$XAUTHORITY \
  --env DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  -it \
  --rm \
  --env DDS_DOMAIN_ID=$DDS_DOMAIN_ID \
  --network=host \
  --cap-add=SYS_PTRACE \
  --cap-add=SYS_ADMIN \
  --name=flystereo \
  --ipc=host\
  --security-opt seccomp=unconfined \
  --gpus all \
  -v /home/msardonini/git/flyStereo:/root/flyStereo \
  flystereo:0.0.1




xhost -local:root
