#!/bin/bash

ARGS="--env DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  --env XAUTHORITY=$XAUTHORITY \
  --env DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all --privileged \
  -it \
  --rm \
  --env DDS_DOMAIN_ID=$DDS_DOMAIN_ID \
  --network=host \
  --cap-add=SYS_PTRACE \
  --ipc=host\
  --security-opt seccomp=unconfined \
  -v /home/msardonini/git/flyStereo:/root/flyStereo "

xhost +local:root

docker run $ARGS flystereo:0.0.1

xhost -local:root
