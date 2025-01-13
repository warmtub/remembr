#!/bin/bash


docker run \
    -it \
    --name nova_carter_demo \
    --ipc host \
    --network host \
    --gpus all \
    --shm-size 14G \
    --device /dev/snd \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/remembr \
    -w /remembr \
    nova_carter_demo:x86
