#!/bin/bash
    # --rm \


docker run \
    -it \
    --name nova_carter_demo \
    --device /dev/snd \
    --network host \
    --runtime nvidia \
    -v $(pwd):/remembr \
    -w /remembr/examples/nova_carter_demo \
    nova_carter_demo:l4t
