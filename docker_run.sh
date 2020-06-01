#!/bin/bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

BASH_OPTION=bash
if [ ! -z "$1" ]; then
    BASH_OPTION=$@
fi
echo "running $BASH_OPTION in docker"

docker run -it --rm \
        --name tracking \
        --network host \
        --privileged \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v "$XAUTH:$XAUTH" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix" \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /dev:/dev \
	-v /data/datasets/:/data \
        -v "${PWD}:/workspace" \
        -w "/workspace" \
        biomotion/bnn-tracking:latest $BASH_OPTION
