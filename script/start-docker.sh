#!/bin/bash

set -e

# Docker image to pull
image="ruansp/ubuntu-20.04:cfc"

if [[ "$(docker images -q "$dockerImage" 2> /dev/null)" == "" ]]; then
    docker pull "$image"
fi

# Name of the docker container
name="cfc"

# Workspace to mount to docker container
workspace="$(pwd)/../"

docker run \
    -it \
    --rm \
    --ipc host \
    --name $name \
    -v "$workspace":/home/cfc/ \
    "$image" \
    /bin/bash
