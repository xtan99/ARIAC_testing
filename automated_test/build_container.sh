#!/bin/bash

teamName=ariac_test

#enable local connections to docker
xhost +local:docker

if [[ "$1" == "nvidia" ]] ; then
    # Build the docker image
    docker run -t -d --name $teamName -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1000  --gpus=all --runtime=nvidia -e "NVIDIA_DRIVER_CAPABILITIES=all" --network=host --pid=host --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw nistariac/ariac2024:latest
else
    # Build the docker image
    docker run -t -d --name $teamName -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1000  --network=host --pid=host --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw  nistariac/ariac2024:latest
fi

# Copy scripts directory and yaml file
docker cp ./container_scripts/ $teamName:/
docker cp ./competitor_configs/competitor_build_scripts/ $teamName:/
docker cp ./trials/ $teamName:/
docker cp ./ariac_test $teamName:/workspace/src
# docker cp ./competitor_configs/$1.yaml $teamName:/container_scripts

# Run build script
docker exec -it $teamName bash -c ". /container_scripts/build_environment.sh ariac_test"