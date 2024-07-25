#!/bin/bash

# Get the container ID of the running instance
CONTAINER_ID=$(docker ps -q --filter ancestor=adyansh04/crazyflie-ros2:latest)

# Check if the container is running
if [ -z "$CONTAINER_ID" ]; then
    echo "No running container found for the image adyansh04/crazyflie-ros2:latest."
    exit 1
fi

# Open a new bash session in the running container
docker exec -it $CONTAINER_ID /bin/bash
