#!/bin/bash

if [ "$SENSOR" = "RGBD" ]; then
    echo "Processing RGBD sensor data"
    DOCKER_COMPOSE_FILE="example/docker-compose-rgbd.yaml"
elif [ "$SENSOR" = "LIDAR" ]; then
    echo "Processing LIDAR sensor data"
    DOCKER_COMPOSE_FILE="example/docker-compose-lidar.yaml"
elif [ "$SENSOR" = "CUSTOM" ]; then
    echo "Processing CUSTOM tracker"
    DOCKER_COMPOSE_FILE="example/docker-compose-custom.yaml"
else
    echo "Unknown sensor type: $SENSOR"
    exit 1
fi

# Creating output directory
# mkdir -p $OUTPUT_DIRECTORY

# Read the experiments from the provided file
for EXPERIMENT in "$INPUT_DIRECTORY"/*/
do
        EXPERIMENT=${EXPERIMENT%*/}
        EXPERIMENT=${EXPERIMENT##*/}

        echo processing experiment "$EXPERIMENT"
        echo starting people tracker
        # xhost local:root
        TRACKER_COMPOSE_FILE=$TRACKER_COMPOSE_FILE EXPERIMENT=$EXPERIMENT INPUD_DIRECTORY=$INPUT_DIRECTORY $DOCKER_COMPOSE_CMD -f $DOCKER_COMPOSE_FILE up -d
        docker wait player
        sleep 2

        echo experiment terminated, killing people tracker
        $DOCKER_COMPOSE_CMD -f $DOCKER_COMPOSE_FILE kill

        echo saving output file in the folder with the file for the evaluation
        cp ../tracker-output-tmp/tracker_output.csv ../evaluation-data/$EXPERIMENT/
done
