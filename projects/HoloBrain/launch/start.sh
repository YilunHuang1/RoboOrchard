#!/bin/bash

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

TMUXP_YAML="$SCRIPT_DIR/launch.yaml"
DOCKER_CONTAINER_NAME="holobrain-copy"


echo ">>> Checking container [$DOCKER_CONTAINER_NAME] state..."

if [ ! "$(docker ps -q -f name=^/${DOCKER_CONTAINER_NAME}$)" ]; then
    echo "⚠️  Container is not running, try to launch..."

    if [ "$(docker ps -aq -f name=^/${DOCKER_CONTAINER_NAME}$)" ]; then
        docker start "$DOCKER_CONTAINER_NAME"
    else
        bash "$SCRIPT_DIR/docker.sh" "$DOCKER_CONTAINER_NAME"
    fi

    sleep 2

    if [ ! "$(docker ps -q -f name=^/${DOCKER_CONTAINER_NAME}$)" ]; then
        echo -e "\033[31m❌ [FATAL] Cannot launch container. Please check 'docker logs $DOCKER_CONTAINER_NAME' \033[0m"
        exit 1
    fi
    echo "✅ Successful launch container！"
else
    echo "✅ Container is running."
fi

echo ">>> Loading workflow..."
tmuxp load "$TMUXP_YAML"
