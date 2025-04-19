#!/bin/bash
PARAMS_FILE=".params"
EXAMPLE_FILE=".params.example"

# If the .params file doesn't exist, create one from the example and prompt the user
if ! [ -f "$PARAMS_FILE" ];
then
   cp "$EXAMPLE_FILE" "$PARAMS_FILE"
   echo -e "\e[1;37;41mChange parameters to suit your project!\e[0m"
   #TODO prompt user to edit the .params file
fi

# Load parameters from the .params file
source "$PARAMS_FILE"

# Get the host's group ID and VSCode commit hash
HOST_USER_GROUP_ARG=$(id -g $USER)
VSCODE_COMMIT_HASH=$(code --version | sed -n '2p')
echo $VSCODE_COMMIT_HASH

docker build \
    --file Dockerfile \
    --tag $image_name:$image_tag \
    --build-arg HOST_USER_GROUP_ARG=$HOST_USER_GROUP_ARG \
    --build-arg VSCODE_COMMIT_HASH=$VSCODE_COMMIT_HASH \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    ./..\
