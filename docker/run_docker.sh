#!/bin/bash
#parameters
source .params

#add all the local processes to xhost, so the container reaches the window manager
xhost + local:

#check if correct directory paths are provided
if [ -z "$src_folder" ] || [ -z "$vscode_folder" ];
then
    echo "Please provide all directory paths in the .params file."
    exit
fi
if  ! [ -d "$src_folder" ] || ! [ -d "$vscode_folder" ];
then
    echo "Please make sure you provide directories that exist."
    exit
fi


#if the git repository exists and it is not shared promt the user to make it shared
if [[ -e ".git" ]];
then
    if [[ $( git config core.sharedRepository ) != "true" ]];
    then
        echo "The repository is not shared (git config core.sharedRepository)."
        echo "If you will use git at any point you should make it shared because if not the file modifications from inside the container will break git."
        echo "Would you like to make the repo shared?"
        select yn in "Yes" "No"; do
            case $yn in
                Yes )
                    echo "Changing repo to shared.";
                    git config core.sharedRepository true;
                    break;;
                No )
                    echo "Leaving it untouched.";
                    break;;
            esac
        done
    fi
fi

#Set permissions for newly created files. All new files will have the following permissions.
#Important is that the group permissions of the files created are set to read and write and execute.
#We add src as a volume, so we will be able to edit and delete the files created in the container.
setfacl -PRdm u::rwx,g::rwx,o::r ./

#check if container exists
if [[ $( docker ps -a -f name=$container_name | wc -l ) -eq 2 ]];
then
    echo "Container already exists. Do you want to restart it or remove it?"
    select yn in "Restart" "Remove"; do
        case $yn in
            Restart )
                echo "Restarting it... If it was started without USB, it will be restarted without USB.";
                docker restart $container_name;
                break;;
            Remove )
                echo "Stopping it and deleting it... You should simply run this script again to start it.";
                docker stop $container_name;
                docker rm $container_name;
                break;;
        esac
    done
else
    echo "Container does not exist. Creating it."
    #NVIDIA_VISIBLE_DEVICES and NVIDIA_DRIVER_CAPABILITIES sets the visible devices and capabilities of the GPU
    #gpus all adds all the gpus to the container
    #runtime=nvidia tells the docker engine to use the nvidia runtime
    #sometimes GUI programs such as rviz are running faster (OpenGL on NVIDIA) with these params, but you don't need them here
    docker run \
        --env DISPLAY=${DISPLAY} \
        --env NVIDIA_VISIBLE_DEVICES=all \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --volume /tmp/.X11-unix:/tmp/.X11-unix \
        --volume ${src_folder}:/home/appuser/ros2_ws/src/my_packages \
        --volume ${vscode_folder}:/home/appuser/ros2_ws/.vscode \
        --network host \
        --interactive \
        --tty \
        --detach \
        --gpus all \
        --runtime=nvidia \
        --privileged \
        --name $container_name \
        $image_name:$image_tag 
fi
