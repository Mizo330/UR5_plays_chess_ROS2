###Setup base
#set dynamic distro arg name with default value
ARG ROS_DISTRO=jazzy

FROM osrf/ros:$ROS_DISTRO-desktop-full

#this is a small but basic utility, missing from osrf/ros. It is not trivial to know that this is
#missing when an error occurs, so I suggest installing it just to bes sure.
RUN apt-get update && apt-get install -y netbase
#set shell 
SHELL ["/bin/bash", "-c"]
#set colors
ENV BUILDKIT_COLORS=run=green:warning=yellow:error=red:cancel=cyan

#set noninteractive mode
ENV DEBIAN_FRONTEND=noninteractive

#start with root user
USER root

###Create new user
#
#Creating a user inside the container, so we won't work as root.
#Setting all setting all the groups and stuff.
#
###

#expect build-time argument
ARG HOST_USER_GROUP_ARG
#create group appuser with id 1001
#add appuser to the group with the same id as the host user
#this is important, because it allows to mount the host user home directory into the container
#this way, the container will have the same permissions as the host user
#create appuser user with id 1001 with home; bash as shell; and in the appuser group
#change password of appuser to admin so that we can sudo inside the container
#add appuser to sudo, hostgroup and all default groups
#copy default bashrc and add ROS sourcing
RUN groupadd -g 1001 appuser && \
    useradd --create-home --shell /bin/bash -u 1001 -g appuser appuser && \
    echo 'appuser:admin' | chpasswd &&\
    usermod -aG $HOST_USER_GROUP_ARG appuser &&\
    usermod -aG sudo,plugdev,video,adm,cdrom,dip,dialout appuser && \
    cp /etc/skel/.bashrc /home/appuser/  


###Install the project
#
#If you install multiple project, you should follow the same 
#footprint for each:
#- dependencies
#- pre install steps
#- install
#- post install steps
#
###

#basic dependencies for everything
USER root
RUN apt-get update && \
    apt-get install -y\
    netbase\
    git\
    build-essential\    
    wget\
    curl\
    gdb \
    python3-pip\
    python3-apt\
    python3-rosdep\
    clang-tidy


USER appuser
#ros sourcing
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/appuser/.bashrc


#make workspace directory
RUN cd /home/appuser && \
    mkdir -p ros2_ws/src 

#install vscode server and extensions inside the container
#it propably won't work on computers of others because of the specific vscode version
#mostly stolen from here: https://gist.github.com/discrimy/6c21c10995f1914cf72cd8474d4501b2
#its great, because it means it is already installed into the image, so when starting a vscode instance inside the container, it will be already there.
#it will not have to download it.
#more info: https://github.com/microsoft/vscode-remote-release/issues/1718
USER root
RUN apt-get update && \
    apt-get install -y\
    curl \
    jq \
    sshpass

#install gazebo prerequisites
USER root
RUN apt-get update && \
    apt-get install -y\
    curl\
    lsb-release\
    gnupg

#install gazebo
USER root
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null &&\
    apt-get update &&\
    apt-get install -y\
    gz-harmonic \
    ros-$ROS_DISTRO-ros-gz

#install moveit2 from source
#prerequisites
USER root
RUN rosdep update &&\
    apt-get update &&\
    apt-get dist-upgrade -y &&\
    apt-get install -y\
    python3-colcon-common-extensions\
    python3-colcon-mixin \
    python3-vcstool &&\
    colcon mixin update default

#download moveit2
USER root
RUN cd /home/appuser/ros2_ws/src &&\
    # If using for a specific version, use the tag instead of main, as for some reason jazzy is the main branch, and not the jazzy branch
    git clone -b main https://github.com/moveit/moveit2_tutorials &&\
    vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos &&\
    apt-get update &&\
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

#build moveit2, this will take a while  
USER appuser
RUN cd /home/appuser/ros2_ws && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    MAKEFLAGS=-j4 colcon build --executor sequential --symlink-install

#Install stuff for the ur diver
USER root
RUN apt-get update && \
    apt-get install -y\
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-srdfdom

#Install UR driver from source, using the gripper branch
#Prerequisites
USER root
RUN apt-get update &&\
    apt-get install -y\
    ros-$ROS_DISTRO-ur

#Install ur gazebo simulation
USER root
RUN cd /home/appuser/ros2_ws &&\
    git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz &&\
    rosdep update && rosdep install --ignore-src --from-paths src -y

COPY  --chown=appuser:appuser ./misc/.devcontainer/ /home/appuser/ros2_ws/.devcontainer/

USER appuser
ARG VSCODE_COMMIT_HASH
RUN bash /home/appuser/ros2_ws/.devcontainer/preinstall_vscode.sh $VSCODE_COMMIT_HASH /home/appuser/ros2_ws/.devcontainer/devcontainer.json

USER appuser
COPY --chown=appuser:appuser ./my_packages /home/appuser/ros2_ws/src/my_packages

#build the workspace
USER appuser
RUN cd /home/appuser/ros2_ws && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install 


RUN mkdir -p /home/appuser/ros2_ws/src/my_packages && \
    mkdir -p /home/appuser/ros2_ws/.vscode

#make it that the setup.bash is sourced when a new terminal is opened
USER appuser
RUN echo "source /home/appuser/ros2_ws/install/setup.bash" >> /home/appuser/.bashrc

