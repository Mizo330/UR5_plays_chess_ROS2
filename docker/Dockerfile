###Setup base

FROM osrf/ros:jazzy-desktop-full

#this is a small but basic utility, missing from osrf/ros. It is not trivial to know that this is
#missing when an error occurs, so I suggest installing it just to bes sure.
RUN apt-get update && apt-get install -y netbase
#set shell 
SHELL ["/bin/bash", "-c"]
#set colors
ENV BUILDKIT_COLORS=run=green:warning=yellow:error=red:cancel=cyan
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
#create group appuser with id 999
#create group hostgroup with ID from host. This is needed so appuser can manipulate the host files without sudo.
#create appuser user with id 999 with home; bash as shell; and in the appuser group
#change password of appuser to admin so that we can sudo inside the container
#add appuser to sudo, hostgroup and all default groups
#copy default bashrc and add ROS sourcing
RUN groupadd -g 899 appuser && \
    groupadd -g 900 hostgroup && \
    useradd --create-home --shell /bin/bash -u 899 -g appuser appuser && \
    echo 'appuser:admin' | chpasswd && \
    usermod -aG sudo,hostgroup,plugdev,video,adm,cdrom,dip,dialout appuser && \
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
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    netbase\
    git\
    build-essential\    
    wget\
    curl\
    gdb \
    python3-pip\
    python3-apt\
    clang-tidy


USER appuser

#ros sourcing
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/appuser/.bashrc


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
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    curl \
    jq \
    sshpass

COPY  --chown=appuser:appuser ./misc/.devcontainer/ /home/appuser/ros2_ws/.devcontainer/

USER appuser
ARG VSCODE_COMMIT_HASH
RUN bash /home/appuser/ros2_ws/.devcontainer/preinstall_vscode.sh $VSCODE_COMMIT_HASH /home/appuser/ros2_ws/.devcontainer/devcontainer.json

COPY --chown=appuser:appuser ./my_packages /home/appuser/ros2_ws/src/my_packages

RUN mkdir -p /home/appuser/ros2_ws/src/my_packages && \
    mkdir -p /home/appuser/ros2_ws/.vscode