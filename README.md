# Robot systems & cognitive robotics project

### TODOS:

**Project makers**
- Márk **Bancsi**
- Zsombor **Ménesi**
- Xiang **Wang**
- Bercel **Papp**

## Requisites
- Docker
- Nvidia GPU with driver
- VSCode

## Pre-Installation 

>[!IMPORTANT]
>This project uses a docker enviroment to run ROS2 and any dependencies, so installation of Docker is required.

### Docker

Install docker: https://docs.docker.com/engine/install/ubuntu/

And perform the post-install step following the link at the bottom of the page:

```bash
sudo groupadd docker
```

```bash
sudo usermod -aG docker $USER
```
You should log out and log in again to update the user groups!

Use **VSCode** for easier acces. Install these extensions:
- Devcontainer
- Docker
- Remote - SSh (*may not be needed*)

### NVIDIA Container

To fully utilize the graphics card inside the container we need this. Note that some examples will work without it, but most of them benefit from it!

Follow the installation steps here:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker


## Running the code

Once you have installed Docker, simpli run these following scripts:

```bash
cd /docker
./build_docker.sh
```

If the Dockerfile was succesfuly built, start the container with:

```bash
cd /docker
./run_docker.sh
```

>[!TIP]
>If you run into any problems, feel free to ask or just open an issue

Once the container has been created, go to the **Docker** tap in vscode -> right click the running container -> attach VSCode.

Open the ros2_ws folder, that will be the workspace folder, where you can build.
>[!IMPORTANT] Only colcon build in this folder to circumvent any issues!