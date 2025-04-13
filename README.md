# UR5E robot plays chess - ROS2 adaptation

### TODOS:
- [ ] Make a world with chess scene
- [ ] Make our own simple controller
- [ ] Make estop?
- [ ] Find out the manipulator type and make a controller for it in ros2
  - [ ] Get real and simulated camera feed as well
- [ ] Find a suitable image det. model for chess figures
- [ ] Learn about Stockfish open-source chess engine
- [ ] Test driver on real life robot

**Project makers**
- Márk **Bancsi**
- Zsombor **Ménes**
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


## Running the container

Once you have installed Docker, simply run these following scripts:

```bash
cd /docker
./build_docker.sh
```

If the Dockerfile was succesfuly built, start the container with:

```bash
cd /docker
./run_docker.sh
```

If you don't have nvidai GPU on your system, remove the lines --gpus=all and --runtime nvidia from the run script, but bevare the simulation software will chug your pc.

Sometimes Docker won't see the nvidia runtime, so we need to edit the docker daemon manually.
```bash
sudo nano /etc/docker/daemon.json
```
And include the following:
```bash
{
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
}
```

>[!TIP]
>If you run into any other problems, feel free to ask or just open an issue

Once the container has been created, go to the **Docker** tap in vscode -> right click the running container -> attach VSCode.

Open the ros2_ws folder, that will be the workspace folder, where you can build.
>[!IMPORTANT] 
> Only colcon build in this folder to circumvent any issues!

## Running the UR driver
Once you are inside the container, you can start running drivers and writing you code.

The full documentation about the UR driver and its packages can be found [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html).

### Simulation

To run the gazebo sim: 
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
```
Launch a simple test to see if everything works.
```bash
ros2 run ur_robot_driver example_move.py
```
This was copied from the documentation, for more in-depth info visit the documentation's [simulation section](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_simulation_gz/ur_simulation_gz/doc/usage.html)

### Real-life
- TBD