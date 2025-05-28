<p align="center">
  <h1 align="center">UR5E robot plays chess - ROS2</h1>
  <p align="center">
    <img src="https://img.shields.io/badge/python-3.12-blue"/>
    <img src="https://img.shields.io/badge/ROS2-%20Jazzy-blue?logo=ros&logoColor=white" />
    <a href="https://mizo330.github.io/UR5_plays_chess_ROS2/">
      <img src="https://img.shields.io/badge/%20Pages-Documentation-181717?logo=github&logoColor=white&labelColor=gray&color=blue"/>
    </a>
    <a href="https://youtu.be/9IK0mxdCMDk">
       <img src="https://img.shields.io/badge/YouTube-Demo%20Video-red?logo=youtube&style=flat" />
    </a>
    <a href="LICENSE">
    <img src="https://img.shields.io/badge/license-MIT-purple"/>
    </a>
    <img src="/media/demo.gif" width="800" />
  </p>
</p>


## Documentation

The documentation about the workings of this project and detailed descpition about its usage can be found in the **[project doc page](https://mizo330.github.io/UR5_plays_chess_ROS2/)**.

**<u>You can watch a full demo match, uploaded on [Youtube](https://youtu.be/9IK0mxdCMDk).</u>**


### TODOS:
- [ ] Refine code
  - [ ] Fix game controls (~~play~~, pause, stop)
  - [ ] Better error handling (~90%done)
  - [ ] Config for constraints, maybe dynamic based on borad loc.
- [x] Update README
- [x] Update doc page
- [ ] **Test driver on real life robot**
  - [ ] Make grabber control interface for ROS2

**People on the project**
- Márk **Bancsi**
- Zsombor **Ménes**
- Xiang **Wang**
- Bercel **Papp**

## Pre-requisites
- Ubuntu 24 (native or WSL)
- Nvidia GPU with driver ([how to](https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/index.html))
- Docker
- (VSCode, optional)

## Pre-Installation 

>[!IMPORTANT]
>This project uses a docker enviroment to run ROS2 and any dependencies, so installation of Docker is strongly suggested.
> *You can fish out the necessary pacakges for it to run from the Dockerfile of course...*

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

### NVIDIA Container

To fully utilize the graphics card inside the container we need this. Note that some examples will work without it, but most of them benefit from it!

Follow the installation steps here:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker


## Installation - Docker

We have provided a Dockerfile to test and run the simulation. 

Build the container. This will take some time.
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

### Running the simulation:

```
ros2 launch ur_chess ur_chess.launch.py mode:=FishVFish
```

This will launch the full simulation, where the robot is using stockfish to place the pieces. To start the game press start on the UI.

#### Game modes

- `'FishVFish'` - stockfish will play againts itself
-  `'PVP'` - player versus player, you can place the pieces on the GUI
- `'P(b)VFish'` - player versus stockfish, player on black side
- `'P(w)VFish'` - player versus stockfish, player on white side

#### Parameters
You can tweak some parameters in the `/ur_chess/config/ur_chess_config.yaml` file, like the stockfish settings and chessboard location.

>[!NOTE]
>The board parameters arent foolproof yet, so you can easily make pieces unreachable for the robot either by setting it too far, or placing the chessboard inside the robot.
>To further edit how the world looks, you can edit the `/chess_gazebo_world/chess_gazebo_world/generate_chess_sdf.py`, but not everything has been synchronized with moveit collision creation, only the params in the yaml.

### Real-life
- TBD
