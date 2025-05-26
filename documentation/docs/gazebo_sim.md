# Gazebo simulation

This project sets up a custom Gazebo simulation environment containing a chess table, a chessboard base, the chessboard itself with all 32 chess pieces placed on the board. The simulation is integrated into a ROS 2 package and includes a Python script to automatically generate the .world file with the desired models and poses.

At this point we would like to acknowledge [Arun](https://github.com/arunkru1998) for his cintribution with the chess models which are available in his [ROS1 chessrobo repository](https://github.com/arunkru1998/chessrobo#).

## About the world

The Gazebo world file (`chess_room.world`) is generated **dynamically** using the `generate_chess_sdf.py` script. The script places all models (chessboard and pieces) based on a flexible parameter setup.

<p align="center">
  <img src="/images/gazebo_world_chess.png" width="800" />
</p>

## Key Parameters

The following parameters control how the models are positioned in the world:

| **Parameter**         | **Description**                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `tile_size`     | The size of one chess square (in meters). Determines spacing of the pieces. |
| `a1` | World coordinates (x, y) where the A1 square of the board is placed.         |
|`orientaion`| Sets the orientation of the board. 

These parameters can be located at the `ur_chess_config.yaml` file modification should be made here.
Moreover, the height of elements are adjustable too (e.g. table or chess board height).

## Generating and launching the world

The generation is quite simple. **After** modifíing the parameters run the generation script.

```bash
ros2 run chess_gazebo_world generate_chess.sdf.py
```

After generating the world the simulation can be launched. Note that with this launch the chess game **will not be active**, only RVizz and Gazebo will be launched with the simulation of controlling the robot in the generated world.

```bash
colcon build
source install/setup.bash
ros2 launch chess_gazebo_world chess_world_with_ur.launch.py
```

## Using the world

If we would like to **run the chess game itself** with modified world generation, we **DO NOT HAVE TO generate the world separately**.  
This is because the `ur_chess.launch.py` file **automatically calls the world generation process**, so launching it will ensure that the world is built with the current configuration.




