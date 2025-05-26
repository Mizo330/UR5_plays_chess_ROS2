# UR5 Chess ROS2 - Documentation

This is a full documentaion for our universisty project, that was about converting a [ROS1 project of the same name](https://github.com/MOGI-ROS/ROS_Chess) to ROS2. 
During the course of development, we realized that due to the massive changes in every code since then, an easy conversion was not on the table. Most of the code was unusable for ROS2, so we decided to start from scratch.

## Project environment

We decided to use **Docker** which helped us avoid issues with compatibility, exports and helped us trim the project by not including third-party packages, drivers directly in the repo. For ease of handling, we have made scipts to run and build the container.

## Third party packages

Before talking about our own packages, we must say our acknowlegment to [***DanielBrenn***](https://github.com/DanielBrenn) for making the necessary changes for the UR drivers to handle the [RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A) gripper in gazebo and MoveIt. And also for general help in the project.

## Poject sturcture 
![projectgraph](images/ur_chess_graph.png)
