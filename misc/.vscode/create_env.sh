#!/bin/bash
cd /home/appuser/ros2_ws
source install/setup.bash
rm .vscode/envfile.env
touch .vscode/envfile.env
printenv > .vscode/envfile.env