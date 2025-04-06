# RBE502-catapult-arm-project

## Environment Setup

Docker is used to setup the development environment.

- Make sure you have an Ubuntu OS available that satisfy one of the requirements
  at https://docs.docker.com/engine/install/ubuntu/.
- Install docker by following the instructions title `Uninstall old versions`
  and `Install using the apt repository` at
  https://docs.docker.com/engine/install/ubuntu/.
- Create a ROS2 workspace.:
``` shell
mkdir -p ~/ros2_ws/src/
```
- If you want to use a different workspace directory, then create it and then set the `ROS2_WS` environment variable in your bash startup script:

``` shell
echo "export ROS2_WS=<path to workspace src folder>" >> ~/.bashrc
```
- Clone the github repository:

``` shell
cd ~/ros2_ws/src
git clone git@github.com:Gregory119/RBE502-catapult-arm-project.git
```
- Build the docker image (this will take a few minutes):

``` shell
cd ~/ros2_ws/src/RBE502-catapult-arm-project/.devcontainer
docker compose build sim
```
- Configure your host (your normal OS not inside the container) display server
  to allow docker to access it for displaying GUIs.

``` shell
xhost +local:
```
- Run a docker container for the image:

``` shell
cd ~/ros2_ws/src/RBE502-catapult-arm-project/.devcontainer
docker compose up sim
```
- Access the container by first opening a new shell and then run:

``` shell
docker exec -it rbe-502-sim-1 bash
```
The shell should have the line `root@dev:~/local_dir#`.
- You can see that the container has access to the contents of the ros workspace as follows:

``` shell
root@dev:~/local_dir# ll
total 12
drwxrwxr-x 3 ubuntu ubuntu 4096 Mar 13 14:05 ./
drwx------ 1 root   root   4096 Mar 13 13:51 ../
drwxrwxr-x 4 ubuntu ubuntu 4096 Mar 13 14:10 src/
```
You will run all your future ros2 compile commands from here (inside the docker container).
- Test running Gazebo inside the container:

``` shell
gz sim
```

## Build
In a shell of the container:

``` shell
cd /root/local_dir
rosdep update && rosdep install --ignore-src --from-paths . -y
colcon build --packages-select catapult --symlink-install
source install/local_setup.bash
```

## Run Examples

``` shell
ros2 launch catapult ur_sim_control.launch.py
```

Move robot using test script from  `ur_robot_driver` package (if you've installed that one):
```
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

Example using MoveIt with simulated robot:
```
ros2 launch catapult ur_sim_moveit.launch.py

```

## Creating the URDF File for Moveit Assistant
- Use source code package of `robotiq_description` with `humble` branch. This is
  used by the `catapult_description` package.
- Build `catapult_description` and its dependencies with (this will build `robotiq_description`)

``` shell
colcon build --packages-up-to catapult_description
```
- Go to `catapult_description/urdf/` and generate the urdf file by running 

``` shell
xacro ur_with_gripper.urdf.xacro name:=ur_manipulator > ur_with_gripper.urdf
```
- Visualize the urdf file by first building the `catapult_description` package and then running:

``` shell
ros2 launch catapult_description view_ur.launch.py
```

- Open this file with the moveit assistant

