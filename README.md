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
source ~/.bashrc
docker compose start
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

