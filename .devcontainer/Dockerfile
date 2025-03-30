FROM ubuntu:22.04 AS ros2-gazebo

######################################################################
# Gazebo
######################################################################

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata

# Install gazebo
RUN curl -sSL http://get.gazebosim.org | sh

ENV LIBGL_ALWAYS_SOFTWARE=1

######################################################################
# ROS2
######################################################################

# Enable required repositories
RUN apt install software-properties-common -y \
    && add-apt-repository universe \
    && apt update && apt install curl -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ros 2
RUN apt update \
    && apt upgrade -y \
    && apt install ros-humble-desktop -y

# add user with sudo password
RUN useradd -m ros && echo "ros:ros" | chpasswd && adduser ros sudo
USER ros
ENV HOME /home/ros
RUN echo "source /opt/ros/humble/setup.bash" >> ${HOME}/.bashrc
USER root

# install colcon
RUN apt install -y python3-colcon-common-extensions

# setup auto completion
USER root
RUN apt-get update && apt-get install -y git-core bash-completion
USER ros
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> ${HOME}/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> ${HOME}/.bashrc

# make colcon_cd available
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ${HOME}/.bashrc \
    && echo "export _colcon_cd_root=/opt/ros/humble/" >> ${HOME}/.bashrc

# install rosdep
USER root
RUN apt install python3-rosdep -y\
    && rosdep init
USER ros
RUN rosdep update

# install particular ros2 packages
USER root
RUN apt install ros-humble-ur-robot-driver -y

######################################################################
# Install Moveit
######################################################################
# install mixin to complement colcon for building packages
RUN apt install python3-colcon-mixin \
    && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default

# install vcstool
RUN apt install python3-vcstool

USER ros
# RUN mkdir -p ${HOME}/tmp_ws/src \
#     && cd ${HOME}/tmp_ws/src \
#     && git clone -b humble https://github.com/moveit/moveit2_tutorials \
#     && vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
# USER root
# RUN apt remove ros-humble-moveit* \
#     && apt update && rosdep install -r --from-paths ${HOME}/tmp_ws/src --ignore-src --rosdistro humble -y
# RUN rm -r ${HOME}/tmp_ws/src

RUN mkdir -p ${HOME}/tmp_ws/src
WORKDIR ${HOME}/tmp_ws/src
RUN git clone -b humble https://github.com/moveit/moveit2_tutorials \
    && vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
RUN git clone https://github.com/Gregory119/RBE502-catapult-arm-project.git
#    && vcs import --recursive < RBE502-catapult-arm-project/Universal_Robots_ROS2_Gazebo_Simulation.humble.repos
USER root
RUN apt remove ros-humble-moveit* \
    && apt update && rosdep install -r --from-paths ${HOME}/tmp_ws/src --ignore-src --rosdistro humble -y
WORKDIR ${HOME}
RUN rm -r ${HOME}/tmp_ws/src


######################################################################
# create directory to access host files
######################################################################
USER ros
RUN mkdir ${HOME}/local_dir
WORKDIR ${HOME}/local_dir
