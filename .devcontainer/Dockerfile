FROM ubuntu:24.04 AS ros2-gazebo

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
RUN apt-get update && apt-get install -y curl lsb-release gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install gz-harmonic -y

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
    && apt install ros-jazzy-desktop -y

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# install colcon
RUN apt install -y python3-colcon-common-extensions

# setup auto completion
RUN apt-get update && apt-get install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc

# make colcon_cd available
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc \
    && echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /root/.bashrc

# install rosdep
RUN apt install python3-rosdep -y \
    && rosdep init \
    && rosdep update

RUN mkdir root/local_dir
WORKDIR root/local_dir


# might need this later
# useradd ros \
# && apt-get install -y sudo \
# && echo "ros ALL=\(root\) NOPASSWD:ALL" > /etc/sudoers.d/ros \
# && chmod 0440 /etc/sudoers.d/ros
