FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update && \
    apt-get install -y --no-install-recommends \
    wget \
    curl \
    git \
    vim \
    python3-pip \
    lsb-release \
    gnupg

# ROS setting
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update && \
    apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-catkin-tools
RUN rosdep init && rosdep update

RUN apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update && \
    apt-get install -y --no-install-recommends \
    ros-noetic-cv-bridge

# Python3 setting
# RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
# RUN python3 get-pip.py
RUN pip3 install rospkg catkin_pkg

WORKDIR /code
ADD requirements.txt /code
RUN pip3 install -r requirements.txt

# Setup the Bash shell environment with '/root/.bashrc'.
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc
