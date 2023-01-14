FROM dorowu/ubuntu-desktop-lxde-vnc:xenial
ENV DEBIAN_FRONTEND noninteractive

### Core Linux tools
RUN apt-get update && apt-get install -y --no-install-recommends\
  apt-utils lsb-release sudo unzip wget ssh vim curl tmux\
  software-properties-common gnupg2

### Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update && apt install -y --no-install-recommends ros-kinetic-desktop-full

### ROS tools
RUN apt-get update && apt-get install -y --no-install-recommends\
    python-dev python-dev git python-pip\
    python-catkin-tools python-rosdep python-rosinstall python-rosinstall-generator python-wstool\
    build-essential
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### User and permissions
ARG user=ir  # Intelligent Robots!
ARG group=ir
ARG uid=1000
ARG gid=1000
ARG home=/home/${user}
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${gid} ${group} \
    && useradd -d ${home} -u ${uid} -g ${gid} -m -s /bin/bash ${user} \
    && echo "${user} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sudoers_${user}  # passwordless sudo
USER ${user}
RUN sudo usermod -a -G video ${user}
WORKDIR ${home}
ENV HOME ${home}
ENV USER ${user}

### Setup ROS environment
RUN sudo rosdep init && rosdep update
RUN printf '\nsource /opt/ros/kinetic/setup.bash' >> ${home}/.bashrc

### Lab 1 dependency
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends\
    ros-kinetic-turtlebot-stage ros-kinetic-joint-state-publisher-gui\
    ros-kinetic-turtlesim

### Docker image entrypoint
RUN sudo sed -i -e\
    's/chown -R $USER:$USER ${HOME}/chown -R $USER:$USER {.config,.gtkrc-2.0,.asoundrc}/g'\
    /startup.sh
COPY entrypoint.bash /usr/local/bin/entrypoint
COPY start-novnc.bash /usr/local/bin/start-novnc
ENTRYPOINT ["entrypoint"]
CMD ["start-novnc"]
