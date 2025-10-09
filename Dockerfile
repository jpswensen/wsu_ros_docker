# This Dockerfile builds an Ubuntu MATE desktop environment with ROS2 Humble, Gazebo, Turtlebot3, VSCode, and MATLAB R2025b with ROS Toolbox.
# It is based on the linuxserver.io baseimage-kasmvnc:ubuntujammy image.
# The image also includes an OpenSSH server to allow SSH access to the 'me485' user with password authentication
FROM ghcr.io/linuxserver/baseimage-kasmvnc:ubuntujammy

# set version label
ARG BUILD_DATE
ARG VERSION
LABEL build_version="Linuxserver.io version:- ${VERSION} Build-date:- ${BUILD_DATE}"
LABEL maintainer="thelamer"

# title
ENV TITLE="Ubuntu MATE"

# prevent Ubuntu's firefox stub from being installed
COPY /root/etc/apt/preferences.d/firefox-no-snap /etc/apt/preferences.d/firefox-no-snap

RUN \
  echo "**** add icon ****" && \
  curl -o \
    /kclient/public/icon.png \
    https://raw.githubusercontent.com/linuxserver/docker-templates/master/linuxserver.io/img/webtop-logo.png && \
  echo "**** install packages ****" && \
  add-apt-repository -y ppa:mozillateam/ppa && \
  apt-get update && \
  DEBIAN_FRONTEND=noninteractive \
  apt-get install --no-install-recommends -y \
    ayatana-indicator-application \
    firefox \
    mate-applets \
    mate-applet-brisk-menu \
    mate-terminal \
    pluma \
    ubuntu-mate-artwork \
    ubuntu-mate-default-settings \
    ubuntu-mate-desktop \
    ubuntu-mate-icon-themes && \
  echo "**** mate tweaks ****" && \
  rm -f \
    /etc/xdg/autostart/mate-power-manager.desktop \
    /etc/xdg/autostart/mate-screensaver.desktop && \
  echo "**** cleanup ****" && \
  apt-get autoclean && \
  rm -rf \
    /config/.cache \
    /config/.launchpadlib \
    /var/lib/apt/lists/* \
    /var/tmp/* \
    /tmp/*

RUN apt-get update && apt-get upgrade -y
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe -y
RUN sudo apt update && sudo apt install curl wget ca-certificates -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null



# Create an ubuntu user
RUN useradd -m -s /bin/bash me485 && \
    mkdir -p /home/me485 && \
    chown -R me485:me485 /home/me485
RUN usermod -aG sudo me485
RUN echo "me485:me485" | chpasswd

# Switch to the 'ubuntu' user for appending to .bashrc
USER me485
WORKDIR /home/me485

# Install ROS2, Gazebo, and Turtlebot3 packages
RUN sudo apt update
RUN sudo apt upgrade -y
RUN sudo apt install ros-humble-desktop -y
RUN sudo apt install ros-humble-turtlebot3* -y
RUN sudo apt install ros-humble-gazebo* -y
RUN sudo apt install python3-colcon-common-extensions python3-argcomplete -y
RUN sudo apt install gedit nano vim -y

# Setup ROS2 terminal usage
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> /home/me485/.bashrc
# RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /home/me485/.bashrc
# RUN echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_profile.xml' >> /home/me485/.bashrc
RUN echo 'source /usr/share/gazebo/setup.bash' >> /home/me485/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/me485/.bashrc
RUN echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /home/me485/.bashrc
RUN echo 'if [ -z "$GAZEBO_MODEL_PATH" ]; then         export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/;     else         export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/;     fi' >> /home/me485/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=waffle' >> /home/me485/.bashrc

# Install VSCode
RUN curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
RUN sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/keyrings/microsoft-archive-keyring.gpg
RUN sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/microsoft-archive-keyring.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
RUN sudo apt update -y
RUN sudo apt install code -y
RUN sudo apt install python3.10-venv -y

# Get the interface set up, with working shortcuts for terminal sizing
RUN gsettings set org.mate.interface gtk-theme "Yaru-MATE-dark"
RUN gsettings set org.mate.terminal.keybindings zoom-in '<Primary>equal'
RUN gsettings set org.mate.terminal.keybindings zoom-out '<Primary>minus'
RUN gsettings set org.mate.terminal.keybindings zoom-normal '<Primary>0'

# Install MATLAB R2025b and ROS Toolbox
RUN wget -q https://www.mathworks.com/mpm/glnxa64/mpm
RUN chmod +x mpm
RUN sudo HOME=/home/me485 ./mpm install --release=R2025b --destination=/opt/matlab/R2025b --products="MATLAB"
RUN sudo HOME=/home/me485 ./mpm install --release=R2025b --destination=/opt/matlab/R2025b --products="ROS Toolbox"
RUN sudo rm -rf mpm /tmp/mathworks_root.log
RUN sudo ln -s /opt/matlab/R2025b/bin/matlab /usr/local/bin/matlab

USER root

# Install and configure OpenSSH server so the me485 user can login with password
RUN apt-get update && \
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends openssh-server && \
  mkdir -p /var/run/sshd && \
  sed -i 's/^#\?PasswordAuthentication.*/PasswordAuthentication yes/' /etc/ssh/sshd_config || true && \
  sed -i 's/^#\?PermitRootLogin.*/PermitRootLogin no/' /etc/ssh/sshd_config || true && \
  sed -i 's/^#\?UsePAM.*/UsePAM yes/' /etc/ssh/sshd_config || true && \
  apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
RUN usermod --password $(openssl passwd -6 me485) me485

# Create s6 service for sshd so s6-overlay will manage the sshd long-running process
RUN mkdir -p /etc/s6-overlay/s6-rc.d/svc-sshd /etc/s6-overlay/s6-rc.d/user/contents.d && \
  echo 'longrun' > /etc/s6-overlay/s6-rc.d/svc-sshd/type && \
  printf '%s\n' '#!/bin/sh' 'exec /usr/sbin/sshd -D' > /etc/s6-overlay/s6-rc.d/svc-sshd/run && \
  chmod +x /etc/s6-overlay/s6-rc.d/svc-sshd/run && \
  touch /etc/s6-overlay/s6-rc.d/user/contents.d/svc-sshd

# link home to /config/home so user data is persistent
RUN sed -i '/^HOME=/d' /etc/environment \
 && mkdir -p /home/me485 \
 && chown -R me485:me485 /home/me485 \
 && ln -s /config /home/me485/config


# add local files
COPY /root /

# ports and volumes
EXPOSE 3000 22
VOLUME /config
