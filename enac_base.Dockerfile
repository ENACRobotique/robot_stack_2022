FROM enac_robotique/enac_ros:latest

SHELL [ "/bin/bash" , "-c" ]

#TODO : look for alternative lightweight of opencv

RUN apt install -y python3-opencv

RUN apt install -y ros-galactic-cv-bridge

#TODO : conditionnal gui elements - add ~200MB

RUN apt install -y ros-galactic-rviz2 ros-galactic-rviz-default-plugins

RUN apt install -y ros-galactic-rqt ros-galactic-rqt-common-plugins

RUN mkdir /enac_ws &&\
    cd /enac_ws &&\ 
    mkdir src &&\
    mkdir bag &&\
    cd src

COPY ../. /enac_ws/src

VOLUME [ "/enac_ws/src" , "/enac_ws/bag"]

RUN cd /enac_ws &&\
    source /opt/ros/galactic/setup.bash &&\
    colcon build --symlink-install &&\
    source install/local_setup.bash

RUN echo "source /enac_ws/install/local_setup.bash" >> /root/.bashrc