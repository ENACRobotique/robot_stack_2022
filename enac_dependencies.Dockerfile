# to use on raspberry pi, without GUI
# WILL add nav2
#add usb_cam_driver

#galactic version from ~ 7 jan 2022
# https://hub.docker.com/layers/ros/library/ros/galactic-ros-base/images/sha256-d52ee1b0d65d7df83a4897c39568d6a450a10d76c330450c90ae0e79e4c0d2a8?context=explore
FROM ros@sha256:d52ee1b0d65d7df83a4897c39568d6a450a10d76c330450c90ae0e79e4c0d2a8
LABEL Name=robotstack2022 Version=0.1

SHELL [ "/bin/bash" , "-c" ]

#install build tools

RUN apt-get update && apt-get install -y git

RUN apt-get install -y python3-colcon-common-extensions

RUN apt-get install -y cmake gcc g++ build-essential

#install python tools

RUN apt-get install -y python3-pip

#install python packages

RUN pip install pyserial

#install visual/image processing tools

RUN apt install -y python3-opencv

RUN apt install -y ros-galactic-cv-bridge

# disabled due to taking ~600 mo
# RUN apt install -y ros-galactic-navigation2

#install camera tools

RUN apt-get install -y ros-galactic-v4l2-camera

#TODO : more proper camera_calibration, or check if package has been fixed https://github.com/ros-perception/image_pipeline/issues/713
#adding build tools but removing them just after
#fix from 22 jan 2022

RUN sudo apt-get install -y cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev &&\
    source /opt/ros/galactic/setup.bash &&\
    mkdir /driver_ws &&\
    cd driver_ws &&\
    mkdir src &&\
    cd src &&\ 
    #fix 2 
    git clone -b ros2 https://github.com/ros-perception/image_pipeline &&\
    cd image_pipeline &&\
    git checkout 6257bda1449a6a9ffb918f05f0b92ffbfee6a623 &&\
    cd .. &&\
    # git clone -b ros2 https://github.com/jonathanTIE/image_pipeline &&\
    git clone -b ros2 https://github.com/ros-perception/vision_opencv &&\
    cd .. &&\
    colcon build --packages-up-to camera_calibration &&\
    sudo apt-get remove -y libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev &&\
    sudo apt-get autoremove -y &&\
    rm -rf src
    
RUN sudo apt install -y libcanberra-gtk-module libcanberra-gtk3-module
#install gui elements - add ~200MB

RUN apt install -y ros-galactic-rviz2 ros-galactic-rviz-default-plugins

RUN apt install -y ros-galactic-rqt ros-galactic-rqt-common-plugins


RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "source /driver_ws/install/local_setup.bash" >> ~/.bashrc



