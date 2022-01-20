# to use on raspberry pi, without GUI
# WILL add nav2
#add usb_cam_driver

#galactic version from ~ 7 jan 2022
# https://hub.docker.com/layers/ros/library/ros/galactic-ros-core/images/sha256-bf74caf34c33125b891d1c600725006b4fdb31d3ec0861601699d498f39bbcdc?context=explore
FROM ros@sha256:ed03b07f7560de940d550373f5e8ec036e46265ffa43e8629579c2d38c99d240
LABEL Name=robotstack2022 Version=0.1

# RUN apt-get -y update && apt-get install -y fortunes
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y git

RUN apt-get install -y python3-colcon-common-extensions

RUN mkdir /driver_ws &&\
    cd driver_ws &&\
    mkdir src &&\
    cd src &&\
    git clone -b foxy-devel https://github.com/klintan/ros2_usb_camera 
    # &&\
    # cd .. &&\
    # source /opt/ros/galactic/setup.bash &&\
    # colcon build &&\
    # source install/local_setup.bash &&\
    # cd

RUN echo "source /opt/ros/galactic/setup.bash >> ~/.bashrc"




# apt-get update && apt-get install --no-install-recommends -y \
#    ^ython open cv

