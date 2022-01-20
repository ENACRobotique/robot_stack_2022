# to use on raspberry pi, without GUI
# WILL add nav2
#add usb_cam_driver

#galactic version from ~ 7 jan 2022
# https://hub.docker.com/layers/ros/library/ros/galactic-ros-base/images/sha256-d52ee1b0d65d7df83a4897c39568d6a450a10d76c330450c90ae0e79e4c0d2a8?context=explore
FROM ros@sha256:d52ee1b0d65d7df83a4897c39568d6a450a10d76c330450c90ae0e79e4c0d2a8
LABEL Name=robotstack2022 Version=0.1

# RUN apt-get -y update && apt-get install -y fortunes
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y git

RUN apt-get install -y python3-colcon-common-extensions

RUN apt-get install -y cmake gcc g++ build-essential

RUN apt-get install ros-galactic-v4l2-camera

# RUN mkdir /driver_ws &&\
#     cd driver_ws &&\
#     mkdir src &&\
#     cd src &&\
#     git clone -b foxy-devel https://github.com/klintan/ros2_usb_camera &&\
#     cd .. &&\
#     source /opt/ros/galactic/setup.bash 
#     #&&\
# rosdep install --from-paths ./src --ignore-src &&\
# colcon build &&\
# source install/local_setup.bash &&\
# cd

RUN echo "source /opt/ros/galactic/setup.bash >> ~/.bashrc"




# apt-get update && apt-get install --no-install-recommends -y \
#    ^ython open cv

