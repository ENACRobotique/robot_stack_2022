FROM enacrobotique/enac-ros:latest

SHELL [ "/bin/bash" , "-c" ]

RUN cd /

RUN mkdir /enac_ws &&\
    cd /enac_ws &&\ 
    mkdir src &&\
    mkdir bag &&\
    cd src

COPY . /enac_ws/src

RUN pip install -r /enac_ws/src/requirements.txt

RUN cd /enac_ws &&\
    source /opt/ros/galactic/setup.bash &&\
    colcon build --symlink-install &&\
    source install/local_setup.bash

VOLUME [ "/enac_ws/src" , "/enac_ws/bag"]

RUN echo "source /enac_ws/install/local_setup.bash" >> /root/.bashrc