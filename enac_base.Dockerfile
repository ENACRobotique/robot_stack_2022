FROM enacrobotique/enac-ros:latest

ARG DEV

SHELL [ "/bin/bash" , "-c" ]

RUN cd /

#TODO : TO MOVE TO ENAC_DEPENDENCIES
RUN apt-get install -y libudev-dev

RUN mkdir /enac_ws &&\
    cd /enac_ws &&\ 
    mkdir src &&\
    mkdir bag &&\
    cd src

COPY . /enac_ws/src

RUN pip install -r /enac_ws/src/requirements.txt

RUN cd /enac_ws &&\
    source /opt/ros/galactic/setup.bash &&\
    #conditionnal symlink if target don't have src (because not dev env)
    #Don't Correct this condition below with elif if you don't fix the CI pipeline build-arg for prod env
    if [ "$DEV" = "True" ]; then \
    colcon build --symlink-install; \
    else \
    colcon build; \
    fi &&\
    source install/local_setup.bash

VOLUME [ "/enac_ws/src" , "/enac_ws/bag"]

RUN echo "source /enac_ws/install/local_setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "ros2", "run", "ros2serial", "ros2serial"]
