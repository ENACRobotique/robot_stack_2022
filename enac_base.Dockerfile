FROM enac_ros

SHELL [ "/bin/bash" , "-c" ]

RUN mkdir /enac_ws &&\
    cd /enac_ws &&\ 
    mkdir src &&\
    cd src

COPY ../. /enac_ws
# VOLUME ["/enac_ws"]

RUN cd /enac_ws &&\
    colcon build --symlink-install &&\
    source install/local_setup.bash