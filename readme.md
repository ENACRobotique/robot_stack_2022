# To build :
```
docker build . -f enac_dependencies.Dockerfile -t enacrobotique/enac-ros
docker build . -f enac_base.Dockerfile -t enacrobotique/enac-base
```

# To run - No GUI :
```
docker run -it --net=host enacrobotique/enac-base bash --volume PATH/TO/ROBOT_STACK_2022:/enac_ws/src
```
docker run -it --net=host --volume D:\Sync\Code\Robotique\CDR2022\robot_stack_2022:/enac_ws/src  enacrobotique/enac-base bash

# To run - GUI Linux :

**TODO**
``` 
xhost local:root 
+

docker run -it --net=host -e DISPLAY --volume  /home/robot/ros_aruco/src/robot_stack_2022:/enac_ws/src /home/robot/bag_files:/enac_ws/bag enacrobotique/enac-base bash
 
docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:noetic-desktop-full \
    bash -it -c "roslaunch gazebo_ros empty_world.launch"
```
# To run - GUI Windows :

## Installation X11 Windows
Install Chocolatey

    choco install vcxsrv

Launch Xlaunch, with these config :

    Multiple windows
    Start no client
    Clipboard, Primary Selection, Disable Access Control 
    !! untick native opengl !!
    save configuration


## Commande pour NVIDIA :

**TODO : faire pas que pour mon PC**

ipconfig

obtenir l'addresse IP WSL

docker run -it --rm --net=host --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="DISPLAY=172.29.208.1:0.0" --env="QT_X11_NO_MITSHM=1" --env="LIBGL_ALWAYS_INDIRECT=0" --volume D:\Sync\Code\Robotique\CDR2022\robot_stack_2022:/enac_ws/src --volume "C:\Users\Jonathan\Downloads\Aruco data":/enac_ws/bag enacrobotique/enac-base bash

ros2 bag play /enac_ws/bag/one_marker_1/one_marker_1_0.db3 -l

En cas de soucis avec les GUI : vérifier les drivers de la carte graphique (du pc sous windows)

(références : https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde
https://marinerobotics.gtorg.gatech.edu/running-ros-with-gui-in-docker-using-windows-subsystem-for-linux-2-wsl2/)

## Known issues :
1. Rviz2 use all the cpu and at ~20fps with nothing
2. GUI is a little bit slow in general 

## improvement possibilities :
1. Use NVIDIA cuda image as base to get graphic acceleration


# Pour les devs : 
### La bible docker/ROS :
https://roboticseabass.com/2021/04/21/docker-and-ros/
### Retirer les images inutilisés :
    docker image prune
    docker container prune

### durant un run, supprimer les containers en exitant :
    -d



