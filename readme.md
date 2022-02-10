# Usage

## To run - basic version
for GUI : 
``` 
xhost local:root 
```
For the rest : 
```
docker run -it --net=host -e DISPLAY --volume /home/robot/ros_aruco/src/robot_stack_2022:/enac_ws/src --volume /home/robot/bag_files:/enac_ws/bag enacrobotique/enac-base bash
```

## Useful commands

#### Connection with rosbridge (for foxglove,...)

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

#### Add others bash terminals

```
docker exec -it club_robot bash

ros2 bag play /enac_ws/bag/one_marker_1/one_marker_1_0.db3 -l
```
#### durant un run, supprimer les containers en exitant :
    -d



## Windows specifics - GUI

### Installation - X11

Install Chocolatey

    choco install vcxsrv

Launch Xlaunch, with these config :

    Multiple windows
    Start no client
    Clipboard, Primary Selection, Disable Access Control 
    !! untick native opengl !!
    save configuration

En cas de soucis avec les GUI : vérifier les drivers de la carte graphique (du pc sous windows)
### running

ipconfig

obtenir l'addresse IP WSL

```
docker run -it --rm --name club_robot --net=host --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="DISPLAY=172.29.208.1:0.0" --env="QT_X11_NO_MITSHM=1" --env="LIBGL_ALWAYS_INDIRECT=0" --volume D:\Sync\Code\Robotique\CDR2022\robot_stack_2022:/enac_ws/src --volume "C:\Users\Jonathan\Downloads\Aruco data":/enac_ws/bag enacrobotique/enac-base bash
```

### pour faire marcher foxglove :

replacer --net=host par 
     -p "9090:9090" 
# Developpers

## Building

### target AMD64 and ARM64 (raspy included):

Configure the multiarch container : 
```
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker buildx rm builder
docker buildx create --name builder --driver docker-container --use
docker buildx inspect --bootstrap
```
Then compile :
```
sudo docker buildx build --platform linux/amd64,linux/arm64 . -f enac_dependencies.Dockerfile -t enacrobotique/enac-ros --push

sudo docker buildx build --platform linux/amd64,linux/arm64 . -f enac_base.Dockerfile -t enacrobotique/enac-base --push
```
### AMD64 only:
```
docker build . -f enac_dependencies.Dockerfile -t enacrobotique/enac-ros
docker build . -f enac_base.Dockerfile -t enacrobotique/enac-base
```




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





