# Usage

## To run - basic version
for GUI : 
``` 
xhost local:root 
```
For the rest : 
```
docker run -it --rm --net=host -e DISPLAY --name club_robot --volume /home/robot/ros_aruco/src/robot_stack_2022:/enac_ws/src --volume /home/robot/bag_files:/enac_ws/bag enacrobotique/enac-base:dev bash
```

Ordi de gauche :
docker run -it --rm --net=host -e DISPLAY --name club_robot --volume /home/robot/Documents/robot_stack/robot_stack_2022:/enac_ws/src --volume /home/robot/bag_files:/enac_ws/bag enacrobotique/enac-base:dev bash

--device=/dev/ldlidar --device=/dev/stm32 

docker run -it --privileged --rm --name club_robot --net=host --pid=host -e DISPLAY -v /dev/bus/usb:/dev/bus/usb enacrobotique/enac-base:prod bash


PI 4 with camera :
docker run -it --privileged --rm --name club_robot --net=host --pid=host -e DISPLAY --device=/dev/ttyACM0 enacrobotique/enac-base:prod bash

ros2 run ros2serial ros2serial --ros-args -p serial_port:=/dev/ttyACM0 -p baudrate:=115200


## Useful commands

#### Connection with rosbridge (for foxglove,...)

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

#### Add others bash terminals

```
docker exec -it club_robot bash

ros2 run robot_simu_enac simu_robot

ros2 bag play /enac_ws/bag/one_marker_1/one_marker_1_0.db3 -l
```



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
docker run -it --rm -p "9090:9090" --name club_robot --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="DISPLAY=172.29.208.1:0.0" --env="QT_X11_NO_MITSHM=1" --env="LIBGL_ALWAYS_INDIRECT=0" --volume D:\Sync\Code\Robotique\CDR2022\robot_stack_2022:/enac_ws/src --volume "C:\Users\Jonathan\Downloads\Aruco data":/enac_ws/bag enacrobotique/enac-base:dev bash
```

melvin environnement virtuel : 
docker run -it --rm --net=host -e DISPLAY --privileged --name club_robot --volume /home/ubuntu/enac_ws/src/robot_stack_2022:/enac_ws/src --volume /home/ubuntu/rosbag:/enac_ws/bag enacrobotique/enac-base:dev bash

cd enac_ws/
colcon build --packages-select lidar_location
source install/local_setup.bash 
ros2 run lidar_location comm_node

docker exec -it club_robot bash
cd enac_ws/
ros2 bag play bag/rosbagfix/

### pour faire marcher foxglove (sous windows):

replacer --net=host par 
      -p "9090:9090"
# Developpers

## List of images available :

enac_dependencies:latest -> (ARM64+AMD64) All the dependencies needed by the club to run

**enac_base:prod** -> (ARM64+AMD64) The base image to use for the club USAGE

enac_base:dev -> (ARM64+AMD64) The base image to use for the club DEVELOPMENT (SYMLINK install so source is needed in the volume)

enac_base:raspy_win (ARM64) -> Future deprecration, just to use when compilling for ARM64 from windows in case of problem

*enac_base:latest -> to use for people who forget to set a tag*
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

Pour la version **prod** 
    --build-arg DEV="False"

docker buildx build --build-arg DEV="False" --platform linux/amd64,linux/arm64 . -f enac_base.Dockerfile -t enacrobotique/enac-base:prod --push

Pour la version **dev** (avec le volume)
    --build-arg DEV="True"

docker buildx build --build-arg DEV="True" --platform linux/amd64,linux/arm64 . -f enac_base.Dockerfile -t enacrobotique/enac-base:dev --push

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
3. When using 

## improvement possibilities :
1. Use NVIDIA cuda image as base to get graphic acceleration


# Pour les devs : 
### La bible docker/ROS :
https://roboticseabass.com/2021/04/21/docker-and-ros/
### Retirer les images inutilisés :
    docker image prune
    docker container prune





