To build :
```
docker build . -f enac_dependencies.Dockerfile -t enac_ros
docker build . -f enac_base.Dockerfile -t enac_base
```

To run - No GUI :
```
docker run -it --net=host enac_base bash
```

To run - GUI Linux :
``` 
xhost + 
 
docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:noetic-desktop-full \
    bash -it -c "roslaunch gazebo_ros empty_world.launch"
```
To run - GUI Windows :
```
```

La bible docker/ROS :
https://roboticseabass.com/2021/04/21/docker-and-ros/
