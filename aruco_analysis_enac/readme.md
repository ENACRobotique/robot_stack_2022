## Utilitaire de création de code aruco à imprimer
https://chev.me/arucogen/

## calibrer la caméra
lancer la caméra :
ros2 run v4l2_camera_driver v4l2_camera_node --ros-args -p 'image_size':[1920,1080]

Lancer l'outil de calibration :
ros2 run aruco_analysis_enac camera_calibrator --ros-args -p use_console_input:=false -p calibration_relative_path:=*PATH*

Sur une IHM (Foxglove studio):
Publier sur les topics Take picture/generate calibration file

TODO : publier avec les marqueurs l'image downscaled
TODO : renommer calibration_relative to absolute_path


ros2 run aruco_analysis_enac camera_calibrator --ros-args -p calib_path:='/enac_ws/src/aruco_analysis_enac/calibration/Calib_fisheye_480'

### Workflow pour calibrer en remote
Lancer le driver & le node de calibration
Puis ramener le fichier en local : 
scp ubuntu@192.168.42.164:/home/ubuntu/.ros/camera_info/mmal_service_16.1.yaml ~


 ## utilisation normale

Launch file sur la raspy :

Debuggage à la main detect_aruco : 

 ros2 run aruco_analysis_enac detect_aruco --ros-args -p image_size:=[1280,720]


