## Utilitaire de création de code aruco à imprimer
https://chev.me/arucogen/

## calibrer la caméra
    ros2 run camera_calibration cameracalibrator -s 7x9 -q 0.2 --no-service-check image:=/camera/image camera:=/camera

 ?? --no-service-checker

 ## utilisation normale

Launch file sur la raspy :

Debuggage à la main detect_aruco : 

 ros2 run aruco_analysis_enac detect_aruco
