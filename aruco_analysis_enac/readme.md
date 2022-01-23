## Utilitaire de création de code aruco à imprimer
https://chev.me/arucogen/

## calibrer la caméra
    ros2 run camera_calibration cameracalibrator --size=8x6 --square=0.02 --no-service-check --ros-args --remap /image:=/camera/image_raw

 ?? --no-service-checker