import cv2
import numpy as np
from time import sleep

print("test")

this_marker = np.zeros((300, 300,1), dtype="uint8")
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
cv2.aruco.drawMarker(dict, 5, 300, this_marker, 1)
cv2.imwrite("test_aruco_501.jpg", this_marker)
cv2.imshow("aa", this_marker)
sleep(1000000)


