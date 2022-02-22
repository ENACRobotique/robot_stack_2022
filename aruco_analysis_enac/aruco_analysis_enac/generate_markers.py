import cv2

dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
def generate_markers(id):
    """generate pdf file for printing aruco marker in a4 paper format given the id and using dict """
    img = cv2.aruco.drawMarker(dict, id, 700)
    cv2.imwrite(f"aruco_marker_{id}.jpg", img)