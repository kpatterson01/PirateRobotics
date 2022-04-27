# trying this 
import cv2 
import pyrealsense2 as rs
 

# Load Realsense camera 
while True: 
    bgr_frame = rs.frame()

    #cv2.imshow("Bgr frame", bgr_frame)

    key = cv2.waitKey(1)
    if key == 27:
        break 



