import cv2 
from realsense_camera import* 
 

#Load Realsese camera 
rs = RealsenseCamera() 
img = cv2.imread('kayla.jpg')

while True: 
    ret, bgr_frame, depth_frame = rs.get_frame_stream()

    point_x, point_y = 250, 100
    distance_mm = depth_frame[point_y, point_x]

    #cv2.circle(bgr_frame, (point_x, point_y), 8, (0, 0, 255), -1)
    #cv2.putText(bgr_frame, "{} mm".format(distance_mm), (point_x, point_y - 10), 0, 1, (0, 0, 255), 2)

   
    #cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Robotic Eyesight", bgr_frame)
    

    key = cv2.waitKey(1)
    if key == 27:
        break 