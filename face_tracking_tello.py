import cv2
from helper import *

#to kill drone process: "ps -elf | grep python"
#width and height for drone stream
w, h = 720, 480

#PID controllers for drone motor control
#1 = yaw
#2 = up / down
#3 = forward / backward
pid1 = [.3, .4, 0] #.4, .7, 0
pid2 = [.2, .4, 0] #[.5, .5, 0]
pid3 = [.1, .5, 0]

#PID controller error
previous_error1 = previous_error2 = previous_error3 = 0

#start counter for drone takeoff and for debugging
#0 = flight 
# 1 = no-flight
start_counter = 0

#starts drone and connects
drone = start_tello()

#connects to drones video stream
#Keyboard Key "Q" lands and stops drone
while True:

    if start_counter == 0:
        drone.takeoff()
        start_counter = 1

    img = get_tello_frame(drone, w, h)

    #draws green circle in middle of video frameqq
    cv2.circle(img, (w//2, h//2), 3, (0, 255, 0), 5)  

    #finds closest face found in image
    #draws red box around face,
    #outputs img = img & info = [(cx, cy), area]
    #info[0][0] = cx of closest detected face in frame
    #info[1] = cy of closest detected face in frame
    img, info = find_face(img)

    '''
    TODO:
    Make it continuously go in a direction slowly if it loses face, if face last seen to right, continue right and vice versa
    '''

    #Calculates error and motor control based on three pid controllers
    previous_error1, previous_error2, previous_error3 = follow_face(drone, info, w, h, pid1, pid2, pid3, previous_error1, previous_error2, previous_error3)

    #displays formatted video stream to window
    cv2.imshow('Image', img)

    #when keyboard key 'Q' is clicked, emergency end drone flight
    key = cv2.waitKey(1)
    if (key == ord('q')):
        drone.land()
        drone.end()
        break

#destroys video stream window
cv2.destroyAllWindows()




'''
EXTRA CODE SNIPPETS

drone.forward_backward_velocity = 0
drone.left_right_velocity = 0
drone.up_down_velocity = 0

if drone.send_rc_control:
    drone.send_rc_control(drone.left_right_velocity,
                            drone.forward_backward_velocity,
                            drone.up_down_velocity,
                            drone.yaw_velocity)
    
'''