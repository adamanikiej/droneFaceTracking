import time
from djitellopy import Tello
import cv2
import numpy as np

expected_face_area = 10000
height_offset = 50

def start_tello():
    '''
    Connects to drone and sets velocities and speed to 0
    Cycles drone stream
    '''
    #makes drone and connects to it
    drone = Tello()
    drone.connect()

    #sets all drones velocity to 0
    drone.forward_backward_velocity = 0
    drone.left_right_velocity = 0
    drone.up_down_velocity = 0
    drone.yaw_velocity = 0
    drone.speed = 0

    #cycles drone stream off and on
    drone.streamoff()
    drone.streamon()

    #prints drone's battery at start
    time.sleep(5)
    print(drone.get_battery())

    return drone

def get_tello_frame(drone, width = 360, height = 240):
    '''
    Returns video stream frame from drone
    '''
    drone_frame = drone.get_frame_read()
    drone_frame = drone_frame.frame

    img = cv2.resize(drone_frame, (width, height))

    return img

def find_face(img):
    '''
    Returns video stream frame with added red box around detected faces
    Uses haarcascade_frontalface_default.xml for facial detection
    '''
    face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    faces = face_cascade.detectMultiScale(img_gray, 1.2, 4)

    my_face_list_c =[]
    my_face_list_area = []

    for (x, y, w, h) in faces:
        #draws red box around detected face
        cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,255),2)
        cx = x + (w/2)
        cy = y + (y/2)

        #draws circle in middle of face
        cv2.circle(img, (x+(w//2), y+(h//2)), 3, (0, 0, 255), 5)

        area = w*h
        my_face_list_area.append(area)
        my_face_list_c.append([cx, cy])

    #if there is more than one detected face, use the closer one
    if len(my_face_list_area) != 0:
        i = my_face_list_area.index(max(my_face_list_area))
        return img, [my_face_list_c[i], my_face_list_area[i]]
    else:
        return img, [[0, 0], 0]

def follow_face(drone, info, width, height, pid1, pid2, pid3, previous_error1, previous_error2, previous_error3):
    '''
    Adjusts drones motors to follow face
    uses three pid controllers with seperate errors and deadzone
    '''
    '''
    keep soft locks on pid if face is in frame, if it isn't then turn hard towards whatever direction last seen face
    stop turning or trying to find face after 5 seconds and then set all values back to 0
    '''
    #PID for left and right movement
    error1 = info[0][0] - width//2
    if error1 > -60 and error1 < 60:
        error1 = 0
    speed1 = int(pid1[0]*error1 + pid1[1]*(error1 - previous_error1))
    speed1 = int(np.clip(speed1, -50, 50))

    #PID for up and down movement
    error2 = height//2 - (info[0][1] + height_offset) #height = 240, info[0][1] > height when drone above person
    if error2 > -70 and error2 < 70:
        error2 = 0
    speed2 = int(pid2[0]*error2 + pid2[1]*(error2 - previous_error2))
    speed2 = int(np.clip(speed2, -30, 30))

    #PID for forward and backward movement
    error3 = -(info[1] - expected_face_area)
    if error3 > -3000 and error3 < 3000:
        error3 = 0
    speed3 = int(pid2[0]*error3 + pid3[1]*(error3 - previous_error3))
    speed3 = int(np.clip(speed3, -30, 30))
    #speed3 might need to be reversed


    print("area of face: ", info[1])
    print("face error: ", error3)
    
    '''
    print("up/down error w/ offset: ", error2)
    print("info[0][1]: ", info[0][1])
    '''

    '''
    print("speed: ", speed1)
    print("error: ", error1)

    if error1 > 0:
        print("Face is to the right")
        if speed1 > 0:
            print("Drone is going to the right")
        elif speed1 < 0:
            print("Drone is going to the left")
    elif error1 < 0:
        print("Face is to the left")
        if speed1 > 0:
            print("Drone is going to the right")
        elif speed1 < 0:
            print("Drone is going to the left")
    '''

    #if face exists in frame:
    if info[0][0] !=0:
        #print("face detected")
        drone.yaw_velocity = speed1
        drone.up_down_velocity = speed2        #NEW
        drone.forward_backward_velocity = speed3
    else:
        drone.forward_backward_velocity = 0
        drone.left_right_velocity = 0
        drone.up_down_velocity = 0
        drone.yaw_velocity = 0
        error1 = speed1 = error2 = speed2 = error3 = speed3 = 0
        print("no detected face")
    if drone.send_rc_control:
        drone.send_rc_control(drone.left_right_velocity,
                                drone.forward_backward_velocity,
                                drone.up_down_velocity,
                                drone.yaw_velocity)

    return error1, error2, error3
