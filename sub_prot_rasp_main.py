from __future__ import division    #neccesary for cv2
import numpy as np    #neccesary for cv2
import cv2
import camera_m as camera
from camera_m import removearray as remove_a   #because of a bug we need a function for sorting through arrays
import geogebra_m as geo
import serial   #for bluetooth
import time

camera0 = camera.setup(320,240) #width,height

sliderstart = (0, 0, 0, 255, 255, 255)   #staring values for sliders
camera.setup_sliders(sliderstart)

last_time = 0

bluetooth = 0   #variable for current state

while True:
    slider_values = camera.read_sliders()
    #gets values from sliders

    frame = camera.snap(camera0)
    camera.show_img(frame)
    mask = camera.show_mask(frame,slider_values)
    contours = camera.find_contours(mask)
    
    if(len(contours)>1):    #ckecks to make sure there are atleast two contours
        
        biggest_contour = max(contours, key=cv2.contourArea)
        remove_a(contours,biggest_contour)  #removes the biggest contour from contours
        second_biggest_contour = max(contours, key=cv2.contourArea)
        
        camera.show_contour(biggest_contour,frame)
        camera.show_contour(second_biggest_contour,frame)
        c1 = camera.get_center(biggest_contour)
        c2 = camera.get_center(second_biggest_contour)
        c_obj = geo.mp(c1,c2)   #center of the obj
        v_obj = (c_obj,c1)  #defines a vector for the obj     
        ceta = geo.angle_of_vector(v_obj)   #finds the angle between the x axis and the vector
        o = (0,0)   #origin point
        o_vector= (c_obj,o) #vector from the center of the object to the origin
        ceta2 = geo.angle_of_vector(o_vector)
        mag = geo.mag_v(o_vector)   #magnitud of th vector aka distance to center
        print(ceta,",",ceta2,",",mag,";")
        if(bluetooth == 1):
            if(time.time()-last_time>1):    #waits a second to send data
                port.write(str(int(ceta)).encode('utf-8'))
                port.write(",").encode('utf-8'))
                port.write(str(int(ceta2)).encode('utf-8'))
                port.write(",").encode('utf-8'))
                port.write(str(int(mag)).encode('utf-8'))
                port.write(";").encode('utf-8'))
                print("sending")
                last_time=time.time()
        
    if(camera.wait_for_exit('a',5)==1):   #key to exit, milliseconds to wait
        if(bluetooth == 0):
            port = serial.Serial("/dev/rfcomm"+input("port number"), baudrate=9600)
            port.write(input("primer coeficiente").encode('utf-8'))
            port.write(input("segundo coeficiente").encode('utf-8'))
            bluetooth = 1
        else:
            bluetooth = 0
        
cv2.destroyAllWindows()
camera0.release()
