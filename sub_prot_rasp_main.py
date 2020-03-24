from __future__ import division    #neccesary for cv2
import numpy as np    #neccesary for cv2
import cv2
import camera_m as camera
from camera_m import removearray as remove_a   #because of a bug we need a function for sorting through arrays
import geogebra_m as geo
import serial   #for bluetooth
import time
import calculus1_m as calculus

camera0 = camera.setup(320,240) #width,height

sliderstart = (96, 195, 106, 255, 255, 209)   #staring values for sliders
camera.setup_sliders(sliderstart)

last_time = 0
bluetooth = 0   #variable for current state
xPostion_list= calculus.track(2)
yPostion_list= calculus.track(2)
Rotation_list=calculus.track(2)
time_list= calculus.track(2)
stateVector= [0,0,0,0,0,0] # ordered as x, xdot,y,ydot,angle,angledot
roombaRadius= 0.10 # this is the radius of the roomba in SI units
i=0 # this makes sure that the first loop, where the derivative is error, it won’t be printed

while True:
	slider_values = camera.read_sliders()#gets values from sliders
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
        obj_center = geo.mp(geo.vector(c1,c2))   #center of the obj
        obj_vector = geo.vector(obj_center,c1)  #defines a vector for the obj
        
        ceta = geo.mix_vector(obj_vector)[1][1]   #finds the angle between the x axis and the vector
        origin = geo.origin()   #origin point
        origin_vector= geo.vector(obj_center,origin) #vector from the center of the object to the origin
        ceta2 = geo.mix_vector(origin_vector)[1][1]
        mag = geo.mix_vector(origin_vector)[1][0]   #magnitud of th vector aka distance to center
        
	calcRadius = geo.mix_vector(obj_vector)[1][0]   # the calculated radius of the roomba
	
        camera.draw_vector(frame,obj_vector)
        camera.draw_vector(frame,origin_vector)
		#here we update the lists
		xPostion_list= calculus.update(xPostion_list,obj_center[0]*(roombaRadius/calcRadius)) #includes unit conversion	
		yPostion_list=calculus.update(yPostion_list,obj_center[1]*(roombaRadius/calcRadius)) #includes unit conversion
		Rotation_list=calculus.update(Rotation_list,ceta)
		time_list= calculus.update(time_list,time.time())
		stateVector[0]=xPostion_list[-1] #the most recent item in the list
		stateVector[1]=calculus.der(xPostion_list,time_list,-1)
		stateVector[2]=yPostion_list[-1]
		stateVector[3]=calculus.der(yPostion_list,time_list,-1)
		stateVector[4]=Rotation_list[-1]
		stateVector[5]=calculus.der(Rotation_list,time_list,-1)
		if i==1:
			print(int(stateVector[0]),",",int(stateVector[1]),",",int(stateVector[2]),",",int(stateVector[3]),",",int(stateVector[4]),",",int(stateVector[5]))
		i=1
        if(bluetooth == 1):
            if(time.time()-last_time>1):    #waits a second to send data
                if((stateVector[1]==0)and(stateVector[3]==0)and(stateVector[5]==0)): # if object is steady
                    port.write(("2").encode('utf-8'))# ID of calibration information
                    port.write((",").encode('utf-8'))
                    port.write(str(int(stateVector[0])).encode('utf-8')) # position in x
                    port.write((",").encode('utf-8'))
                    port.write(str(int(stateVector[2])).encode('utf-8')) #position in y
                    port.write((",").encode('utf-8'))
                    port.write(str(int(stateVector[4])).encode('utf-8')) #rotation
				else:
					port.write(("0").encode('utf-8')) #ID of camera information
					port.write((",").encode('utf-8'))
					port.write(str(int(stateVector[0])).encode('utf-8')) # position in x
					port.write((",").encode('utf-8'))
					port.write(str(int(stateVector[1])).encode('utf-8')) #velocit in x
					port.write((",").encode('utf-8'))
					port.write(str(int(stateVector[2])).encode('utf-8')) #position in y
					port.write((",").encode('utf-8'))
					port.write(str(int(stateVector[3])).encode('utf-8'))#velocity in y
					port.write((",").encode('utf-8')) # inting the velocities might be a bug source since they might be small decimals
					port.write(str(int(stateVector[4])).encode('utf-8')) #angle
					port.write((",").encode('utf-8'))
					port.write(str(int(stateVector[5])).encode('utf-8')) #angle rate of change
				print("sending")
				last_time=time.time()
					
    if(camera.wait_for_exit('a',5)==1):   #key to exit, milliseconds to wait
        if(bluetooth == 0):
            port = serial.Serial("/dev/rfcomm"+input("port number"), baudrate=9600)
            port.write(("1").encode('utf-8'))
            port.write(input("primer coeficiente").encode('utf-8'))
            port.write(input("segundo coeficiente").encode('utf-8'))
            port.write(input("tercer coeficiente").encode('utf-8'))
            port.write(input("cuarto coeficiente").encode('utf-8'))
            port.write(input("deadband position").encode('utf-8'))
            port.write(input("deadband rotation").encode('utf-8'))
            bluetooth = 1
        else:
            bluetooth = 0

cv2.destroyAllWindows()
camera0.release()
