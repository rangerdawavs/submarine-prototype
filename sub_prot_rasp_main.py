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
sliderstart = (90, 85, 50, 130, 255, 255)   #staring values for sliders
camera.setup_sliders(sliderstart)


prom_list=[]
n=0
bluetooth = 0   #variable for current state
xPostion_list= calculus.track(2)
yPostion_list= calculus.track(2)
Rotation_list=calculus.track(2)
time_list= calculus.track(2)
stateVector= [0,0,0,0,0,0] # ordered as x, xdot,y,ydot,angle,angledot
roombaRadius= 0.10 # this is the radius of the roomba in SI units
loopN=-1 #Loop number, This is used to make sure only in certain loops the bluetooth is sent+ removes derivative bug

def printState(stateVector):
    #this function prints the statevector in a nice format
    #slightly laggy
    pString=""
    for i in range(4):
        if stateVector[i]<0: # for the first 4 numbers, which have same format
            pString= pString+str("%2.4f"%(stateVector[i]) )
        else:
            pString=pString+str(" " )+ str("%2.4f" %(stateVector[i]) )
    if stateVector[4]<0:# only shows 1 digit
        pString=pString+str("%2.1f" %(stateVector[4]))
    else:
        pString=pString+str(" " )+str("%2.1f" %(stateVector[4]) )
    if stateVector[5]<0:
        pString=pString+str("%2.2f" %(stateVector[5]))
    else:
        pString=pString+str(" " )+str("%2.2f" %(stateVector[5]) )                            
    print(pString)

def compressToSend(nDataType,pVector,pItemList):
    #This function turns the information from a given Vector into a string to
    #Send it over bluetooth. It includes data type number and multiplies the remaining
    #Values by a 1000 to include decimals. It also adds the end line character(z)
    #nDataType is an int, pVector a vector,pItem list should be a array of numbers
    pString= str(nDataType)+","
    for i in pItemList:
        pString= pString+str(int(pVector[i]*1000))+","
    pString= pString+"z"
    return pString

while True:
   
    slider_values = camera.read_sliders()#gets values from sliders
    frame = camera.snap(camera0)
    camera.show_img(frame)
    mask = camera.show_mask(frame,slider_values)
    contours = camera.find_contours(mask)
    
    if(len(contours)>1):    #ckecks to make sure there are atleast two contours
        loopN+=1# during first loop it will equal 0
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

        if(bluetooth == 1): # This is where bluetooth is sent
            if((loopN%5!=0)):  
                if((stateVector[1]==0)and(stateVector[3]==0)and(stateVector[5]==0)): # if object is steady
                    port.write((compressToSend(2,stateVector,range(0,6,2))).encode('utf-8'))
                else:
                    port.write((compressToSend(0,stateVector,range(6))).encode('utf-8'))
            else:
                 if (loopN!= 0):# ensures no prints during first loop when derivative is 0
                     #printState(stateVector)
                     pass

               # This is where the average duration of a loop is calculated
            if(n<100):
                prom_list.append(calculus.der(time_list,[0,1],-1))# adds the time since the last loop to a list to be averaged
                n+=1
            else:
                temp_int=0
                for slc in prom_list:
                    temp_int+=slc
                temp_int/=100
                print(temp_int)
                prom_list=[]
                n=0
            
                    
    if(camera.wait_for_exit('a',5)==1):   #key to exit, milliseconds to wait
        if(bluetooth == 0):
            port = serial.Serial("/dev/rfcomm"+input("port number"), baudrate=9600)
            '''
            port.write(("1").encode('utf-8'))
            port.write(input("primer coeficiente").encode('utf-8'))
            port.write(input("segundo coeficiente").encode('utf-8'))
            port.write(input("tercer coeficiente").encode('utf-8'))
            port.write(input("cuarto coeficiente").encode('utf-8'))
            port.write(input("deadband position").encode('utf-8'))
            port.write(input("deadband rotation").encode('utf-8'))
            '''
            setupString= "1,"
            setupString=setupString+input("position proportional")+ ","
            setupString=setupString+input("position derivative")+ ","
            setupString=setupString+input("position integral")+ ","
            setupString=setupString+input("rotation proportionals")+ ","
            setupString=setupString+input("rotation derivative")+ ","
            setupString=setupString+input("rotation integral")+ ","
            port.write((setupString).encode('utf-8'))
            bluetooth = 1
        else:
            bluetooth = 0

cv2.destroyAllWindows()
camera0.release()
