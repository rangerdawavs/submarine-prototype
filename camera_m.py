#module: camera_m
#Author: olcosubmarino
#Date: october 2019

from __future__ import division
import numpy as np
import cv2

def nothing(*arg):  #no idea why this is here
        pass
    
def removearray(L,arr): #there is an error when trying to eliminate arrrays
    ind = 0             #in a list, this fixes it manually 
    size = len(L)
    while ind != size and not np.array_equal(L[ind],arr):
        ind += 1
    if ind != size:
        L.pop(ind)
    else:
        raise ValueError('array not found in list.')

def setup(width,height):    #sets up camera, possible extension in the future
    FRAME_WIDTH = width
    FRAME_HEIGHT = height
    # Initialize webcam. Webcam 0 or webcam 1 or ...
    vidCapture = cv2.VideoCapture(0)
    vidCapture.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
    vidCapture.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)

    return vidCapture

def setup_sliders(starting_values2):    #setsup trackbars and windows
    icol = starting_values2     #starting values for trackbars
    cv2.namedWindow('colorTest')
    # Lower range colour sliders.
    cv2.createTrackbar('lowHue', 'colorTest', icol[0], 255, nothing)
    cv2.createTrackbar('lowSat', 'colorTest', icol[1], 255, nothing)
    cv2.createTrackbar('lowVal', 'colorTest', icol[2], 255, nothing)
    # Higher range colour sliders.
    cv2.createTrackbar('highHue', 'colorTest', icol[3], 255, nothing)
    cv2.createTrackbar('highSat', 'colorTest', icol[4], 255, nothing)
    cv2.createTrackbar('highVal', 'colorTest', icol[5], 255, nothing)

def read_sliders():
    lowHue = cv2.getTrackbarPos('lowHue', 'colorTest')
    lowSat = cv2.getTrackbarPos('lowSat', 'colorTest')
    lowVal = cv2.getTrackbarPos('lowVal', 'colorTest')
    highHue = cv2.getTrackbarPos('highHue', 'colorTest')
    highSat = cv2.getTrackbarPos('highSat', 'colorTest')
    highVal = cv2.getTrackbarPos('highVal', 'colorTest')

    return lowHue, lowSat, lowVal, highHue, highSat, highVal

def snap(vidCapture2): 
    vidCapture = vidCapture2
    # Get webcam frame
    _, frame = vidCapture.read()

    return frame

def show_img(frame):
    # Show the original image.
    cv2.imshow('frame', frame)

def show_mask(frame,slider_values2):
    # Convert the frame to HSV colour model.
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowHue, lowSat, lowVal, highHue, highSat, highVal = slider_values2
    
    # HSV values to define a colour range we want to create a mask from.
    colorLow = np.array([lowHue,lowSat,lowVal])
    colorHigh = np.array([highHue,highSat,highVal])
    mask = cv2.inRange(frameHSV, colorLow, colorHigh)
    # Show the first mask
    cv2.imshow('mask-plain', mask)

    return mask

def wait_for_exit(key,milli):
    k = cv2.waitKey(milli) & 0xFF
    if k == key:
        return 1
    else:
        return 0

def find_contours(mask):
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def get_center(contour):
    x,y,w,h = cv2.boundingRect(contour)
    xc =x+w/2-160
    yc =y+h/2-120
    return xc,yc

def show_contour(contour,frame):
    x,y,w,h = cv2.boundingRect(contour)
    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.imshow('colorTest', frame)
