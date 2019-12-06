#module for geometric shapes on the x,y plane

import math

def mid_point(p1,p2):  #returns a point in the middle of p1 and p2
    x,y =p1
    x2,y2 =p2
    return ((x+x2)/2,(y+y2)/2)

def angle_of_vector(vector):    #returns the angle between the x axis and the vector
    c1,c2 = vector
    x,y =c1
    x2,y2 =c2
    if(x-x2!=0):
                if(x>x2):
                        ceta =  ((math.atan((y-y2)/(x-x2))/math.pi)*180)
                else:
                        if(y>y2):
                                ceta =  180+((math.atan((y-y2)/(x-x2))/math.pi)*180)
                        else:
                                ceta =  -180+((math.atan((y-y2)/(x-x2))/math.pi)*180)
    if(x-x2==0):
            if(y>y2):
                ceta = 90
            else:
                ceta = -90
    return ceta

def magnitud_of_vector(vector):     #returns the magnitud of a vector
    p1,p2 = vector
    x,y =p1
    x2,y2 =p2
    return(math.sqrt((x-x2)**2+(y-y2)**2))
    
def sum_of_vectors(vector1,vector2):
    p1,p2 = vector1
    p3,p4 = vactor2
    x1,y1 = p1
    x2,y2 = p2
    x3,y3 = p3
    x4,y4 = p4
    if(p1 != p3):
        raise Exception("base of vectors must be the same")
    return(p1,(x2+x4,y2+y4))
