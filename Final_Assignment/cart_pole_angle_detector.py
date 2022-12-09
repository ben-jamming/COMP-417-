"""
Find the angle (in radians) between the cart (blue) and the pole (red)
"""
import skimage.exposure
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
from PIL import Image
  
def centeroid(points: np.array) -> np.array:
    length, dim = points.shape
    return np.array([int(np.sum(points[:, i])/length) for i in range(dim)])

def calculate_angle(theta):
    #print("RAW ANGLE IS: ",theta)
    if theta > 45:
        #tilting left
        #print("TILTING LEFT @ {d} or {r} radians".format(d=(theta-90), r = (theta*(math.pi/180))-(math.pi/2)))
        return ((theta*(math.pi/180))-(math.pi/2))
    
    else:
        #tilting right or standing up
        #print("TILTING RIGHT @ {d} or {r} radians".format(d=theta,r=(theta*(math.pi/180))))
        return (theta*(math.pi/180))

def find_pole(im):
    #im = cv2.imread('/Users/kids/Documents/McGill/U2/COMP 417/COMP-417/COMP-417-/Final_Assignment/'+im)
    lower_red = np.array([240, 0, 0])
    upper_red = np.array([255, 0, 0])
    mask1 = cv2.inRange(im,lower_red, upper_red)
    img_res = cv2.bitwise_and(im, im, mask = mask1)
    gray = cv2.cvtColor(img_res,cv2.COLOR_BGR2GRAY)
    _,thresh = cv2.threshold(gray,0,250,cv2.THRESH_BINARY)
    #cv2.imshow("thresh",thresh)
    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    if len(cnts)==0:
            print("box not on screen") 
            return 0
    for c in cnts:
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img_res,[box],0,(50,205,50), 1)
        """
        Uncomment to show the pole detection live
        """
        #cv2.imshow("result",img_res)
        return calculate_angle(rect[2])
