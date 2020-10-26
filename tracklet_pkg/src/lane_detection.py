#!/usr/bin/env python
# -*- coding: utf-8 -*

from __future__ import print_function

import sys
import rospy
import numpy as np
import tf
import roslib
import cv2
import matplotlib as plt

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math

def lanesDetection(img):
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    canny_edges = canny_edge_detector(img)

    cropped_image = region_of_interest(canny_edges)

    # cv2.imshow("show", cropped_image)
    # cv2.waitKey(2)

    lines = cv2.HoughLinesP(
        cropped_image,
        rho=2,              #Distance resolution in pixels
        theta=np.pi / 180,  #Angle resolution in radians
        threshold=100,      #Min. number of intersecting points to detect a line  
        lines=np.array([]), #Vector to return start and end points of the lines indicated by [x1, y1, x2, y2] 
        minLineLength=40,   #Line segments shorter than this are rejected
        maxLineGap=25       #Max gap allowed between points on the same line
    )

    result = lineFitting(img, lines, (50, 255, 255), 5, 0.8)

    return result


def canny_edge_detector(frame):

    gray_img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    blur = cv2.GaussianBlur(gray_img, (5, 5), 0)

    cannyed_edge = cv2.Canny(blur, 50, 150)

    
    return cannyed_edge

# cropping the region of interest
def region_of_interest(img):
    height = img.shape[0]
    width = img.shape[1]

    # A triangular polygon to segment the lane area and discarded other irrelevant parts in the image
    # Defined by three (x, y) coordinates    
    polygons = np.array([ 
        [(300, height), (round(width/2), round(height/2)), (width-300, height)] 
    ])

    mask = np.zeros_like(img) 

    cv2.fillPoly(mask, np.array([polygons], np.int32), 255) 
    # Fill inside the polygon
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image

def get_coordinates (image, params):
     
    slope, intercept = params 
    y1 = image.shape[0] 
    y2 = int(y1 * (3/5)) # Setting y2 at 3/5th from y1
    x1 = int((y1 - intercept) / slope) # Deriving from y = mx + c
    x2 = int((y2 - intercept) / slope) 
    
    return np.array([x1, y1, x2, y2])


def splitTwoSideLines(lines, slope_threshold=(4*np.pi/180)):
    lefts = []
    rights = []

    if lines is None:
        # print("check")
        return None, None

    for line in lines:
        x1 = line[0, 0]
        y1 = line[0, 1]
        x2 = line[0, 2]
        y2 = line[0, 3]

        if (x2-x1) == 0:
            continue
        
        slope = (float)(y2-y1)/(float)(x2-x1)

        if math.fabs(slope) < slope_threshold:
            continue

        if slope <= 0:
            lefts.append([slope, x1, y1, x2, y2])
        else:
            rights.append([slope, x1, y1, x2, y2])
    
    return lefts, rights

def medianPoint(x):
    if x is None or len(x)==0:
        return None
    else:
        xx = sorted(x)
        return xx[(int)(len(xx)/2)]

def interpolate(x1, y1, x2, y2, y):
    if (x1 and y1 and x2 and y2 and y):
        return int(float(y-y1) * float(x2-x1) / float(y2-y1) + x1)
    else:
        return 1

def lineFitting(image, lines, color=(0, 255, 255), thickness=3, slope_threshold=(5*np.pi/180)):
    result = np.copy(image)

    height = image.shape[0]
    lefts, rights = splitTwoSideLines(lines, slope_threshold)
    left = medianPoint(lefts)
    right = medianPoint(rights)

    min_y = int(height*0.6)
    max_y = height
    
    if left is not None:
        min_x_left = interpolate(left[1], left[2], left[3], left[4], min_y)
        max_x_left = interpolate(left[1], left[2], left[3], left[4], max_y)
    else:
        min_x_left = None
        max_x_left = None
    
    if right is not None:
        min_x_right = interpolate(right[1], right[2], right[3], right[4], min_y)
        max_x_right = interpolate(right[1], right[2], right[3], right[4], max_y)
    else:
        min_x_right = None
        max_x_right = None

    try:
        if min_x_left and max_x_left:
            cv2.line(result, (min_x_left, min_y), (max_x_left, max_y), color, thickness)
        
        if min_x_right and max_x_right:
            cv2.line(result, (min_x_right, min_y), (max_x_right, max_y), color, thickness)
    except AttributeError as e:
        print(e)

    return result