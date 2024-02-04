#!/usr/bin/python3
# -*- coding: utf-8 -*-
from threading import *
import time
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import message_filters
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from igvc_self_drive_gazebo.msg import Task # DM subsystem should change accordingly
from igvc_self_drive_gazebo.msg import CV  
from scipy.special import comb
import sys
from std_msgs.msg import Header, Int32
from numba.typed import List
from numba import njit
sys.setrecursionlimit(80000)
resx = 1280
resy = 720
res = 720//resy

@njit
def left_ref_lane(image):
  h,w = image.shape
  #image = image.tolist()
  white_arr = np.array([255])
  left_lane_points = [] #a list to store coordinates of the left lane points
  #left_lane_points_y =[]
  x_bound = 1
  y_bound = 1
  #while((x_bound!=0) and (y_bound!=0)):
  y_ref = h-1
  iteration = 0
  x_prev = 0
  y_prev = 0
  useless_lines_l = 0
  while(y_ref>=0):
    x_ref = 0
    while(x_ref<w):
      #if (image[y_ref][x_ref] == white_arr):
      arr1 = np.array(image[y_ref][x_ref])
      comp = arr1 == white_arr
      if (comp.all() and iteration==0):
        left_lane_points.append([y_ref, x_ref])
        x_prev = x_ref
        y_prev = y_ref
        iteration+=1
        break
      elif (comp.all() and iteration!=0):
        if(abs(y_ref- y_prev)<120):
          if(abs(x_ref - x_prev)<=20):
            left_lane_points.append([y_ref, x_ref])
            x_prev = x_ref
            y_prev = y_ref
            useless_lines_l = 0
            break
          else:
            useless_lines_l+=1
            break
        else:
          break        
      else:
        x_ref+=1
    if(useless_lines_l>5):
      break
    y_ref-=2

  #print(len(left_lane_points))
  return(left_lane_points[15:])
  



@njit
def right_ref_lane(image):
  h,w = image.shape
  white_arr = np.array([255])
  right_lane_points = [] #a list to store coordinates of the right lane points
  #left_lane_points_y =[]
  x_bound = 1
  y_bound = 1
  #while((x_bound!=0) and (y_bound!=0)):
  y_ref = h-1
  iteration = 0
  x_prev = 0
  y_prev = 0
  useless_lines_r = 0
  while(y_ref>=0):
    x_ref = w-1
    while(x_ref>=0):
      #if (image[y_ref][x_ref] == white_arr):
      arr1 = np.array(image[y_ref][x_ref])
      comp = arr1 == white_arr
      if (comp.all() and iteration==0):
        right_lane_points.append([y_ref, x_ref])
        x_prev = x_ref
        y_prev = y_ref
        iteration+=1
        break
      elif (comp.all() and iteration!=0):
        if(abs(y_ref-y_prev)<120):
          if(abs(x_ref-x_prev)<150):
            right_lane_points.append([y_ref, x_ref])
            x_prev = x_ref
            y_prev = y_ref
            useless_lines_r = 0
            break
          else:
            useless_lines_r+=1
            break
        else:
          break
      else:
        x_ref-=1
    if (useless_lines_r>5):
      break
    y_ref-=2

  print(len(right_lane_points))
  
  return(right_lane_points)

def bernstein_poly(i, n, t):
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

def bezier_curve(points, nTimes=1000):
    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])
    t = np.linspace(0.0, 1.0, nTimes)
    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])
    #print(polynomial_array)
    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)
    return xvals, yvals

def polyfit_lane_right(xvals, yvals, img):
    # x_points = [i[0] for i in xvals]
    # y_points = [i[1] for i in yvals]
    try:
        z = np.polynomial.polynomial.Polynomial.fit(xvals, yvals, 2)
        z_deriv = z.deriv()
        test_x_right = xvals.copy()
        test_y_right = z(test_x_right).astype("int32")
        test_z_deriv = z_deriv(test_x_right)
        
        test_x_left = (test_x_right + 475*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_left = (test_y_right - 475/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
                
        test_x_middle = (test_x_right + 230*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_middle = (test_y_right - 230/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
                
        x_right = []
        y_right = []
        x_left = []
        y_left = []
        x_middle = []
        y_middle = []
        
        for i in range(len(test_x_right)):
            if(test_x_right[i] >= 0 and test_x_right[i] < img.shape[0] and test_y_right[i] >= 0 and test_y_right[i] < img.shape[1]):
                x_right.append(test_x_right[i])
                y_right.append(test_y_right[i])
            
            if(test_x_left[i] >= 0 and test_x_left[i] < img.shape[0] and test_y_left[i] >= 0 and test_y_left[i] < img.shape[1]):
                x_left.append(test_x_left[i])
                y_left.append(test_y_left[i])
            
            if(test_x_middle[i] >= 0 and test_x_middle[i] < img.shape[0] and test_y_middle[i] >= 0 and test_y_middle[i] < img.shape[1]):
                x_middle.append(test_x_middle[i])
                y_middle.append(test_y_middle[i])
                
        return y_left, x_left, y_middle, x_middle, y_right, x_right
    except:
        return [], [], [],[],[], [],

def polyfit_lane_left(xvals, yvals, img):
    try:
        z = np.polynomial.polynomial.Polynomial.fit(xvals, yvals, 2)
        z_deriv = z.deriv()
        test_x_right = xvals.copy()
        test_y_right = z(test_x_right).astype("int32")
        test_z_deriv = z_deriv(test_x_right)
        
        test_x_left = (test_x_right - 475*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_left = (test_y_right + 475/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
                
        test_x_middle = (test_x_right - 230*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_middle = (test_y_right + 230/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        x_right = []
        y_right = []
        x_left = []
        y_left = []
        x_middle = []
        y_middle = []
        
        for i in range(len(test_x_right)):
            if(test_x_right[i] >= 0 and test_x_right[i] < img.shape[0] and test_y_right[i] >= 0 and test_y_right[i] < img.shape[1]):
                x_right.append(test_x_right[i])
                y_right.append(test_y_right[i])
            
            if(test_x_left[i] >= 0 and test_x_left[i] < img.shape[0] and test_y_left[i] >= 0 and test_y_left[i] < img.shape[1]):
                x_left.append(test_x_left[i])
                y_left.append(test_y_left[i])
            
            if(test_x_middle[i] >= 0 and test_x_middle[i] < img.shape[0] and test_y_middle[i] >= 0 and test_y_middle[i] < img.shape[1]):
                x_middle.append(test_x_middle[i])
                y_middle.append(test_y_middle[i])
                
        return y_right, x_right, y_middle, x_middle, y_left, x_left
    except:
        return [], [], [],[],[], []
class Cluster:
    def __init__(self):
        self.coordinates = set()
    
    def add_points(self, img, x0, y0):
        if not ((0 <= x0 < img.shape[0]) and (0 <= y0 < img.shape[1])):
            return
        if img[x0][y0] == 0:
            return
        if((x0, y0) in self.coordinates):
            return
        img[x0][y0] = 0
        self.coordinates.add((x0, y0))
        self.add_points(img, x0 + 1, y0)
        self.add_points(img, x0 - 1, y0)
        self.add_points(img, x0, y0 + 1)
        self.add_points(img, x0, y0 - 1)
    
    def polyfit_lane_right(self, img):
        x_points = [i[0] for i in self.coordinates]
        y_points = [i[1] for i in self.coordinates]
        
        z = np.polynomial.polynomial.Polynomial.fit(x_points, y_points, 3)
        z_deriv = z.deriv()
        test_x_right = np.arange(-2*img.shape[0], 2*img.shape[0])
        test_y_right = z(test_x_right).astype("int32")
        test_z_deriv = z_deriv(test_x_right)
        
        test_x_left = (test_x_right + 475*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_left = (test_y_right - 475/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
                
        test_x_middle = (test_x_right + 230*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_middle = (test_y_right - 230/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
                
        x_right = []
        y_right = []
        x_left = []
        y_left = []
        x_middle = []
        y_middle = []
        
        for i in range(len(test_x_right)):
            if(test_x_right[i] >= 150 and test_x_right[i] < img.shape[0] and test_y_right[i] >= 0 and test_y_right[i] < img.shape[1]):
                x_right.append(test_x_right[i] - 150)
                y_right.append(test_y_right[i])
            
            if(test_x_left[i] >= 150 and test_x_left[i] < img.shape[0] and test_y_left[i] >= 0 and test_y_left[i] < img.shape[1]):
                x_left.append(test_x_left[i] - 150)
                y_left.append(test_y_left[i])
            
            if(test_x_middle[i] >= 150 and test_x_middle[i] < img.shape[0] and test_y_middle[i] >= 0 and test_y_middle[i] < img.shape[1]):
                x_middle.append(test_x_middle[i] - 150)
                y_middle.append(test_y_middle[i])
                
        return y_left, x_left, y_middle, x_middle, y_right, x_right
    
    def polyfit_lane_left(self, img):
        x_points = [i[0] for i in self.coordinates]
        y_points = [i[1] for i in self.coordinates]
        
        z = np.polynomial.polynomial.Polynomial.fit(x_points, y_points, 3)
        z_deriv = z.deriv()
        test_x_right = np.arange(-2*img.shape[0], 2*img.shape[0])
        test_y_right = z(test_x_right).astype("int32")
        test_z_deriv = z_deriv(test_x_right)
        
        test_x_left = (test_x_right - 475*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_left = (test_y_right + 475/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
                
        test_x_middle = (test_x_right - 230*test_z_deriv / np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        test_y_middle = (test_y_right + 230/np.sqrt(1+np.square(test_z_deriv))).astype("int32")
        
        x_right = []
        y_right = []
        x_left = []
        y_left = []
        x_middle = []
        y_middle = []
        
        for i in range(len(test_x_right)):
            if(test_x_right[i] >= 150 and test_x_right[i] < img.shape[0] and test_y_right[i] >= 0 and test_y_right[i] < img.shape[1]):
                x_right.append(test_x_right[i] - 150)
                y_right.append(test_y_right[i])
            
            if(test_x_left[i] >= 150 and test_x_left[i] < img.shape[0] and test_y_left[i] >= 0 and test_y_left[i] < img.shape[1]):
                x_left.append(test_x_left[i] - 150)
                y_left.append(test_y_left[i])
            
            if(test_x_middle[i] >= 150 and test_x_middle[i] < img.shape[0] and test_y_middle[i] >= 0 and test_y_middle[i] < img.shape[1]):
                x_middle.append(test_x_middle[i] - 150)
                y_middle.append(test_y_middle[i])
                
        return y_right, x_right, y_middle, x_middle, y_left, x_left

# from stopline_det import lane_without_stoplines

import warnings
warnings.filterwarnings("ignore")
LOWER_THRESHOLD = 180
thresh1 = 125
thresh2 = 145
thresh3 = 202
thresh4=217
thresh5 = 230
thresh6=250

def first_nonzero(arr, axis, invalid_val=-1):
    mask = arr!=0
    return np.where(mask.any(axis=axis), mask.argmax(axis=axis), invalid_val)

class LaneDetector(Thread):

    def __init__(self):
        self.frame_count = 0
        front_camera_topic = '/camera_front/image_raw'
        left_camera_topic = '/camera_front/image_raw'
        right_camera_topic = '/camera_front/image_raw'
        occgrid_topic = '/cv/laneoccgrid'
        task_label_topic = '/dm/task' # DM needs to publish this topic continuously about which task is in progress
        camera_label_topic = '/dm/camera' # DM needs to publish this topic continuously about which camera to use
        cv_output_topic = '/cv/output'

        # to sync the input from all 3 cameras
        front_camera_subscriber = message_filters.Subscriber(front_camera_topic,Image, queue_size = 1)
        left_camera_subscriber = message_filters.Subscriber(left_camera_topic,Image, queue_size = 1)
        #left_camera_subscriber = message_filters.Subscriber(left_camera_topic,Image, queue_size = 2)
        right_camera_subscriber = message_filters.Subscriber(right_camera_topic, Image, queue_size = 1)
        ts = message_filters.TimeSynchronizer([front_camera_subscriber, left_camera_subscriber, right_camera_subscriber], 10)
        ts.registerCallback(self.camera_data_callback) 

        task_topic_subscriber = rospy.Subscriber(task_label_topic, Task, self.task_label_callback)
        camera_topic_subscriber = rospy.Subscriber(camera_label_topic, Task, self.camera_label_callback)
        
        self.front_image = None
        self.left_image = None
        self.right_image = None

        # represents which road format is being used
        self.task_list = ['straight', 'straight-right-lanechange', 'straight-left-lanechange', 'curve-right', 'curve-right-lanechange',  'curve-left', 'curve-left-lanechange', 'intersection-right', 'intersection-left', 'intersection-straight']
        self.task = 'straight'
        # represents which camera needs to be used 
        self.camera_list = {'left' : 0, 'front' : 1, 'right' : 2} 
        self.camera = 1 # Set this to None when concerned topic has been setup concerned camera

        # camera image dimensions
        self.camera_image_width  = 1280
        self.camera_image_height = 720

        # occupancy grid initialization
        self.occupancy_grid_message = OccupancyGrid()
        self.occupancy_grid_message.header.frame_id = 'occgrid'
        self.occupancy_grid_message.info.resolution = 0.01  # 1cm per pixel
        self.occupancy_grid_message.info.width  = 1280
        self.occupancy_grid_message.info.height = 720
        self.occupancy_grid_message.info.origin.position.x = 0
        self.occupancy_grid_message.info.origin.position.y = 0
        self.occupancy_grid_message.info.origin.position.z = 0
        self.occupancy_grid_message.info.origin.orientation.x = 0
        self.occupancy_grid_message.info.origin.orientation.y = 0
        self.occupancy_grid_message.info.origin.orientation.z = -0.707
        self.occupancy_grid_message.info.origin.orientation.w = 0.707

        self.current_lane_store = 2

        self.cv_occupancy_grid_publisher = rospy.Publisher(occgrid_topic, OccupancyGrid, queue_size = 1)
        self.cv_lane_publisher = rospy.Publisher(cv_output_topic, CV, queue_size = 1)
        self.current_lane_publisher = rospy.Publisher('/cv/current_lane', Int32, queue_size = 5)
        self.rate = rospy.Rate(10)

#-----------------------------------------------------
# ROS related utils

    def task_label_callback(self, task):
        try:
            assert task.name in self.task_list
            print('got task')
            self.task = task.name
            print(self.task)
        except AssertionError as msg:
            print(msg)
    
    def camera_label_callback(self, camera):
        try:
            assert camera.name in self.camera_list
            self.camera = self.camera_list[camera.name]
            print(self.camera)
        except AssertionError as msg:
            print(msg)

    def camera_data_callback(self,front_img_msg,left_img_msg,right_img_msg):
        if (self.frame_count !=4):
            self.frame_count+=1
            return
        self.frame_count = 0
        bridge = CvBridge()
        f = open('time_save.txt', 'a')
        f.write(str(time.time()-front_img_msg.header.stamp.secs)+'\n')
        
        try:
            front_img = bridge.imgmsg_to_cv2(front_img_msg, '8UC3')
            left_img = bridge.imgmsg_to_cv2(left_img_msg, '8UC3')
            right_img = bridge.imgmsg_to_cv2(right_img_msg, '8UC3')
            self.front_image = front_img
            self.left_image = left_img
            self.right_image = right_img
            self.run_algo()
        except CvBridgeError as msg:
            print(msg)
            return 

#------------------------------------------------------
# handling intersection cases

    def get_reference_lanes_intersection2(self, image, camera = None):
        # inherently assumes that when left  lane is being extracted when road is turning left  not the vehicle
        # inherently assumes that when right lane is being extracted when road is turning right not the vehicle
        camera = self.camera if camera is None else camera
        (h, w) = image.shape[:2]

        x, y = [], []

        prev = 0 
        if camera==0: 
            for i in range(w): 
                for j in range(h-1, -1, -1): 
                    if image[j][i]>0 and j>prev: 
                        x.append(i) 
                        y.append(j) 
                        prev = j 
                        break 
        elif camera==2:
            for i in range(w-1, -1, -1):
                for j in range(h-1, -1, -1):
                    if image[j][i]>0 and j>prev:
                        x.append(i)
                        y.append(j)
                        prev = j
                        break 
            x.reverse()
            y.reverse()
        print('types:', type(x), type(y))
        return x, y    

    def poly_intersection(self, image):
        reference_x, reference_y = self.reference_lanes(image)
        ref_poly = np.poly1d(np.polyfit(reference_x, reference_y, 3))
        ref_poly_der = np.poly1d(np.polyder(ref_poly, m = 1))

        minx, maxx = min(reference_x), max(reference_x)
        x0 = np.linspace(minx, maxx, 3*(maxx-minx))
        y0 = ref_poly(x0)
        y0 = [max(-20, min(y, 800)) for y in y0]

        x1, y1, x2, y2 = [], [], [], []
        D = 250 
        if self.camera==2:
            D = -250

        for i in range(len(x0)):
            m = -1.0/ref_poly_der(x0[i])
            y1p = y0[i]+D*m*1.0/math.sqrt(1+m**2)
            y2p = y0[i]+D*m*2.0/math.sqrt(1+m**2)
            if y1p<-20 or y1p>800:
                continue
            elif y2p<-20 or y2p>800:
                x1.append(x0[i]+D*1.0/math.sqrt(1+m**2))
                y1.append(y1p)
                continue 
            else:
                x1.append(x0[i]+D*1.0/math.sqrt(1+m**2))
                y1.append(y1p)
                x2.append(x0[i]+D*2.0/math.sqrt(1+m**2))
                y2.append(y2p)       

        return (x0, y0, x1, y1, x2, y2)

    def generate_all_lanes_intersection(self, h, w, x, y, camera = None):
        camera = self.camera if camera is None else camera
        
        D = int(243//res) # 8ft is approx 300 pixels by measurement
        H = int(5//res) # hyperparameter of distance from first point to second point and distance is defined in terms of number of points in between

        if camera==0:
            x3, y3 = x[np.argmax(y)]   , y[np.argmax(y)]
            x1, y1 = x[np.argmin(x)]   , y[np.argmin(x)]
            H = abs(np.argmax(y) - np.argmin(x))//3
            x2, y2 = x[np.argmin(x)+H] , y[np.argmin(x)+H] 

            H = int(2//res)
            while not(min(x1, x3)<x2 and x2<max(x1, x3) and min(y1, y3)<y2 and y2<max(y1, y3)):
                if np.argmin(x)+H>len(x):
                    if (len(x)> np.argmin(x)+ 5//res):
                        x2, y2 = x[np.argmin(x)+int(5//res)] , y[np.argmin(x)+int(5//res)]
                        break 
                x2, y2 = x[np.argmin(x)+H], y[np.argmin(x)+H]
                H += 1
            
            # image = cv2.line(np.zeros((h,w,3)), (x1, y1), (x2, y2), (255,255,255))
            # image = cv2.circle(image, (x3, y3), 5, (0,0,255), -1)
            # cv2.imshow("Image", image)
            # cv2.waitKey(0)

            m  = (y2-y1)*1.0/(x2-x1+1e-5) 
            c  = y1-x1*m 
            c1 = c + D*math.sqrt(1+m**2)

            d = min(abs(m*x3+c1-y3)*1.0/math.sqrt(1+m**2), D)
            r = math.sqrt(D**2 - d**2)


            x4, y4 = (m*y3+x3-m*c1)*1.0/(m**2+1), (m*m*y3+m*x3+c1)*1.0/(m**2+1)
            xc, yc = int(x4-r*1.0/math.sqrt(1+m**2)), int(y4-r*m*1.0/math.sqrt(1+m**2))
            x2, y2 = int((m*yc+xc-m*c)*1.0/(m**2+1)), int((m*m*yc+m*xc+c)*1.0/(m**2+1))

            # image = cv2.line(np.zeros((h,w,3)), (0, int(c1)), (int(x4), int(y4)), (255,255,255))
            # image = cv2.circle(image, (x1, y1), 5, (255,255,255), -1)
            # image = cv2.circle(image, (x2, y2), 5, (255,255,255), -1)
            # image = cv2.circle(image, (x3, y3), 5, (255,255,255), -1)
            # image = cv2.circle(image, (int(x4), int(y4)), 5, (255,255,255), -1)
            # image = cv2.circle(image, (xc, yc), 5, (0,0,255), -1)
            # cv2.imshow("Test Image", image)
            # cv2.waitKey(0)

            startAngle = 360-abs(math.atan((yc-y2)/(x2-xc+1e-5))*180/math.pi)
            
            image_left = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            image_left = cv2.ellipse(image_left, (xc, yc), (D, D), 0., startAngle, startAngle+90, (255,255,255))

            x1, y1 =  x1    , int(y1 - D*math.sqrt(1+m*m))
            x2, y2 =  int(x2+D*m/math.sqrt(1+m**2)), int(y2-D/math.sqrt(1+m**2))

            image_middle = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            image_middle = cv2.ellipse(image_middle, (xc, yc), (2*D, 2*D), 0., startAngle, startAngle+90, (255,255,255))

            x1, y1 = x1    , int(y1 - D*math.sqrt(1+m*m))
            x2, y2 =  int(x2+D*m/math.sqrt(1+m**2 )), int(y2-D/math.sqrt(1+m**2))

            image_right = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            image_right = cv2.ellipse(image_right, (xc, yc), (3*D, 3*D), 0., startAngle, startAngle+90, (255,255,255))

        elif camera==2:
            x3, y3 = x[np.argmax(y)]   , y[np.argmax(y)]
            x1, y1 = x[np.argmax(x)]   , y[np.argmax(x)]
            H = abs(np.argmax(x)-np.argmax(y))//3
            x2, y2 = x[np.argmax(x)-H] , y[np.argmax(x)-H] 

            H = int(2//res)
            while not(x3<=x2 and x2<=x1 and y1<=y2 and y2<=y3):
                if np.argmax(x)-H<0:
                    x2, y2 = x[np.argmax(x)-int(20//res)], y[np.argmax(x)-int(20//res)]
                    break 
                x2, y2 = x[np.argmax(x)-H], y[np.argmax(x)-H]
                H += 1

            m  = (y2-y1)*1.0/(x2-x1+1e-5)
            c  = y1-x1*m 
            c1 = c + D*math.sqrt(1+m**2)

            image = cv2.line(np.zeros((h,w,3)), (x1, y1), (x2, y2), (255,255,255))
            image = cv2.circle(image, (x3, y3), 5, (0,0,255), -1)
            #cv2.imshow("Image", image)
            #cv2.waitKey(0)


            d = min(abs(m*x3+c1-y3)*1.0/math.sqrt(1+m**2), D)
            r = math.sqrt(D**2 - d**2)

            x4, y4 = (m*y3+x3-m*c1)*1.0/(m**2+1), (m*m*y3+m*x3+c1)*1.0/(m**2+1)
            xc, yc = int(x4+r*1.0/math.sqrt(1+m**2)), int(y4+r*m*1.0/math.sqrt(1+m**2))
            x2, y2 = int((m*yc+xc-m*c)*1.0/(m**2+1)), int((m*m*yc+m*xc+c)*1.0/(m**2+1))

            startAngle = 180+abs(math.atan((yc-y2)/(x2-xc+1e-5))*180/math.pi)

            image_right = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            image_right = cv2.ellipse(image_right, (xc, yc), (D, D), 0., startAngle-90, startAngle, (255,255,255))
            image_right = self.adjust_camera(image_right)

            x1, y1 =  x1    , int(y1 - D*math.sqrt(1+m*m))
            x2, y2 =  int(x2+D*m/math.sqrt(1+m**2)), int(y2-D/math.sqrt(1+m**2))

            image_middle = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            image_middle = cv2.ellipse(image_middle, (xc, yc), (2*D, 2*D), 0., startAngle-90, startAngle, (255,255,255))
            image_middle = self.adjust_camera(image_middle)

            x1, y1 = x1    , int(y1 - D*math.sqrt(1+m*m))
            x2, y2 =  int(x2+D*m/math.sqrt(1+m**2 )), int(y2-D/math.sqrt(1+m**2))

            image_left = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            image_left = cv2.ellipse(image_left, (xc, yc), (3*D, 3*D), 0., startAngle-90, startAngle, (255,255,255))
            image_left = self.adjust_camera(image_left)

        arr0 = np.where(image_left>0)
        arr1 = np.where(image_middle>0)
        arr2 = np.where(image_right>0)

        x0, y0 = arr0[1].tolist(), arr0[0].tolist()
        x1, y1 = arr1[1].tolist(), arr1[0].tolist()
        x2, y2 = arr2[1].tolist(), arr2[0].tolist()

        return x0, y0, x1, y1, x2, y2   
    

    def get_reference_lane_intersection(self, image, camera = None):
        # inherently assumes that when left  lane is being extracted when road is turning left  not the vehicle
        # inherently assumes that when right lane is being extracted when road is turning right not the vehicle
        camera = self.camera if camera is None else camera
        (h, w) = image.shape[:2]

        if camera==2:
            image = np.fliplr(image)

        x, y = [], []

        tempx = first_nonzero(image, axis = 1)
        if camera==0:
            for i in range(tempx.shape[0]):
                if tempx[i]!=-1:
                    x.append(tempx[i])
                    y.append(i)
        elif camera==2:
            for i in range(tempx.shape[0]):
                if tempx[i]!=-1:
                    x.append(1280-tempx[i])
                    y.append(i)
 
        x.reverse() # since we wish to have list element in bottoms up manner so that argmax functions give left lane pixels 
        y.reverse()

        return x, y

    def intersection_lane_detection(self, image):
        print('Handling Intersection')
        image  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert image to grayscale
        image  = cv2.resize(image, (resx, resy), interpolation = cv2.INTER_AREA)[:680, :] # remove the bottom portion of image
        warped_image = self.do_ipm(image) # get bird's eye view of image 
        thresholded_image = 255*(warped_image > LOWER_THRESHOLD).astype('uint8') # binary threshold by 180
        eroded_image = cv2.erode(thresholded_image, np.ones((5,5), np.uint8), iterations = 2) # erosion done to remove small white noise bubbles
        eroded_image[680:, :] = 0
        #if self.camera!=1:
        eroded_image = self.adjust_camera(eroded_image) # seperate case handling for right turn at intersection
        cv2.imwrite('eroded_image_6py.png', eroded_image)
        (h,w) = eroded_image.shape[:2]
        #print(eroded_image.shape)
        (x0, y0, x1, y1, x2, y2) = None, None, None, None, None, None
        #a = time.time()
        if self.camera==0:
            #reference_x, reference_y = self.get_reference_lanes_intersection2(eroded_image)
            #reference_x, reference_y = self.ref_lane_gen(eroded_image)
            #(x0, y0, x1, y1, x2, y2) = self.generate_all_lanes_intersection(h, w, reference_x, reference_y)
            a = time.time()
            left_lane_points = left_ref_lane(eroded_image)
            yvals, xvals = bezier_curve(left_lane_points, nTimes=1000)
            x0, y0, x1, y1, x2, y2 = polyfit_lane_left(yvals, xvals, eroded_image)
            print(time.time()-a)

        elif self.camera == 2:
            a = time.time()
            right_lane_points = right_ref_lane(eroded_image)
            yvals, xvals = bezier_curve(right_lane_points, nTimes=1000)
            x0, y0, x1, y1, x2, y2 = polyfit_lane_right(yvals, xvals, eroded_image)
            print(time.time()-a)

        else:
            eroded_image[50:720, :] = 0
            peak = np.mean(eroded_image, axis = 0)
            pointx1 = None; pointx3 = None
            for i in range(1280):
                if eroded_image[0][i]>0:
                    pointx1 = i
                    break
            pointx2 = pointx1 + 243
            pointx3 = pointx2 + 243
            x0 = [pointx1 for i in range(h)]
            y0 = [i-150 for i in range(h)]
            x1 = [pointx2 for i in range(h)]
            y1 = y0
            x2 = [pointx3 for i in range(h)]
            y2 = y0

        current_lane = self.current_lane(x1, y1)
        #b = time.time()
        #print("t7 = ",b-a)
        return (x0, y0, x1, y1, x2, y2, current_lane)
#-----------------------------------------------------
# handling straight lanes

    def get_reference_lane_straight(self, image, camera = None):
        # inherently assumes that when left  lane is being extracted when road is turning left  not the vehicle
        # inherently assumes that when right lane is being extracted when road is turning right not the vehicle
        camera = self.camera if camera is None else camera
        (h, w) = image.shape[:2]

        x, y = [], []

        if self.task.find('left')!=-1: #changin to left lane
            refx = first_nonzero(image, axis = 1)
            for i in range(refx.shape[0]):
                if refx[i]!=-1:
                    x.append(refx[i])
                    y.append(i)
            x.reverse() # since we wish to have list element in bottoms up manner so that argmax functions give left lane pixels 
            y.reverse()
        else: #changing to right lane or completely straight
            image = np.fliplr(image)
            refx = first_nonzero(image, axis = 1)
            for i in range(refx.shape[0]):
                if refx[i]!=-1:
                    x.append(resx - refx[i])
                    y.append(i)
            x.reverse() # since we wish to have list element in bottoms up manner so that argmax functions give left lane pixels 
            y.reverse()            
        print("Types:", type(x), type(y))
        return x, y

    def generate_all_lanes_straight(self, h, w, x, y, camera = None):
        camera = self.camera if camera is None else camera
        
        D = 243 # 8ft is approx 243 pixels by measurements
        
        if self.task.find('left')!=-1: #changing to left lane
            x2, y2 = x[np.argmax(y)]   , y[np.argmax(y)]
            x1, y1 = x[np.argmin(y)]   , y[np.argmin(y)]

            m  = (y2-y1)*1.0/(x2-x1+1e-5)
            c  = y1-x1*m 
            xShift = D*math.sqrt(1+m*m)/abs(m)

            image_left = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))

            x1, y1 = int(x1 + xShift), y1
            x2, y2 = int(x2 + xShift), y2

            image_middle = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))

            x1, y1 = int(x1 + xShift), y1
            x2, y2 = int(x2 + xShift), y2

            image_right = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))

        elif self.task.find('right')!=-1:
            x2, y2 = x[np.argmax(y)]   , y[np.argmax(y)]
            x1, y1 = x[np.argmin(y)]   , y[np.argmin(y)]

            m  = (y2-y1)*1.0/(x2-x1)
            m = min(100, m)
            c  = y1-x1*m 
            xShift = D*math.sqrt(1+m*m)/abs(m)

            image_right = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            
            print(x1, xShift)
            x1, y1 = int(x1 - xShift), y1
            x2, y2 = int(x2 - xShift), y2
            
            image_middle = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))

            x1, y1 = int(x1 - xShift), y1
            x2, y2 = int(x2 - xShift), y2

            image_left = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
        
        else:
            x2, y2 = int(np.mean(x)), h
            x1, y1 =              x2, 0

            m  = (y2-y1)*1.0/(x2-x1+1e-5)
            c  = y1-x1*m 
            xShift = D*math.sqrt(1+m*m)/m

            image_right = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
        
            x1, y1 = int(x1 - xShift), y1
            x2, y2 = int(x2 - xShift), y2
            
            image_middle = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))

            x1, y1 = int(x1 - xShift), y1
            x2, y2 = int(x2 - xShift), y2

            image_left = cv2.line(np.zeros((h,w)), (x1, y1), (x2, y2), (255,255,255))
            

        arr0 = np.where(image_left>0)
        arr1 = np.where(image_middle>0)
        arr2 = np.where(image_right>0)

        x0, y0 = arr0[1].tolist(), arr0[0].tolist()
        x1, y1 = arr1[1].tolist(), arr1[0].tolist()
        x2, y2 = arr2[1].tolist(), arr2[0].tolist()

        lane = self.current_lane(x1, y1)

        return x0, y0, x1, y1, x2, y2, lane            

    def straight_lane_detection(self, image):
        print('Handling Straight Lane')

        # x1, y1, x2, y2, _ = lane_without_stoplines(np.copy(image)) # 2 endpoints of the stopline

        # image[:y1,:] = 0 # region above stopline made empty as it would introduce bias in straight lane prediction
        image  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert image to grayscale
        image  = cv2.resize(image, (resx, resy), interpolation = cv2.INTER_AREA) # remove the bottom portion of image
        warped_image = self.do_ipm(image) # get bird's eye view of image
        cv2.imwrite('ipm_img.png', warped_image)
        thresholded_image = 255*(((warped_image > thresh1) & (warped_image < thresh2)) | ((warped_image > thresh3) & (warped_image < thresh4)) | ((warped_image > thresh5) & (warped_image < thresh6))).astype('uint8') # binary threshold by 180
        cv2.imwrite('thresh_img.png', thresholded_image)
        eroded_image = cv2.erode(thresholded_image, np.ones((5,5), np.uint8), iterations = 2) # erosion done to remove small white noise bubbles
        cv2.imwrite('eroded_img.png', eroded_image)
        eroded_image = self.adjust_camera(eroded_image)
        (h,w) = eroded_image.shape[:2]

        #cv2.imshow("Image_Original", eroded_image)
        #cv2.waitKey(0)

        reference_x, reference_y = self.get_reference_lane_straight(thresholded_image)
        x0, y0, x1, y1, x2, y2, current_lane = self.generate_all_lanes_straight(h, w, reference_x, reference_y)

        return (x0, y0, x1, y1, x2, y2, current_lane)

#------------------------------------------------------
# handling curved lanes
    
    def generate_all_lanes_curved(self, image, x0, x1, x2):        
        R = 609 # 250 pixels for 8ft(empirical result) thus 600 pixels for 20ft Radius of curve radius
        W = 243 # lane width in pixel

        (h,w) = image.shape

        if self.task[0:11]=='curve-right':
            image_left = cv2.circle(np.zeros((2*h,w,3)), (int(x0)+R+2*W, 720), R+2*W, (255,255,255))
            image_middle = cv2.circle(np.zeros((2*h,w,3)), (int(x1)+R+W  , 720), R+W  , (255,255,255))
            image_right = cv2.circle(np.zeros((2*h,w,3)), (int(x2)+R    , 720), R    , (255,255,255))
        elif self.task[0:10]=='curve-left':
            image_left = cv2.circle(np.zeros((2*h,w,3)), (int(x0)-R    , 720), R    , (255,255,255))
            image_middle = cv2.circle(np.zeros((2*h,w,3)), (int(x1)-R-W  , 720), R+W  , (255,255,255))
            image_right = cv2.circle(np.zeros((2*h,w,3)), (int(x2)-R-2*W, 720), R+2*W, (255,255,255))
        
        image_left = self.adjust_camera(image_left)[:720,:]
        image_middle = self.adjust_camera(image_middle)[:720,:]
        image_right = self.adjust_camera(image_right)[:720,:]

        lane = self.current_lane(np.where(image_middle>0)[1].tolist(), np.where(image_middle>0)[0].tolist())

        x0, y0 = np.where(image_left>0)[1].tolist(), np.where(image_left>0)[0].tolist()
        x1, y1 = np.where(image_middle>0)[1].tolist(), np.where(image_middle>0)[0].tolist()
        x2, y2 = np.where(image_right>0)[1].tolist(), np.where(image_right>0)[0].tolist()

        return x0, y0, x1, y1, x2, y2, lane
    
    def get_lanes_curved(self, img):
        # (h, w) = image.shape

        # H = 50 # hyperparameter of distance from bottom of iamge at which we measure x coordinate of all lanes
        # x0 = None
        # while x0 is None:
        #     for i in range(w):
        #         if image[h-H][i]>0:
        #             x0 = i 
        #             break
        #     H += 1
        #     if h-H<0:
        #         break

        # H = 50
        # x2 = None  
        # while x2 is None:
        #     for i in range(w-1,-1,-1):
        #         if image[h-H][i]>0:
        #             x2 = i 
        #             break
        #     H += 1
        #     if h-H<0:
        #         break
        
        # try:
        #     assert x0!=None or x2!=None
            
        #     W = 243 # lane width in pixels i.e 8ft is 243 pixels
        #     if x0==None:
        #         x0 = x2 - 2*W 
        #     elif x2==None:
        #         x2 = x0 + 2*W
                
        #     x1 = (x0+x2)*0.5 # middle lane's bottomost x-coordinate
            
        #     # if distance is less than 500 there has been minor inconsistency in lane width
        #     if abs(x0-x2)<2*W:
        #         if self.task[6:11]=='right': # left lane x coordinate is correct [IMPROVEMENT]
        #             x1 = x0 + W 
        #             x2 = x0 + 2*W
        #         elif self.task[6:10]=='left': # right labe x coordinate is correct [IMPROVEMENT]
        #             x1 = x2 - W 
        #             x0 = x2 - 2*W 

        #     x0, y0, x1, y1, x2, y2, current_lane = self.generate_all_lanes_curved(image, x0, x1, x2)

        #     return (x0, y0, x1, y1, x2, y2, current_lane)

        # except AssertionError as msg:
        #     print('Run Again. Curved Lane Finding algorithm couldn\'t find valid reference points')
        #     return None, None, None, None, None, None, None

        if self.task.find('left')!=-1: #Change to Flag for left turn
            for r in np.arange(0, img.shape[0], 5).tolist():
                stop = False
                for theta in np.arange(0, 90, 3).tolist():
                    x = img.shape[0] - r*np.cos(np.deg2rad(theta))
                    y = img.shape[1] - r*np.sin(np.deg2rad(theta))
                    if(x >= 0 and y >= 0 and x < img.shape[0] and y < img.shape[1]):
                        if img[int(x)][int(y)] == 255:
                            print(x, y)
                            right_lane = Cluster()
                            right_lane.add_points(img, int(x), int(y)) 
                            stop = True
                            break      
                if stop:
                    break
            return right_lane.polyfit_lane_right(img)
        
        else:
            for r in np.arange(0, img.shape[0], 5).tolist():
                stop = False
                for theta in np.arange(0, 90, 3).tolist():
                    x = img.shape[0] - r*np.cos(np.deg2rad(theta))
                    y = r*np.sin(np.deg2rad(theta))
                    if(x >= 0 and y >= 0 and x < img.shape[0] and y < img.shape[1]):
                        if img[int(x)][int(y)] == 255:
                            print(x, y)
                            left_lane = Cluster()
                            left_lane.add_points(img, int(x), int(y)) 
                            stop = True
                            break      
                if stop:
                    break
            return left_lane.polyfit_lane_left(img)
            
    
    def curve_lane_detection(self, image):
        print('Handling Curved Lane')
        
        image  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert image to grayscale
        image  = cv2.resize(image, (1280, 720), interpolation = cv2.INTER_AREA)[:650, :] # remove the bottom portion of image

        warped_image = self.do_ipm(image) # get bird's eye view of image
        thresholded_image = 255*(warped_image > LOWER_THRESHOLD).astype('uint8') # binary threshold by 180
        eroded_image = cv2.erode(thresholded_image, np.ones((5,5), np.uint8)) # erosion done to remove small white noise bubbles
        #cv2.imshow('Eroded Image', eroded_image)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        try: 
            x0, y0, x1, y1, x2, y2 = self.get_lanes_curved(eroded_image.copy()) 
            lane = self.current_lane(x1, y1)

        except:
            x0, y0, x1, y1, x2, y2 = [], [], [], [], [], []
            lane = self.current_lane_store
            print("Error in lane clustering")            

        return (x0, y0, x1, y1, x2, y2, lane)


#-------------------------------------------------------
# utils for lane detection

    def current_lane(self, x1, y1):
        try:
            vehicle_x = self.camera_image_width/2
            middle_lane_x = x1[np.argmax(y1)]

            if middle_lane_x<=vehicle_x:
                print('Lane : ', 2)
                self.current_lane_store = 2
                lane_msg = Int32()
                lane_msg.data = self.current_lane_store
                self.current_lane_publisher.publish(lane_msg)
                return 2
            else:
                print('Lane : ', 1)
                self.current_lane_store = 1
                lane_msg = Int32()
                lane_msg.data = self.current_lane_store
                self.current_lane_publisher.publish(lane_msg)
                return 1 
        except:
            lane_msg = Int32()
            lane_msg.data = self.current_lane_store
            self.current_lane_publisher.publish(lane_msg)

    def adjust_camera(self, image, angle = None, xShift = None):
        camera = self.camera
        (h, w) = image.shape[:2]
        
        camera_centre = (w//2, h+250)

        # rotational adjustment
        angleOfShift = 31.37 if angle is None else angle 
        angularShift = None
        if camera==0:
            angularShift = angleOfShift
        elif camera==2:
            angularShift = -angleOfShift
        elif camera==1:
            angularShift = 0
        M = cv2.getRotationMatrix2D(camera_centre, angularShift, 1.0)
        image = cv2.warpAffine(image, M, (w, h))

        # translation shift matrix (translating camera image to endpoint of car)
        xShift = 210 if xShift is None else xShift
        if camera==0:
            xShift = -xShift # 0.56 m is 56 cm which is approx 60 pixels by resolutions
        elif camera==2:
            xShift = xShift
        elif camera==1:
            xShift =  0
        xShift = 0
        yShift = -395 # 3.95m = 395cm which is 295 pixels by resolution (distance from camera blindspot distance)
        M = np.array([[1, 0, xShift], [0, 1, yShift], [0, 0, 1]], np.float32)
        image = cv2.warpPerspective(image, M, (w, h)) # translated image appropriately

        return image
    
    def get_color_image(self, x0, y0, x1, y1, x2, y2):
        new_img = np.zeros((self.camera_image_height, self.camera_image_width, 3))
        for i in range(len(x0)):
            new_img = cv2.circle(new_img, (int(x0[i]), int(y0[i])), 1, (0,0,255), -1)
        for i in range(len(x1)):
            new_img = cv2.circle(new_img, (int(x1[i]), int(y1[i])), 1, (0,255,0), -1)
        for i in range(len(x2)):
            new_img = cv2.circle(new_img, (int(x2[i]), int(y2[i])), 1, (255,0,0), -1)
        cv2.imwrite('lanes.png', new_img)
        return new_img

    def do_ipm(self, image, **kwargs):
            (h, w) = (image.shape[0], image.shape[1])
            ymax = 648
            x1 = w//2 - 364
            x2 = w//2 + 136
            l = 1200
            # Image coordinates after undistortion
            source = np.float32([[8., 594.], [994., 596.], [788., 470.], [380., 474.]])

            # Image coordinates without undistortion
            # source = np.float32([[19., 550.], [1004., 611.], [767., 436.], [374., 431.]])
            destination  = np.float32([[x1, ymax], [x2, ymax], [x2, ymax-l], [x1, ymax-l]])
            M = cv2.getPerspectiveTransform(source, destination)
            warped = cv2.warpPerspective(image, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            
            return warped
    
    def undistort(self, image, camera):
        camera_matrix = None
        dist_coeffs = None
        if camera==1: #camera1 parameters - Front Camera
            camera_matrix = np.array([[722.2992, 0,952.2574], [0, 722.1567, 539.8917], [0, 0, 1.000]])
            dist_coeffs = np.array([[-0.2058, 0.0562, 0., 0., 0.]])
        elif camera==0: #camera2 parameters - Left Camera
            camera_matrix = np.array([[841.6960, 0, 617.4311], [0, 840.6221, 388.3993], [0, 0, 1.000]])
            dist_coeffs = np.array([[-0.4017, 0.1730, 0., 0., 0.]])
        elif camera==2: #camera3 parameters - Right Camera
            camera_matrix = np.array([[874.9190, 0, 606.9020], [0, 868.3715, 370.6982], [0, 0, 1.000]])
            dist_coeffs = np.array([[-0.4519, 0.1926, 0., 0., 0.]])
        width, height = np.shape(image)[1], np.shape(image)[0] 
        newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width, height), 1, (width, height))
        undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs, None, newcameramatrix)
        return undistorted_image
 
 #----------------------------------------------------------

    def publish_CV_outputs(self, occupancy_grid, colored_image, x0, y0, x1, y1, x2, y2):
        occupancy_grid = occupancy_grid[:,:,0]+occupancy_grid[:,:,1]+occupancy_grid[:,:,2]
        occupancy_grid = cv2.resize(occupancy_grid, (1280, 720))
        occupancy_grid = cv2.flip(occupancy_grid, 0)
        #occupancy_grid = cv2.dilate(occupancy_grid, kernel = np.ones((5,5), np.uint8), iterations = 4)  
        occupancy_grid_int8 = np.array(occupancy_grid, dtype='int8')
        occupancy_grid_int8 = occupancy_grid_int8.flatten()

        occupancy_grid_int32 = np.array(occupancy_grid, dtype='int32')
        occupancy_grid_int32 = occupancy_grid_int32.flatten()

        lane_image_int32 = occupancy_grid_int32

        # Occupancy Grid Message
        self.occupancy_grid_message.info.map_load_time = rospy.Time.now()

        self.occupancy_grid_message.data = [value for value in np.nditer(occupancy_grid_int8)]
        self.cv_occupancy_grid_publisher.publish(self.occupancy_grid_message)

        x0 = np.array(x0, dtype = 'int32').flatten()
        x1 = np.array(x1, dtype = 'int32').flatten()
        x2 = np.array(x2, dtype = 'int32').flatten()
        y0 = np.array(y0, dtype = 'int32').flatten()
        y1 = np.array(y1, dtype = 'int32').flatten()
        y2 = np.array(y2, dtype = 'int32').flatten()

        output = CV()

        if (len(x0)>0):
            output.x0 = [value for value in np.nditer(x0)]
        if (len(x1)>0):
            output.x1 = [value for value in np.nditer(x1)]
        if (len(x2)>0):
            output.x2 = [value for value in np.nditer(x2)]
        if (len(y0)>0):
            output.y0 = [value for value in np.nditer(y0)]
        if (len(y1)>0):
            output.y1 = [value for value in np.nditer(y1)]
        if (len(y2)>0):
            output.y2 = [value for value in np.nditer(y2)]

        output.warped = [value for value in np.nditer(lane_image_int32)]
        output.warped_shape = [720, 1280]
        
        output.occGrid = [value for value in np.nditer(occupancy_grid_int32)]
        output.occGrid_shape = [720, 1280]

        self.cv_lane_publisher.publish(output)

    def run_algo(self): 
        f = open('timecheck.txt','a')
        f.write(str(time.time())+'\n')
        f.close()

        ## Get Camera feed ## 
        if self.camera == 0:
            image = self.left_image
            image = self.undistort(image, self.camera)
        elif self.camera == 2:
            image = self.right_image
            image = self.undistort(image, self.camera)
        elif self.camera == 1:
            image = self.front_image 
            image = self.undistort(image, self.camera)

        if self.task[:8] == 'straight':
            (x0, y0, x1, y1, x2, y2, current_lane) = self.straight_lane_detection(image)
        elif self.task[:12] == 'intersection':
            (x0, y0, x1, y1, x2, y2, current_lane) = self.intersection_lane_detection(image)
        elif self.task[:5] == 'curve':
            (x0, y0, x1, y1, x2, y2, current_lane) = self.curve_lane_detection(image)

        # x is rightwards from top left origin, y is downwards from top left origin
        if self.task[-10:] == 'lanechange': # removes middle lane
            x1 = []
            y1 = []

        colored_image = self.get_color_image(x0, y0, x1, y1, x2, y2)

        ## Get Lane Occupancy grid from birds eye view
        lane_occupancy_grid_image = colored_image

        ## Publish Lane Occupancy grid and CV outputs
        self.publish_CV_outputs(lane_occupancy_grid_image, colored_image, x0, y0, x1, y1, x2, y2)
        time.sleep(0.1)

if __name__ == '__main__':
    try:
        rospy.init_node('CV_Lane_Detection_Node')
        LaneDetectorObject1 = LaneDetector()
        #LaneDetectorObject2 = LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[INFO] Node Terminated.')
