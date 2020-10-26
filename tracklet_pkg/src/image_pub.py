#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import rospy
import numpy as np
import tf
import os
import cv2
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray

from cv_bridge import CvBridge, CvBridgeError

from tf import transformations

from message_filters import ApproximateTimeSynchronizer, Subscriber

import util
from bev_util import makeBEVMap, get_filtered_lidar
from lane_detection import lanesDetection
import ros_numpy

import scipy.misc

picto_velo = None
pub_velo = None

paths = []

def run():
    global pub_velo

    rospy.init_node('ego_publisher', anonymous=False)

    # rospy.Subscriber("/kitti/camera_color_left/image_raw", Image, callback, queue_size=1)
    # rospy.Subscriber("/bboxs", BoundingBoxArray, callback_2, queue_size=1)
    rospy.Subscriber("/paths", MarkerArray, callback_path, queue_size=1)

    ts = ApproximateTimeSynchronizer([Subscriber("/kitti/velo/pointcloud", PointCloud2),
                                      Subscriber("/kitti/camera_color_left/image_raw", Image),
                                      Subscriber("/kitti/camera_color_left/camera_info", CameraInfo),
                                      Subscriber("/kitti/oxts/gps/vel", TwistStamped),
                                      Subscriber("/bboxs", BoundingBoxArray)], 1, 0.5)

    ts.registerCallback(callback)

    pub_velo = rospy.Publisher("driver_view", Image, queue_size=1)

    rospy.spin()

def callback_path(data):
    global paths

    path_len = len(data.markers)

    # print(path_len)
    if path_len > 0:
        for _ in range(path_len):
            paths.append(data.markers.pop())
    # print(paths)

def callback(pc, image, camera_info, vel, bbox):
    header = image.header
    frame = header.seq

    bridge = CvBridge()
    # print(camera_info)
    # print(bbox)

    calib = camera_info
    # print(calib)
    velo = vel.twist.linear.x

    pc_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc)
    pc_filtered = get_filtered_lidar(pc_array)

    try:
        cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

        image = lanesDetection(cv_image)
        image = draw_at_driver_view(image.copy())
        image = draw_centerfascia(image.copy(), pc_filtered)
        show_rgb_image_with_boxes(image, bbox, calib, velo)

    except CvBridgeError as e:
        print(e)

    # cv2.imshow("Image window", merge)
    # cv2.waitKey()

    try:
        mrg_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        pub_velo.publish(mrg_msg)
        
    except CvBridgeError as e:
        print(e)

def put_velocity_to_image(image, velo):

    vel = round((velo * 3.6), 1)

    cv2.putText(image, str(vel) + ' Km/h',(280, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255, 255, 255), 2, cv2.LINE_AA)

def distance_between_cars(point):
    distance = None

    x = point[0]
    y = point[1]

    distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

    return distance

def draw_at_driver_view(cv_image):
    dashboard = cv2.imread('/home/mds/catkin_ws/src/tracklet_pkg/dashboard.png', 1)
    dash_row, dash_col, dash_ch = dashboard.shape

    row_1, col_1, ch_1 = cv_image.shape
    # print(cv_image.shape)

    dashboard_resize = cv2.resize(dashboard, dsize=(col_1,row_1+175))
    blank_img = np.zeros((row_1 - 100, col_1, 3), np.uint8)
          
    merge = cv2.vconcat([cv_image, blank_img])
    merge_row, merge_col, merge_ch = merge.shape
    # print(merge.shape)

    roi = merge[100:, :, :]

    img2gray = cv2.cvtColor(dashboard_resize, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(img2gray, 1, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)

    merge_bg = cv2.bitwise_and(roi, roi, mask = mask_inv)
    merge_fg = cv2.bitwise_and(dashboard_resize, dashboard_resize, mask=mask)

    dst = cv2.add(merge_bg, merge_fg)
    merge[100:, :] = dst

    return merge

def draw_centerfascia(image, pc):

    BEV_map = makeBEVMap(pc)
    BEV_map = (BEV_map.transpose(1, 2, 0) * 255).astype(np.uint8)
    BEV_map = cv2.rotate(BEV_map, cv2.ROTATE_180)
    BEV_map = BEV_map[: , 120 : 480]
    BEV_map = cv2.resize(BEV_map, (267, 270))

    image[345:615, 488:755] = BEV_map

    # cv2.imshow("show", BEV_map)
    # cv2.waitKey(100)

    return image


def put_warning_sign_to_image(image, x, y):

    warning_sign = cv2.imread('/home/mds/catkin_ws/src/tracklet_pkg/warning.png', 1)

    warning_sign_resize = cv2.resize(warning_sign, dsize=(100, 100))

    shape = image.shape

    height = shape[0]
    width = shape[1]

    alpha = 0.2
    
    if x > width * 2/3: # 오른쪽
        added = cv2.addWeighted(image[0:100, width-100:width, :], alpha, warning_sign_resize, 1 - alpha, 0)
        image[0:100, width-100:width] = added
    elif width * 1/3 < x < width * 2/3: # 중앙
        added = cv2.addWeighted(image[0: 100, width/2 -50 : width/2 +50, :], alpha, warning_sign_resize, 1 - alpha, 0)
        image[0: 100, width/2 -50 : width/2 +50] = added
    elif width * 1/3 > x: # 왼쪽
        added = cv2.addWeighted(image[0:100, 0:100, :], alpha, warning_sign_resize, 1 - alpha, 0)
        image[0:100, 0:100] = added

    return image

def check_range_of_predicted_path(safty_range):
    minx = -5
    maxx = safty_range
    miny = -5
    maxy = 5

    warning_car = []

    paths_len = len(paths)
    if paths_len:
        for _ in range(paths_len):
            path = paths.pop()
            point_len = len(path.points)
            if point_len:
                for _ in range(point_len):
                    point = path.points.pop()
                    # print(point)
                    point_x, point_y, point_z = point.x, point.y, point.z

                    if (minx <= point_x <= maxx) and (miny <= point_y <= maxy):
                        warning_car.append(path.id)

    return warning_car

def show_rgb_image_with_boxes(image, bbox, calib, velo):
    warn_id = []
    a = np.empty((8, 2), dtype=np.int32)

    put_velocity_to_image(image, velo)

    if len(bbox.boxes) > 0:
        for i in range(len(bbox.boxes)):
            box = bbox.boxes.pop()

            cls_id = 1
            location = np.array([box.pose.position.x, box.pose.position.y, box.pose.position.z, 1])
            dim = [box.dimensions.x, box.dimensions.y, box.dimensions.z]
            angles = tf.transformations.euler_from_quaternion([box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w])
            # print(angles)
            ry = angles[2]

            pos = util.velo3d_2_camera2d_points(location)
            # print(pos)
            x, y = pos[0], pos[1]

            loc = location[:-1]

            corners_3d = util.compute_box_3d(dim, loc, ry)
            # print(corners_3d)
            pts_3d_homo = np.concatenate([corners_3d, np.ones((corners_3d.shape[0], 1), dtype=np.float32)], axis=1)
            # print(pts_3d_homo)
            k = len(pts_3d_homo)
            
            for i in range(k):
                cor = util.velo3d_2_camera2d_points(pts_3d_homo[i])
                if cor.size:
                    # print(cor)
                    a[i] = [cor[0], cor[1]]

            # print(a)

            distance = round(distance_between_cars((location[0], location[1])), 1)

            safe_range = stopping_distance(velo)
            warning_car_id = check_range_of_predicted_path(safe_range)
            
            if warning_car_id:
                warn_id = warning_car_id.pop()

            # print(warn_id, box.label)

            if x > 0 and y > 0:
                if warn_id == box.label:
                    # image = cv2.circle(image, (x, y), 1, (0,0,255), 5)
                    util.draw_box_3d(image, a, color=(0, 0, 255))
                    put_warning_sign_to_image(image, x, y)
                    if distance:
                        image = cv2.putText(image, str(distance) + ' m',(x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255, 255, 255), 2, cv2.LINE_AA)
                else :
                    # image = cv2.circle(image, (x, y), 1, (255,0,0), 5)
                    util.draw_box_3d(image, a, color=(255, 0, 0))
                    if distance:
                        image = cv2.putText(image, str(distance) + ' m',(x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255, 255, 255), 2, cv2.LINE_AA)

    return image

def stopping_distance(v) :
    g, mu, t_act, l =  9.81, 0.7, 1, 3
    v_m = v / 3.6
    # print(str(v_m) +'m/s')
    # 공주거리(t_d) thinking distance
    t_d = round(v_m * t_act, 4)
    # print('t_d : '+ str(t_d) + 'm')
    # 제동거리(b_d) braking distance
    b_d = round((v_m**2) / (2*g*mu), 4)
    # print('b_d : ' + str(b_d) + 'm')
    # 정지거리(s_d) stopping distance  = t_d + b_d
    s_d = round(t_d + b_d, 4)
    # print('s_d : ' + str(s_d) + 'm')
    # 안전거리(safe_d) safe distance = 정지거리 + 자동차 길이
    safe_d = round(t_d + b_d, 4) + l
    # print(str(safe_d) +'m')
    return safe_d


if __name__ == '__main__':

    run()
