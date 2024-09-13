#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from math import *
from geometry_msgs.msg import Pose,PointStamped
import tf2_ros
import tf2_geometry_msgs

HUE_LOW   = 0
HUE_HIGH  = 10
SATURATION_LOW  = 120
SATURATION_HIGH = 255
VALUE_LOW    = 70
VALUE_HIGH   = 255

class image_converter:
    def __init__(self):    
    # 创建cv_bridge，声明图像的发布者和订阅者
      self.bridge=CvBridge()	#ROS图像和OpenCV图像信息的转换
    #   self.image_sub=rospy.Subscriber("/camera/image_raw", Image, self.callback)	#订阅Image，Camera的话题
      self.image_sub=rospy.Subscriber("/limo/color/image_raw", Image, self.visual_callback)	#订阅Image，Camera的话题
      self.depth_sub=rospy.Subscriber("/limo/depth/image_raw",Image,self.depth_callback)    # 获取深度信息
      self.camera_info_sub = rospy.Subscriber("/limo/color/camera_info",CameraInfo,self.camera_info_callback)   # 获取相机内参
      self.image_pub=rospy.Publisher("object_detect_image", Image, queue_size=1)	#发布识别结果
      self.target_pub=rospy.Publisher("/object_detect_pose", PointStamped, queue_size=1)	#发布target的Pose信息
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def visual_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            image_input = self.bridge.imgmsg_to_cv2(data, "bgr8")	#将ROS中拿到的数据转换成OpenCV能够使用的数据
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.cvtColor(image_input,cv2.COLOR_BGR2HSV)  #将获得的bgr图转化为hsv图，这样更利于我们在真实环境中识别物体
        # print("Size of image:", cv_image.shape) #(480,640,3)

        # define the list of boundaries in BGR 
        boundaries = [([HUE_LOW, SATURATION_LOW, VALUE_LOW], [HUE_HIGH,SATURATION_HIGH,VALUE_HIGH])]	#识别颜色的范围值BGR

        # loop over the boundaries
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(cv_image, lower, upper)      # mask中颜色区域内的像素值为255，其他区域像素值为0，lower表示每个通道的下界

        cnts,_ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # output = cv2.bitwise_and(cv_image, cv_image, mask = mask)   # 取出掩码部位的像素

        # cvImg = cv2.cvtColor(output, 6) #cv2.COLOR_BGR2GRAY
        # npImg = np.asarray( cvImg )
        # thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

        # # find contours in the thresholded image，寻找轮廓，cv2.RETR_LIST：检测的轮廓不建立等级关系，cv2.CHAIN_APPROX_SIMPLE：只保存轮廓点坐标
        # cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cnts[0]

        # loop over the contours
        for c in cnts:  # 每个c是一个轮廓的轮廓点组成的数组
            # compute the center of the contour
            M = cv2.moments(c)

            if int(M["m00"]) not in range(10, 307200):	#M["m00"]是面积
                continue
            
            cX = int(M["m10"] / M["m00"])   # M["m10"]是一阶矩，用来计算质心位置
            cY = int(M["m01"] / M["m00"])   # M["m10"]是一阶矩，用来计算质心位置
            
            # 在图像上显示识别出的物体轮廓和质心
            cv2.drawContours(image_input, [c], -1, (255, 68, 0), 2)
            cv2.circle(image_input, (cX, cY), 1, (0, 255, 230), -1)

            # 获取对应质心的深度信息
            depth = self.depth_image[cY,cX]
            # rospy.loginfo("Robot detecting: get a target with depth "+str(depth)+"m")

            # 计算物体在相机坐标系中的位置,像素坐标系-->相机坐标系
            point_camera = PointStamped()
            point_camera.header.frame_id = data.header.frame_id
            point_camera.point.x = (cX - self.camera_info.K[2]) * depth / self.camera_info.K[0]
            point_camera.point.y = (cY - self.camera_info.K[5]) * depth / self.camera_info.K[4]
            point_camera.point.z = depth

            # 使用tf2将点从相机坐标系转换到世界坐标系
            point_world = self.tf_buffer.transform(point_camera,"odom")
            # rospy.loginfo("Robot detecting: target position "+str(point_world))

            # 发布目标位置
            self.target_pub.publish(point_world)

        # 显示Opencv格式的图像
        cv2.imshow("Image window", image_input)
        # cv2.imshow("Image window", mask)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def depth_callback(self,data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")  # "32FC1"表示32位浮点数单通道，常用于表示深度图像

    def camera_info_callback(self,data):
        self.camera_info = data



if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down object_detect node.")
        cv2.destroyAllWindows()