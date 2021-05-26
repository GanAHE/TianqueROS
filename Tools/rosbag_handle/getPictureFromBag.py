
# coding:utf-8
#!/usr/bin/python
    
# Extract images from a bag file.
    
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
    
# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys
    
rgb_path = '/home/ganahe/D435i/track_image/'   #已经建立好的存储rgb彩色图文件的目录
depth_path= '/home/ganahe/D435i/right/' # 已经建立好的存储深度图文件的目录
    
class ImageCreator():
    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('/home/ganahe/experiment.bag', 'r') as bag:  #要读取的bag文件；
            for topic,msg,t in bag.read_messages():
                if topic == "/vins_estimator/image_track": #图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print e
                    timestr = "%.0f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr+ ".png" #图像命名：时间戳.png
                    cv2.imwrite(rgb_path + image_name, cv_image)  #保存；
                    step=0
    
if __name__ == '__main__':
    #rospy.init_node(PKG)
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass

