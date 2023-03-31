#! /usr/bin/python

import cv2
import os
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from carla_msgs.msg import CarlaEgoVehicleStatus
from rospy.exceptions import ROSException

# Instantiate CvBridge
bridge = CvBridge()

cmd_file = "/home/chris/catkin_ws/src/ai_control/src/cmd.txt"
img_path = "/home/chris/catkin_ws/src/ai_control/src/img_data/"

rgb_img_files = os.listdir(img_path)
rgb_img_num = len(rgb_img_files)
cnt = rgb_img_num + 1 if rgb_img_num != 0 else 0

def image_callback(front_img_msg, cmd_msg):
    global cnt
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(front_img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        img_name = img_path + "img" + str(cnt) + ".jpg"
        cv2.imwrite(img_name, cv2_img)
        steer = cmd_msg.control.steer
        with open(cmd_file, 'a+') as f:
            f.write(str(cnt) + '.jpg ' + str(steer) + '\n')
            cnt = cnt + 1

def main():
    rospy.init_node('image_listener')

    cmd_sub = message_filters.Subscriber('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus)
    front_image_sub = message_filters.Subscriber('/carla/ego_vehicle/rgb_front/image', Image)
    ts = message_filters.TimeSynchronizer([front_image_sub, cmd_sub], 10)
    ts.registerCallback(image_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
