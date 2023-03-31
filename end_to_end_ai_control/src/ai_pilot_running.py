#! /usr/bin/python

# Version: v1
# Model: pilotnet-pytorch-rgb-and-depth & pilotnet-pytorch
# Note: One rgb image input
# Author: Chris Lee

import cv2
import torch
import rospy
import math
import message_filters
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PIL_Image

#import the source files path
import sys
# TODO: Change the file path as your environment
sys.path.append(r'/home/chris/catkin_ws/src/ai_control/src')
from ai_pilot_common_lib import dataset_transforms, load_checkpoint
from ai_pilot_model import DrivingModel

from carla_msgs.msg import CarlaEgoVehicleControl
from rospy.exceptions import ROSException

bridge = CvBridge()
model = DrivingModel()
load_checkpoint(model)
device = torch.device("cuda:0")
model.to(device)

destination = {'x': -61.8, 'y': -12.1}

def callback(img_msg, odom_msg):
    print("Received an image!")
    try:
        cmd_publisher = rospy.Publisher(
            "/carla/ego_vehicle/vehicle_control_cmd",
            CarlaEgoVehicleControl,
            queue_size=10)
        command = CarlaEgoVehicleControl()
        current_pos_x = odom_msg.pose.pose.position.x
        current_pos_y = odom_msg.pose.pose.position.y
        d_x = current_pos_x - destination['x']
        d_y = current_pos_y - destination['y']
        distance = math.sqrt((d_x**2)+(d_y**2))
        print(distance)
        if (distance < 2):
            command.throttle     = 0
            command.steer        = 0
            # command.reverse      = reverse
            command.brake        = 1
            cmd_publisher.publish(command)
            print("Finish")
            return
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Change image format from cv2 to PIL
        raw_img_rgb = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        im_pil = PIL_Image.fromarray(raw_img_rgb)
        rgb_torch_data = dataset_transforms(im_pil)

        rgb_torch_data = rgb_torch_data.cuda()
        rgb_torch_data = torch.unsqueeze(rgb_torch_data, 0)
        steer = float(model(rgb_torch_data).item())
        print(steer)
        # Publish Commands

        command.throttle     = 0.3
        command.steer        = steer
        # command.reverse      = reverse
        # command.brake        = brake
        cmd_publisher.publish(command)

def main():
    rospy.init_node('image_listener')
    image_topic = message_filters.Subscriber('/carla/ego_vehicle/rgb_front/image', Image)
    odom_topic = message_filters.Subscriber('/carla/ego_vehicle/odometry', Odometry)
    ts = message_filters.TimeSynchronizer([image_topic, odom_topic], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main()
