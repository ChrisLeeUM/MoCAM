#! /usr/bin/python

# Version: v1
# Model: pilotnet-pytorch-rgb-and-depth & pilotnet-pytorch
# Note: One rgb image input
# Author: Chris Lee

import cv2
import torch
import rospy
import numpy as np
from sensor_msgs.msg import Image
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

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        try:
            depth_msg = rospy.wait_for_message("/carla/ego_vehicle/depth_front/image", Image, timeout = 0.5)
            depth_img = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            depth_img_array = np.array(depth_img, dtype = np.dtype('f8'))
            norm_res = cv2.normalize(depth_img_array, depth_img_array, 0, 255, cv2.NORM_MINMAX)
        except ROSException as e:
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
            cmd_publisher = rospy.Publisher(
                "/carla/ego_vehicle/vehicle_control_cmd",
                CarlaEgoVehicleControl,
                queue_size=10)
            command = CarlaEgoVehicleControl()
            command.throttle     = 0.3
            command.steer        = steer
            # command.reverse      = reverse
            # command.brake        = brake
            cmd_publisher.publish(command)

def main():
    rospy.init_node('image_listener')
    image_topic = "/carla/ego_vehicle/rgb_front/image"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
