#! /usr/bin/python

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl

class Controller:
    def __init__(self):
        rospy.init_node('ai_control', anonymous=True)
        self.cmd_publisher = rospy.Publisher(
            "/carla/ego_vehicle/vehicle_control_cmd",
            CarlaEgoVehicleControl,
            queue_size=10)
        self.rate = rospy.Rate(10)

    def publish_one_command(self, throttle, steer, reverse, brake):
        command = CarlaEgoVehicleControl()
        command.throttle     = throttle
        command.steer        = steer
        command.reverse      = reverse
        command.brake        = brake
        self.cmd_publisher.publish(command)

if __name__ == '__main__':
    try:
        vehicle_controller = Controller()
        while not rospy.is_shutdown():
            vehicle_controller.publish_one_command(0.3, 0.0, False, 0.0)
            vehicle_controller.rate.sleep()
    except rospy.ROSInterruptException:
        pass
