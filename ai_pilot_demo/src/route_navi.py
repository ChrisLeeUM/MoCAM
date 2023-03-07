import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, pi, sqrt

goal1 = [-81.1, -30.8, 0, 0]  # init pose
goal2 = [-81.1, -7.7, 0, 0]
goal3 = [-81.1, 9.8, 0, 0]

goal_list = [goal1, goal2, goal3]
goal_index = 0
goal_num = len(goal_list)
# vehicle_pose = PoseStamped()

cur_position = [-86, -3.7, 4.0, 0]

def goal_pub():

    global goal_index, cur_position

    pub = rospy.Publisher('/carla/ego_vehicle/goal', PoseStamped, queue_size=10)
    rospy.init_node('ad_goal', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odom_callback)

    while not rospy.is_shutdown():

        pose = goal_to_pose(goal_list[goal_index])
        pub.publish(pose)
        print('pub goal ' + str(goal_index) + ' successful')

        rate.sleep()

def odom_callback(data):
    # global vehicle_pose
    global goal_index, cur_position

    vx = data.pose.pose.position.x
    vy = data.pose.pose.position.y
    vz = data.pose.pose.position.z
    vyaw = quat_to_yaw(data.pose.pose.orientation)

    gx = goal_list[goal_index][0]
    gy = goal_list[goal_index][1]
    gz = goal_list[goal_index][2]

    cur_position = [vx, vy, 4.0, vyaw]

    distance = sqrt( (vx-gx)**2 + (vy - gy) ** 2 )

    if distance <= 5:
        goal_index += 1

        if goal_index >= goal_num:
            goal_index = 0


def goal_to_pose(goal):
    x = goal[0]
    y = goal[1]
    z = goal[2]
    yaw = goal[3]
    yaw = yaw * (180 / pi)

    pose = PoseStamped()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation = yaw_to_quaternion(yaw)

    return pose

def quat_to_yaw(quater):

    x = quater.x
    y = quater.y
    z = quater.z
    w = quater.w

    yaw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

    return yaw

def yaw_to_quaternion(yaw):

    qx = 0
    qy = 0
    qz = sin(yaw/2)
    qw = cos(yaw/2)

    orientation = Quaternion()
    orientation.x = qx
    orientation.y = qy
    orientation.z = qz
    orientation.w = qw

    return orientation


if __name__ == '__main__':
    try:
        goal_pub()
    except rospy.ROSInterruptException:
        pass
