#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from math import ceil


# grilla y:-5, +5
max_y = 2#5
min_y = -2#-10
max_z = 3#15
min_z = -3#-10
step = 0.1

z_height = abs(min_z)+abs(max_z)
y_length = abs(min_y)+abs(max_y)
grilla = np.zeros((int(ceil(z_height/step)), 
                   int(ceil(y_length/step))))

to_save = []

colors = iter(cm.rainbow(np.linspace(0, 1, 10)))
global move_pub, just_once, robot_pose, matriz_hom
robot_pose = []
just_once = True

def callback(msg):
    print("points received")
    x = []
    y = []
    z = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        # delete everything lower than the camera -> considered as floor
        point_coor_robot = np.matmul(matriz_hom, np.array([point[0], point[1], point[2], 1]))
        if point_coor_robot[0] <= 0:
            print "[{}][{}]".format(point[1], point[2])
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        else:
            print "ignoring, x={}".format(point_coor_robot[0])
    plt.scatter(y, z)
    global robot_pose
    if len(robot_pose) > 0:
        plt.plot(robot_pose[1], robot_pose[2], '+', color='black')
    plt.title("time "+str(rospy.get_time()))
    plt.pause(0.5)
    #plt.gcf().clear()

def set_pos(msg):
    global robot_pose, matriz_hom
    matriz_hom = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        matriz_hom.append([point[0], point[1], point[2], point[3]])
    inv_hom = np.linalg.inv(matriz_hom)
    robot_pose = [inv_hom[0,3], inv_hom[1,3], inv_hom[2,3]]

def plot_grilla(msg):
    print("points received")
    x = []
    y = []
    z = []
    #f, axarr = plt.subplots(1, 2)
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        to_save.append(point)
        # delete everything lower than the camera -> considered as floor
        if point[0] <= 0:
            z_occupied = int(point[2]-min_z/step)
            y_occupied = int(point[1]-min_y/step)
            if grilla[z_occupied][y_occupied] == 0:
                grilla[z_occupied][y_occupied] = 1
                print "point:[{}][{}], set in:[{}][{}]".format(point[2], point[1], z_occupied, y_occupied)
                print "-min:[{}][{}], /step:[{}][{}]".format(point[2]-min_z, point[1]-min_y, (point[2]-min_z)/step, (point[1]-min_y)/step)
            x.append(point[0])
            y.append(point[1])
            z.append(point[2]) 
    if len(x) == 0:
        print "all points are floor"
    # plot grilla
    if len(robot_pose) > 0:
        print "robot at:[{}][{}], set in:[{}][{}]".format(robot_pose[2], robot_pose[1], z_occupied, y_occupied)
        grilla[int(robot_pose[2]-min_z/step)][int(robot_pose[1]-min_y/step)] = 0.6# robot position
    plt.imshow(grilla, extent=[min_y, max_y, max_z, min_z]) # cmap=plt.cm.Greens
    global just_once
    if just_once:
        plt.colorbar()
        just_once = False
    np.savetxt('points.out', np.array(to_save))
    plt.pause(0.5)

def move():
    global move_pub 
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    move_pub.publish(vel_msg)

def talker():
    rospy.init_node('grilla_maker', anonymous=True)
    rospy.Subscriber("/map_points", PointCloud2, callback)
    rospy.Subscriber("/robot_pos", PointCloud2, set_pos)
    request_pub = rospy.Publisher('/request_points', Int8, queue_size=10)
    global move_pub
    move_pub = rospy.Publisher('plan_cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) # 10hz

    request_points = 1
    while not rospy.is_shutdown():
        #rospy.loginfo(hello_str)
        request_pub.publish(request_points)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
