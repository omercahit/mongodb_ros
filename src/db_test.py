#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from pymongo import MongoClient
import json
import numpy as np
import tf


total_points = 0
path_list = []
robot_x, robot_y = (0,0)

def callback_path(msg):
    global total_points, route, path_list
    path_points = msg.poses
    if total_points==0:
        total_points = len(path_points)
        
    percentage = str(round((100-(len(path_points)/total_points*100)),2))
    #print(percentage + " " + str(len(path_points)))

    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/odom_combined', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))

    path_points = [(k.pose.position.x + trans[0] , k.pose.position.y + trans[1]) for k in path_points]
    collection.update_one({"robotName":"robot1"}, { "$set":{"Task.taskPercentage": percentage, "Task.pathPoints":path_points}})


def callback_odom(msg):
    posx = str(round(msg.pose.pose.position.x, 12))
    posy = str(round(msg.pose.pose.position.y, 12))
    posz = str(round(msg.pose.pose.position.z, 12))
    orix = str(round(msg.pose.pose.orientation.x, 12))
    oriy = str(round(msg.pose.pose.orientation.y, 12))
    oriz = str(round(msg.pose.pose.orientation.z, 12))
    oriw = str(round(msg.pose.pose.orientation.w, 12))
    msg = "{\"Pose.Position.x\":" + posx + ",\"Pose.Position.y\":" + posy + ",\"Pose.Position.z\":" + posz + ",\"Pose.Orientation.x\":" + orix + ",\"Pose.Orientation.y\":" + oriy + ",\"Pose.Orientation.z\":" + oriz + ",\"Pose.Orientation.w\":" + oriw + "}"
    msg = json.loads(msg)
    print(msg)
    collection.update_one({"robotName":"robot1"}, { "$set":msg})

    global robot_x, robot_y
    robot_x, robot_y = float(posx), float(posy)

def callback_vel(msg):
    v_lin = str(round(msg.linear.x, 2))
    v_ang = str(round(msg.angular.z, 2))
    msg = "{\"robotVelocity.linearVelocity\":"+ v_lin + ",\"robotVelocity.angularVelocity\":"+ v_ang + "}"
    msg = json.loads(msg)
    #print(msg)
    collection.update_one({"robotName":"robot1"}, { "$set":msg})

def callback_costmap(msg):
    global robot_x, robot_y
    length = int(np.sqrt(len(msg.data)))
    msg.data = np.array(msg.data).reshape((length,length))
    #print(msg.data.shape)
    costmap_border = 15
    divider = length/costmap_border
    hundreds = []
    unknowns = []
    for i in range(0, msg.data.shape[0]):
        for j in range(0, msg.data.shape[1]):
            if msg.data[i][j] == 100:
                hundreds.append((robot_x + (round((((j/divider)-(costmap_border/2))),3)),(robot_y + round(((i/divider)-(costmap_border/2)),3))))
                #print(hundreds)
            elif msg.data[i][j] == -1:
                unknowns.append((i,j))
    print((hundreds))
    collection.update_one({"robotName":"robot1"}, { "$set":{"createdCostmap":hundreds}})
    
def callback(msg):
    msg = str(msg)[6:]
    msg = "{\"data\":"+(str(msg))+"}"
    msg = json.loads(msg)
    print(msg)
    msg = collection.insert_one(msg)
    msg_id = msg.inserted_id
    print(msg_id)

if __name__ == '__main__':
    
    client=MongoClient()
    client = MongoClient("mongodb://172.16.66.130:27017/")
    db = client.altinay_amr_fleet
    collection = db.robots
    
    rospy.init_node('db_test')
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_odom)
    rospy.Subscriber("/cmd_vel", Twist, callback_vel)
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, callback_path)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback_costmap)

    rospy.spin()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
