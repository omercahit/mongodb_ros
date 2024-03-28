#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from pymongo import MongoClient
import json
import numpy as np
import tf


total_points = 0
path_list = []

def path_callback(msg):
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
    collection.update_one({"robotName":"robot2"}, { "$set":{"Task.taskPercentage": percentage, "Task.pathPoints":path_points}})


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
    collection.update_one({"robotName":"robot2"}, { "$set":msg})

def callback_vel(msg):
    v_lin = str(round(msg.linear.x, 2))
    v_ang = str(round(msg.angular.z, 2))
    msg = "{\"robotVelocity.linearVelocity\":"+ v_lin + ",\"robotVelocity.angularVelocity\":"+ v_ang + "}"
    msg = json.loads(msg)
    #print(msg)
    collection.update_one({"robotName":"robot2"}, { "$set":msg})
    
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
    subs = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_odom)
    sub_vel = rospy.Subscriber("/cmd_vel", Twist, callback_vel)
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, path_callback)

    rospy.spin()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
