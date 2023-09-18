#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pymongo import MongoClient
import json

def callback_odom(msg):
    posx = str(round(msg.pose.pose.position.x, 12))
    posy = str(round(msg.pose.pose.position.y, 12))
    posz = str(round(msg.pose.pose.position.z, 12))
    orix = str(round(msg.pose.pose.orientation.x, 12))
    oriy = str(round(msg.pose.pose.orientation.y, 12))
    oriz = str(round(msg.pose.pose.orientation.z, 12))
    oriw = str(round(msg.pose.pose.orientation.z, 12))
    msg = "{\"localisation.position.x\":" + posx + ",\"localisation.position.y\":" + posy + ",\"localisation.position.z\":" + posz + ",\"localisation.orientation.x\":" + orix + ",\"localisation.orientation.y\":" + oriy + ",\"localisation.orientation.z\":" + oriz + ",\"localisation.orientation.w\":" + oriw + "}"
    msg = json.loads(msg)
    print(msg)
    collection.update_one({"data":"robot3"}, { "$set":msg})
    
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
    client = MongoClient("mongodb://localhost:27017/")
    db = client.ros_db
    collection = db.ros_db_col
    
    rospy.init_node('db_test')
    subs = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_odom)

    #message = String()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
