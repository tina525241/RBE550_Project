#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
#from __future__ import print_function
import sys
from nav_msgs.srv import GetPlan

def getplan(start, Goal):
    rospy.wait_for_service('/move_base/make_plan')  #wait_for_service(' service's name ')
    try:
        get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        req = GetPlan()
        req.start = start #change this
        req.goal = Goal #change this
        req.tolerance = .5
        resp = get_plan(req.start, req.goal, req.tolerance)
        print(resp)    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def callback(data):
    for i in range(0,len(data)):
        sum += math.sqrt(pow((data.poses[i+1].pose.position.x - data.poses[i].pose.position.x),2) + pow((data.poses[i+1].pose.position.y - data.poses[i].pose.position.y), 2))
    rospy.loginfo(data)

'''
def compair(sum):
    for i in range(0,len(sum)):
        min=sum[0]
        if min>sum[i]:
            min=sum[i]
    return min
'''

def pathlength():
    rospy.init_node('pathlength', anonymous=True)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback)

    


if __name__ == '__main__':
    
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = 0  #2.6
    start.pose.position.y = 0  #1.3

    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = 0.2  #-6.2
    Goal.pose.position.y = 0.5  #-3.0
    getplan(start,Goal)
    pathlength()
   
    