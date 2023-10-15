#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import pow, atan2, sqrt
import time
import numpy as np

class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/viv/viv_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/trunk_detection/y_err', Float32, self.update_pose)
        self.y_pose_subscriber = rospy.Subscriber ('/trunk_detection/x_err', Float32, self.update_y_pose)
        self.depth_to_tree = rospy.Subscriber ('/chatter', Float32, self.update_depth);
        self.depth = Float32()
        self.pose = Float32()
        self.rate = rospy.Rate(10)
        self.desired_pose = 0.0
        self.current_pose = 0.0
        self.KP = 1.0
        self.max_vel = 0.26

    def update_depth (self, data):
        self.depth = data

    def update_y_pose (self, data):
        self.y_pose = data

    def update_pose(self, data):
        self.pose = data
        self.current_pose = -data.data
        #pose is distance from the tree
        #self.pose.y = round(self.pose.y, 4)
    
    def linear_vel (self, constant = 1.5):
        return constant * self.pose
        
    def move2goal (self):
        vel_msg = Twist()
        do_job = False

        print ("Enter no of trees: ")
        #number_of_trees = int(input())
        #print("number_of_trees = ", number_of_trees)
        number_of_trees = 5
        offset = False
        while number_of_trees != 0:
            print ("usao")
            do_job = False
            vel_msg = Twist()
            pose_error = self.desired_pose - self.current_pose
            


            #while -float(self.pose.data) < 0.4 or do_job == False:
            while do_job == False:
                pose_error = self.desired_pose - self.current_pose
                print ("pose err > ", pose_error, "offset > ", offset)
                if offset == True:
                    vel_msg.linear.x = 0.5
                    #vel_msg.linear.x = 1
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                else:
                    vel_msg.linear.x = self.KP * pose_error
                    #vel_msg.linear.x = 1
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                
                self.velocity_publisher.publish(vel_msg)
                
                self.rate.sleep()
                if pose_error < 0.1 and pose_error > -0.1 and offset == False and self.depth.data < 0.65: 
                    do_job = True
                if offset == True and pose_error > 0.2 and self.depth.data < 0.65:
                    offset = False

            number_of_trees = number_of_trees - 1;
            offset = True
            vel_msg.linear.x = 0
            print ("parkiram se ----- ", number_of_trees);
            time.sleep(2);
            do_job = False;
            
            
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin()  

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass