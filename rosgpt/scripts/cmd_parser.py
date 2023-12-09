#!/usr/bin/env python3

"""
INFO: This node parses the command from "voice_cmd" node and perform the operation on different topics
"""

import rospy
from std_msgs.msg import String
import json
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
from concurrent.futures import ThreadPoolExecutor
import copy
import math



class cmd_parse():
    def __init__(self):
        # make subscription to voice_cmd
        self.cmd_sub = rospy.Subscriber("voice_cmd", String, callback=self.get_args_and_parse)
        self.velocity_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) 
        self.pose_sub = rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback )
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose = Pose()
        self.thread_executor = ThreadPoolExecutor(max_workers=1)


    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pose = msg

    def get_args_and_parse(self, msg):
        try:
            rospy.loginfo("Msg Reveiced ... Now parsing: ")
            cmd = json.loads(msg.data)
            rospy.loginfo(cmd)

            if cmd['action']=='move':
                self.thread_executor.submit(self.turtlesim_move, cmd)
                #self.turtlesim_move(linear_speed, distance, is_forward)
            
            elif cmd['action']=='rotate':
                self.thread_executor.submit(self.turtlesim_rotate, cmd)
                #self.turtlesim_rotate(angular_velocity, angle, is_clockwise)
            
            elif cmd['action'] == 'sequence':
                for action in cmd['params']:
                    if action['action'] == 'move':
                        self.thread_executor.submit(self.turtlesim_move, action)
                    elif action['action'] == 'rotate':
                        self.thread_executor.submit(self.turtlesim_rotate, action)
                
        except json.JSONDecodeError:
            rospy.logerr(f"[json.JSONDecodeError] Invalid of empty JSON string received {msg.data}")
        except Exception as e:
            rospy.logerr(f"[Exception] An unexpected error occured: {str(e)}")
    
    def get_distance(self, start, destination):
        return math.sqrt(((destination.x-start.x)**2 + (destination.y-start.y)**2))
    

    def turtlesim_move(self, cmd):
        linear_speed = cmd['params'].get('linear_speed', 0.2)
        distance = cmd['params'].get('distance', 1.0)
        is_forward = cmd['params'].get('is_forward', True)
        if is_forward:
            direction='forward'
        else:
            direction='backward'
        if linear_speed > 10:
            rospy.logerr("[ERROR] linear speed should be less than 10")
            return -1
        if distance > 5.5:
            rospy.logerr("[ERROR] Distance should be less than 5.5")
            return -1
        rospy.loginfo(f'Start moving the robot {direction} at {linear_speed} m/s and for a distance {distance} meter')
        twise_msg = Twist()
        twise_msg.linear.x = float(abs(linear_speed) if is_forward else -abs(linear_speed))
        twise_msg.linear.x = float(abs(linear_speed) * (1 if is_forward else -1))

        start_pose = copy.copy(self.pose)
        rospy.loginfo(f'Start pose {start_pose}')
        

        while self.get_distance(start_pose, self.pose) < distance:
            rospy.loginfo(f"distance moved: {self.get_distance(start_pose, self.pose)}")
            self.velocity_pub.publish(twise_msg)

        twise_msg.linear.x = 0.0
        self.velocity_pub.publish(twise_msg)
        rospy.loginfo(f"distance moved: {self.get_distance(start_pose, self.pose)}")
        rospy.loginfo("Robot Stopped..")


    def turtlesim_rotate(self, cmd):
        angular_speed_degree = cmd['params'].get('angular_velocity', 1.0)
        desired_relative_angle_degree = cmd['params'].get('angle', 90.0)
        clockwise = cmd['params'].get('is_clockwise', True)
        rospy.loginfo("Start rotating the robot..")
        twist_msg = Twist()
        angular_speed_degree=abs(angular_speed_degree)
        if (angular_speed_degree>30) :
            rospy.loginfo(angular_speed_degree)
            rospy.loginfo('[ERROR]: The rotation speed must be lower than 0.5!')
            return -1

        angular_speed_radians = math.radians(angular_speed_degree)
        twist_msg.angular.z = -abs(angular_speed_radians) if clockwise else abs(angular_speed_radians)
        twist_msg.angular.z = abs(angular_speed_radians) * (-1 if clockwise else 1)

        start_pose = copy.copy(self.pose)
        rotated_related_angle_degree=0.0

        while rotated_related_angle_degree<desired_relative_angle_degree:
            self.velocity_pub.publish(twist_msg)
            rotated_related_angle_degree = math.degrees(abs(start_pose.theta - self.pose.theta))
            time.sleep(0.01)
        
        twist_msg.angular.z = 0.0
        self.velocity_pub.publish(twist_msg)
        print('The Robot has stopped...')

    def control_led(self):
        pass # TODO


def main():
    rospy.init_node("cmd_parser")
    rospy.loginfo("Started cmd_parse node..")
    node = cmd_parse()
    rospy.spin()
    

if __name__ == "__main__":
    main()

