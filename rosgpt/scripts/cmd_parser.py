#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
from concurrent.futures import ThreadPoolExecutor
import copy
import math
from text_to_speech_tests import eleven_labs_tts, pyttxx3_test
from control_light import send_mqtt_command
from mailer import run_email_script
from nav_msgs.msg import Odometry



class cmd_parse():
    def __init__(self):
        # make subscription to voice_cmd
        self.cmd_sub = rospy.Subscriber("voice_cmd", String, callback=self.get_args_and_parse)
        #self.velocity_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.velocity_pub = rospy.Publisher("/tb_cmd_vel", Twist, queue_size=10) 
        self.pose_sub = rospy.Subscriber("/tb_control/wheel_odom", Odometry, self.pose_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.ctrl_c = False
        self.pose = Odometry()
        self.thread_executor = ThreadPoolExecutor(max_workers=1)
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.stop_robot()
        self.ctrl_c = True
        return 1

    def stop_robot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        twise_msg = Twist()
        twise_msg.linear.x = 0.0
        twise_msg.angular.z = 0.0
        self.velocity_pub.publish(twise_msg)


    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.position.z
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
            elif cmd["action"] == "Email":
                run_email_script()

            elif cmd["action"] == "stop":
                self.shutdownhook()

            elif cmd["action"] == "simple_chat":
                text = cmd["params"]
                eleven_labs_tts(text)
            elif cmd["action"]=="TurnLights":
                info = cmd["action"]
                onoff = cmd["params"]["value"]
                topic = "home/light/command" # topic will be changed to light topic in the lab
                try:
                    send_mqtt_command(topic, info+onoff, info)
                    eleven_labs_tts(f"Turned {onoff} the lights successfully")
                except:
                    rospy.logerr("[ERROR] Error occured in sending MQTT command, Please try again")
                
        except json.JSONDecodeError:
            rospy.logerr(f"[json.JSONDecodeError] Invalid of empty JSON string received {msg.data}")
        except Exception as e:
            rospy.logerr(f"[Exception] An unexpected error occured: {str(e)}")
    
    def get_distance(self, start, destination):
        return math.sqrt(((destination.pose.pose.position.x-start.pose.pose.position.x)**2 + \
                          (destination.pose.pose.position.y-start.pose.pose.position.y)**2))
    

    def turtlesim_move(self, cmd):
        linear_speed = cmd['params'].get('linear_speed', 0.1)
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
        

        while self.get_distance(start_pose, self.pose) < distance and not self.ctrl_c:
            rospy.loginfo(f"distance moved: {self.get_distance(start_pose, self.pose)}")
            self.velocity_pub.publish(twise_msg)

        twise_msg.linear.x = 0.0
        self.velocity_pub.publish(twise_msg)
        rospy.loginfo(f"distance moved: {self.get_distance(start_pose, self.pose)}")
        rospy.loginfo("Robot Stopped..")
        self.thread_executor.submit(eleven_labs_tts, f"Moved the Robot {distance} meter {direction} successfully ")
        time.sleep(1)
        #eleven_labs_tts(f"Moved the Robot {distance} meter {direction} successfully ")


    def turtlesim_rotate(self, cmd):
        angular_speed_degree = cmd['params'].get('angular_velocity', 1.0)
        desired_relative_angle_degree = cmd['params'].get('angle', 90.0) # 60
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
        print(f"Start pose twist: {start_pose.twist.twist.angular}")
        print(f"target degree: {desired_relative_angle_degree}")
        print(f"angular speed degrees: {angular_speed_degree} radians: {angular_speed_radians}")
        angle_r = desired_relative_angle_degree * 3.14 / 180
        t0 = rospy.Time.now().secs
        while rotated_related_angle_degree<angle_r:
            self.velocity_pub.publish(twist_msg)
            t1 = rospy.Time.now().secs
            rotated_related_angle_degree = angular_speed_radians* (t1-t0)
            #rotated_related_angle_degree = math.degrees(abs(start_pose.twist.twist.angular.z - self.pose.twist.twist.angular.z))
            rospy.loginfo(f"[Angle remaining] {desired_relative_angle_degree - math.degrees(rotated_related_angle_degree):.2f}")
            time.sleep(0.01)
        
        twist_msg.angular.z = 0.0
        self.velocity_pub.publish(twist_msg)
        rospy.loginfo('The Robot has stopped...')
        self.thread_executor.submit(eleven_labs_tts, f"Rotated the Robot {desired_relative_angle_degree} degrees ")
        #eleven_labs_tts(f"Rotated the Robot {desired_relative_angle_degree} degrees ")
        time.sleep(1)

def main():
    rospy.init_node("cmd_parser")
    rospy.loginfo("Started cmd_parse node..")
    node = cmd_parse()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == "__main__":
    main()

