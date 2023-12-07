#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

"""
Step 1) Type the user input
Step 2) Send the input to ChatGPT and parse information
        Run promts to train CHATGPT
Step 3) Read the response and perform tasks

----Future Work---
Currently this uses a text respose from the user, this will be converted to speech input, or text + speech will be implemented
"""

class LLMGPTClient():
    def __init__(self, pub):
        self.pub = pub
        self.send_text_command()

    def send_text_command(self):
        rospy.loginfo("Enter text command for the robot")
        while not rospy.is_shutdown():
            text_command = input("Enter here: ")
            rospy.loginfo(f"You typed : {text_command}") 
            self.pub.publish(text_command)


def main(args=None):
    rospy.init_node("client_node")
    rospy.loginfo("client node started")
    pub = rospy.Publisher("input_stream", String, queue_size=10) 
    llm_ros_client = LLMGPTClient(pub)
    while not rospy.is_shutdown(): 
        llm_ros_client
    



if __name__ == '__main__':
    main()
