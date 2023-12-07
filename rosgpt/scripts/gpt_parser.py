#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from openai import OpenAI
import os

"""
Idea:
This node subcribes the "input_stream" topic from client node. 
Get the input from the user and send it to chatgpt to get response
"""

openai_api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI()
print(openai_api_key)

def askGPT(text_command):
    # TODO create prompts to train the model
    prompt = ''' '''
    prompt = prompt+'\nprompt: '+ str(text_command)
    print(prompt)

    # uncomment below to get prompts from chatgpt. Comment to save tokens

    # try:
    #     response = client.chat.completions.create(
    #         model="gpt-3.5-turbo",
    #         messages=[
    #         {"role": "system", "content": "You are a helpful assistant"},
    #         {"role": "user", "content": prompt}
    #     ]
    #     )
    # except Exception as e:
    #     print(f"Unexpected Error: {e}")
    #     return None
    # chatgpt_response = response.choices[0].message.content.strip()
    # print(chatgpt_response)



def parse(input:String):
    rospy.loginfo(f"The following message was received: {input}")
    #rospy.loginfo("Getting GPT response")
    #askGPT(input)

if __name__ == '__main__':
    rospy.init_node("gpt_parser")
    sub = rospy.Subscriber("user_text", String, callback=parse)
    rospy.loginfo('node initiated')
    
    # block until rosnode is shutdown
    rospy.spin()