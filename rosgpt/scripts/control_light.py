#!/usr/bin/env python

# save for now, not updated !!!

'''
REFERENCES: 
https://www.emqx.com/en/blog/how-to-use-mqtt-in-python
'''

import rospy
from std_msgs.msg import String
from paho.mqtt import client as mqtt_client
import time


# generate client ID with pub prefix randomly
username = 'ohmni'
password = 'root'

# MQTT setup
broker = "129.69.207.135"  # replace with your MQTT broker address
port = 1883  # replace with your MQTT broker port

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client()
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client, topic, value):
    result = client.publish(topic, value) # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{value}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def send_mqtt_command(topic, value, info):
    print(f"Processing your request {info}...")
    client = connect_mqtt()
    client.loop_start()
    publish(client, topic, value)
    client.loop_stop()


def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)
    client.loop_stop()
    


if __name__ == '__main__':
    run()

