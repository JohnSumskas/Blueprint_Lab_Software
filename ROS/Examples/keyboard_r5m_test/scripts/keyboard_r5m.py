#!/usr/bin/env python3

import os, sys, time
import rospy
import serial
import re
import struct
import threading
from blueprintlab_reachsystem_ros_messages.msg import single_float
from pynput.keyboard import Key
from pynput.keyboard import Listener


cmd_velocity_pub = rospy.Publisher('r5m_0/cmd_velocity', single_float, queue_size=10)
cmd_velocity_message = single_float()

def keyboard_press(key):
    try:
        key_pressed = key.char
    except:
        return
    if key.char == 'w':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = -0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('s pressed')
    elif key.char == 'a':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'd':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = -0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed')        

def keyboard_release(key):    
    if key == Key.esc:
        return False
    try:
        key_released = key.char
    except:
        return
    if key.char == 'w':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('s pressed')
    elif key.char == 'a':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'd':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed')     

     


def keyboard_r5m_main():

    if not rospy.is_shutdown():
        with Listener(
                    on_press=keyboard_press,
                    on_release=keyboard_release
                    ) as listener:
            listener.join()

        


        
        #cmd_velocity_message.device_id = 3
        #cmd_velocity_message.value = 0.5
        #cmd_velocity_pub.publish(cmd_velocity_message)
        #time.sleep(3.0)
        #cmd_velocity_message.device_id = 3
        #cmd_velocity_message.value = -0.5
        #cmd_velocity_pub.publish(cmd_velocity_message)
        #time.sleep(3.0)




if __name__ == "__main__":
    rospy.init_node("keyboard_r5m")
    keyboard_r5m_main()
