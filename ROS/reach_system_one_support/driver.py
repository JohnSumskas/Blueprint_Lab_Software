#!/usr/bin/env python

import rospy

def request():

    pass

def command():

    pass

def heartbeat():

    print("heartbeat")
    pass


if __name__ == '__main__':
    try:
        heartbeat()
    except rospy.ROSInterruptException:
        pass