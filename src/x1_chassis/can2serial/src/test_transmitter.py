# -*- coding:utf-8 -*-
#/usr/bin/python

from can_msgs.msg import Frame
import rospy

def main():
    rospy.init_node("test_transmitter_node")
    pub = rospy.Publisher("/send_can_msg", Frame, queue_size=100)

    frame = Frame()
    frame.id = 100
    frame.dlc = 8
    frame.data = [1,2,3,4,5,6,7,8]

    cnt = 1
    while not rospy.is_shutdown():
        pub.publish(frame)
        print("send count: %d" %cnt)
        cnt = cnt+1
        rospy.sleep(0.001)
    
main()