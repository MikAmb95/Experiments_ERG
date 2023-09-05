#!/usr/bin/env python3
############################################################


import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import numpy as np


cmd = 0
val = 0
count = 0

def callback_ID1(data):
    global count
    count = 0
    x = 0
    print("ID200", data.data)

def callback_ID2(data):
    global count
    count = 0
    y = 0
    print("ID201", data.data)

def callback_cmd_inp(data):
    print("cmd_inp ", data.data[0]," ",data.data[1])

    global cmd
    global val
    global count
    count = 0


    cmd = data.data[0]
    val = data.data[1]



def main():
    
    
    msg_1 = Float64MultiArray()

    global cmd
    global count
    global val


    rospy.init_node('listener', anonymous=True)
    
    # ID200: z(cable) ID201: x(cart)

    #rospy.Subscriber("ID200", String, callback_ID1) 
    #rospy.Subscriber("ID201", String, callback_ID2) 

    rospy.Subscriber("cmd_input", Float64MultiArray, callback_cmd_inp) 
     

    

    pub1 = rospy.Publisher('cmd_left_right', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('cmd_down_up', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(1000) # 10hz
    
    while not rospy.is_shutdown():
        #val = 70
        #msg_1.data = np.array([val])
        
        #print("send")
        #pub1.publish(msg_1)
        #pub2.publish(msg_1)
        if cmd == 1 and count < 10:
            msg_1.data = np.array([val])
            print("x",val)
            pub1.publish(msg_1)
            count = count + 1
        elif cmd == 2 and count < 10:
            msg_1.data = np.array([val])
            print("z",val)
            pub2.publish(msg_1)
            count = count + 1

        rate.sleep

if __name__ == '__main__':
    main()