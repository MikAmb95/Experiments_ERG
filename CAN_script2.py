#!/usr/bin/env python3
############################################################

import rospy
from std_msgs.msg  import String
from std_msgs.msg import Float64MultiArray
import numpy as np

from PCANBasic import *
import os
import sys


#global data config #

mex_up_down = 0
mex_left_right = 0

# Sets the PCANHandle (Hardware Channel)
PcanHandle = PCAN_USBBUS1

# Sets the bitrate for normal CAN devices
Bitrate = PCAN_BAUD_500K
# Sets the bitrate for CAN FD devices. 
# Example - Bitrate Nom: 1Mbit/s Data: 2Mbit/s:
#   "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"
BitrateFD = b'f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1'    
#endregion
# Members
#region

# Shows if DLL was found
m_DLLFound = False


#pub and sub

rospy.init_node('talker', anonymous=True)

# ID200: z(cable) ID201: x(cart)

pub1 = rospy.Publisher('ID200', String, queue_size=10)
pub2 = rospy.Publisher('ID201', String, queue_size=10)


def init_function():
    try:
        m_objPCANBasic = PCANBasic()
        m_DLLFound = True
    except :
        print("Unable to find the library: PCANBasic.dll !")
        #getInput("Press <Enter> to quit...")
        m_DLLFound = False
        return
    
    return m_objPCANBasic

def getInput():
    res = ""
    if sys.version_info[0] >= 3:
        res = input()
    else:
        res = raw_input()
    if len(res) == 0:
        res = default
    
    return res


#ID100: left ID101: right ID102: down ID103: up

def WriteMessageLeftRight(m_objPCANBasic,msg1):

    
    
    msgCanMessage = TPCANMsg()
    msgCanMessage.ID = 0x64
    msgCanMessage.LEN = 8
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED.value
    mgs1 = int(msg1*100)
    msgCanMessage.DATA[0] = mgs1-256*int(mgs1/256)
    msgCanMessage.DATA[1] = int(mgs1/256)

    return m_objPCANBasic.Write(PcanHandle, msgCanMessage)


def WriteMessageUpDown(m_objPCANBasic,msg1):
    msgCanMessage = TPCANMsg()
    msgCanMessage.ID = 0x66
    msgCanMessage.LEN = 8
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED.value
    mgs1 = int(msg1*100)
    msgCanMessage.DATA[0] = mgs1-256*int(mgs1/256)
    msgCanMessage.DATA[1] = int(mgs1/256)

    return m_objPCANBasic.Write(PcanHandle, msgCanMessage)



def ReadMessage(m_objPCANBasic):
    ## We execute the "Read" function of the PCANBasic   
    stsResult = m_objPCANBasic.Read(PcanHandle)

    if stsResult[0] == PCAN_ERROR_OK:
        ProcessMessageCan(stsResult[1],stsResult[2])
            
    return stsResult[0]

def callback_ID1(data):
    global mex_left_right
    #msg = (mex_left_right/10)-data.data[0]
    # if you are using the script ros_PD_TS
    msg = 24.1-data.data[0] #to change 20 based on the intial condition of the crane
    # if you are using the script ros_PD_JS
    #msg = data.data[0] 
    print("ID1",msg)
    WriteMessageLeftRight(m_objPCANBasic,msg)
    

def callback_ID2(data):
    #global mex_up_down
    #msg = (mex_up_down/10)+data.data[0]
    msg = 112.84+data.data[0]
    #msg = data.data[0]
    print("ID2",msg)
    WriteMessageUpDown(m_objPCANBasic,msg)




def ProcessMessageCan(msg,itstimestamp):
    
    #rospy.init_node('talker', anonymous=True)
    #pub1 = rospy.Publisher('ID1', String, queue_size=10)
    #pub2 = rospy.Publisher('ID2', String, queue_size=10)

    #rospy.Subscriber('cmd_left_right',Float64MultiArray, callback_ID1)
    #rospy.Subscriber('cmd_down_up', Float64MultiArray, callback_ID2)
    
    #microsTimeStamp = itstimestamp.micros + 1000 * itstimestamp.millis + 0x100000000 * 1000 * itstimestamp.millis_overflow
         
    #print("ID: " + GetIdString(msg.ID, msg.MSGTYPE))
    #print("Time: " + GetTimeString(microsTimeStamp))
    print("ID: " + GetIdString(msg.ID, msg.MSGTYPE) + " Data: " + GetDataString(msg.DATA,msg.MSGTYPE))
    #print("----------------------------------------------------------")

    #ID200: z(cable) ID201: x(cart)

    global mex_left_right
    global mex_up_down

    if GetIdString(msg.ID, msg.MSGTYPE) == "0C8h":
        #print("trasmitting z: " + GetDataString(msg.DATA,msg.MSGTYPE))
        mex_up_down = float(GetDataString(msg.DATA,msg.MSGTYPE))
        #print("mex_up_down",mex_up_down)
        pub1.publish(GetDataString(msg.DATA,msg.MSGTYPE))
    elif GetIdString(msg.ID, msg.MSGTYPE) == "0C9h":
        #print("trasmitting x: " + GetDataString(msg.DATA,msg.MSGTYPE))
        mex_left_right = float(GetDataString(msg.DATA,msg.MSGTYPE))
        #print("mex_left_right",mex_left_right)
        pub2.publish(GetDataString(msg.DATA,msg.MSGTYPE))
    


    

def GetIdString(id, msgtype):
    if (msgtype & PCAN_MESSAGE_EXTENDED.value) == PCAN_MESSAGE_EXTENDED.value:
        return '%.8Xh' %id
    else:
        return '%.3Xh' %id

def GetTimeString(time):
        fTime = time / 1000.0
        return '%.1f' %fTime

def GetDataString(data, msgtype):
    if (msgtype & PCAN_MESSAGE_RTR.value) == PCAN_MESSAGE_RTR.value:
        return "Remote Request"
    else:
        strTemp = 0
        strTemp = data[0] + 256*data[1]
        return str(strTemp)


def main():

    #endregion
    #m_objPCANBasic = init_function()
    #stsResult = m_objPCANBasic.Initialize(PcanHandle,Bitrate)

    rospy.Subscriber('cmd_left_right',Float64MultiArray, callback_ID1)
    rospy.Subscriber('cmd_down_up', Float64MultiArray, callback_ID2) 

  
    while not rospy.is_shutdown():
        #strinput = getInput()
        #strinput = chr(ord(strinput))
        #print("string"+strinput)
        ReadMessage(m_objPCANBasic)
        #WriteMessageUp(m_objPCANBasic)




if __name__ == '__main__':
    m_objPCANBasic = init_function()
    stsResult = m_objPCANBasic.Initialize(PcanHandle,Bitrate)
    main()