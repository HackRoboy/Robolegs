#!/usr/bin/python3

import rospy
from joint_angle_sensors.msg import jas
import serial
import argparse
import sys
import can
from numpy import *

can.rc['interface'] = 'socketcan_native'
can.rc['channel'] = 'can0'

idMap = {"Bottom0" : 0x50, "Top0" : 0x51, "Bottom1" : 0x53, "Top1" : 0x52}


def receiveByteAsInt(serialPort):
    try:
        data = serialPort.read()
        while len(data) == 0:
            data = serialPort.read()
        return ord(data)
    except serial.SerialException:
        print("Serial port " + serialPort.name + " seems to be closed. I will not try to recover it and exit immediately")
        sys.exit(-1)


##
# expect following data
# 0xFF, 0xFF - data header
# 0xXX - CAN ID
# 0xXX, 0xXX - 12b angle
def receiveDataSerial(serialPort):
    data = receiveByteAsInt(serialPort)
    if data == 0xFF:
        data = receiveByteAsInt(serialPort)
        if data == 0xFF:
            channelID = receiveByteAsInt(serialPort)
            data = receiveByteAsInt(serialPort)
            angle = data
            data = receiveByteAsInt(serialPort)
            angle += data << 8
            return (channelID, angle)

    # error return value
    return (-1, -1)

def receiveDataCAN(canBus):
    message = canBus.recv()
    if message is not None:
        channelId = message.arbitration_id
        if message.dlc == 2:
            angle = message.data[0] + (message.data[1] << 8)
            return (channelId, angle)

    return (-1, -1)

def calculateMissingValue( q1 , q2 , q3 ):
    L2 = 0.535;
    L3 = 0.173;
    Llegs = 0.93;
    L = 0.48
    theta1 = (180 + q1)*pi/180;
    theta2 = (180 + q2)*pi/180;
    theta3 = (180 + q3)*pi/180;
    K1 = sqrt( L3**2 + L2**2 - 2*L2*L3*cos(theta2) );
    s1 = arcsin( (L2/K1) * sin(theta2) );
    s2 = theta3 - s1;
    K2 = sqrt( K1**2 + L2**2 - 2*K1*L2*cos(s2) );
    a2 = arcsin( L2/K2*sin(s2) );
    a1 = arcsin( L3/K1*sin(theta2) );
    a3 = theta1 - a1 - a2;
    K3 = sqrt( K2**2 + L**2 - 2*L*K2*cos(a3) );
    a4 = arcsin( L2/K3*sin(a3) );
    l2 =  pi - a3 - a4;
    l1 = pi - s2 - a2;
    tmp = (K3**2 - L**2 - Llegs**2) / (-2*Llegs*L)
    tmp = max(min(tmp,1),-1)
    theta5 = arccos( tmp );
    l3 = arcsin( Llegs/K3*sin(theta5) );
    theta4 = l1 + l2 + l3;
    q4 = theta4 - pi + (50*pi/180)
    return -q4*180/pi;
    


def publisher(serialPort, canBus, useSerial):
    pub = rospy.Publisher("joint_angle_sensors", jas, queue_size=10)
    rospy.init_node("robolegs")
    jasmsg = jas()
    jasmsg.Bottom0 = 0
    jasmsg.Top0 = 0
    jasmsg.Bottom1 = 0
    jasmsg.Top1 = 0
    cnt = 0
    while not rospy.is_shutdown():
        if useSerial == True:
            jasData = receiveDataSerial(serialPort)
        else:
            jasData = receiveDataCAN(canBus)

        if jasData[0] == idMap['Bottom0']:
           jasmsg.Bottom0 = int((jasData[1] - 2000)*0.085)
           jasmsg.Bottom0 = int(jasData[1])
        if jasData[0] == idMap['Top0']:
           jasmsg.Top0 = int(jasData[1]*0.0982 -277.26)
           jasmsg.Top0 = int(jasData[1])
        if jasData[0] == idMap['Bottom1']:
            jasmsg.Bottom1 = int(jasData[1]*-0.0939 + 196.78)
            jasmsg.Bottom1 = int(jasData[1])
        if jasData[0] == idMap['Top1']:
            jasmsg.Top1 = int(jasData[1]*-0.071 + 42.239)
            jasmsg.Top1 = int(jasData[1])

        if jasData[0] != -1:
            #print(jasmsg.Top1)
            pub.publish(jasmsg)

if __name__ == "__main__":
    try:
        argv = rospy.myargv(argv=sys.argv)
        parser = argparse.ArgumentParser(description='Publisher for joint angle sensors')
        parser.add_argument("--useSerial", dest="useSerial", action="store_true", help="Use serial interface instead of CAN bus", default='false')
        parser.add_argument("--port", dest="port", help="Path to the serial port device", default='/dev/ttyACM0')
        args = parser.parse_args(argv[1:])

        #serialPort = serial.Serial(args.port, 115200, timeout=1)
        if args.useSerial == True:
            serialPort = serial.Serial(args.port, 115200, timeout=1)
            serialPort.close()
            serialPort.parity = serial.PARITY_ODD
            serialPort.open()
            serialPort.close()
            serialPort.parity = serial.PARITY_NONE
            serialPort.open()
            publisher(serialPort, None, True)
        else:
            canBus = can.interface.Bus()
            publisher(None, canBus, False)
    except rospy.ROSInterruptException:
        pass

