#!/usr/bin/env python
# coding=utf-8   #
import rospy
import struct
import serial
from geometry_msgs.msg import Twist
# import random
import math


def motion(linearx,lineary, angular):
    # 计算机器人的前进速度和转向速度
    vx = linear
    vy = 0.
    omega = angular

    # 计算机器人的移动速度和方向角
    speed = math.sqrt(vx**2 + vy**2)
    angle = math.atan2(vy, vx)
    print(speed,angle)
    # 计算机器人的转向角速度和半径

    # 返回机器人的移动速度、方向角、转向角速度和半径
    return speed, angle
    
    
# 创建一个ROS节点
rospy.init_node('cmd_vel_to_serial_node')
baud=1500000

# 创建一个serial对象
ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
rospy.loginfo("波特率：%s",baud)

model=4
distance=0.0 # random.uniform(0, 5)
angle_yaw=0.0
angel_pithc=0.0
shoot_model=0

def callback(data):
    # 将cmd_vel消息的线速度和角速度转换成字节序列
    linear_speed = data.linear.x
    angular_speed = data.angular.z
    # speed, angle=motion(linear_speed, angular_speed)
    # angle=0
    #twist_msg = Twist()
    #twist_msg.linear.x = linear_speed
    #twist_msg.angular.z = angular_speed
    rospy.loginfo("发送的运动数据：%s 和%s,and %s",data.linear.x, data.angular.z,shoot_model)
    
    data_bytes =struct.pack('fffiffi',angle_yaw,angel_pithc,distance,shoot_model,data.linear.x,data.angular.z,model)
    # rospy.loginfo("发送的字节：%s",data_bytes)
   
    # 将数据发送到串口
    ser.write(data_bytes)
    #ser.write(data_str.encode())

# 订阅cmd_vel消息

if ser.is_open:
    print('串口已经打开')
    # date =1
    rospy.Subscriber('/cmd_vel', Twist, callback)

    # 进入ROS循环
    rospy.spin()
else:
    print('串口未打开')


# 关闭串口
ser.close()
