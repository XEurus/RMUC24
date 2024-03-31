#!/usr/bin/env python
# coding=utf-8   #
import rospy
import struct
import serial
from geometry_msgs.msg import Twist
from nav.msg import rosdata
from std_msgs import float32
from std_msgs import bool
import math

class send_messges:
    def __init__(self):
        self.linear_speed_x = 0  # 初始化数据变量为0
        self.angular_speed_z = 0
        self.yaw= 0
        self.pithc= 0
        self.dis= 0 
        self.shoot_model=4
        self.model=0
        self.is_aimed = false
        self.is_target_exists = false
        
    def callback1(self, data):
        self.linear_speed_x = data.linear.x  # 将接收到的数据赋值给变量
        self.angular_speed_z = data.angular.z
        self.send_data()  # 调用处理数据的方法

    def callback2(self, data):
        self.yaw = data.yaw 
        self.pitch = data.pitch
        self.dis = data.dis
        self.aimed = data.is_aimed
        self.target_exists = data.is_target_exists
        self.send_data()  # 调用处理数据的方法

    def send_data(self):
        # if self.data1 is not None and self.data2 is not None:  # 如果数据1和数据2都不为None
        
        # 在这里处理接收到的数据
        rospy.loginfo("move_speed:%s 和%s,and %s",
        self.linear_speed_x,
        self.angular_speed_z,
        self.shoot_model)
        
        rospy.loginfo("fight:%s 和%s,and %s,%s,and %s",
        self.yaw,
        self.pithc,
        self.dis,
        self.is_aimed,
        self.is_target_exists,)
        if self.is_aimed is True:
            self.model=2
            self.shoot_model=1
        else:
            self.model=4
            self.shoot_model=0
            
        data_bytes =struct.pack('fffiffi',self.yaw,
        self.pithc,
        self.dis,
        self.shoot_model,
        self.linear_speed,
        self.angular_speed,
        model)
    # rospy.loginfo("发送的字节：%s",data_bytes)
   
    # 将数据发送到串口
    ser.write(data_bytes)
    #ser.write(data_str.encode())
  	
        # 清除数据，以便下一次接收
    self.linear_speed = 0  
    self.angular_speed= 0
    self.yaw= 0
    self.pithc= 0
    self.dis= 0 
    self.is_aimed = false
    self.is_target_exists = false

if __name__ == '__main__':

    #global model=4
    #global shoot_model=0
    
    # 创建一个ROS节点
    rospy.init_node('cmd_vel_to_serial_node')
    baud=1500000
    # 创建一个serial对象
    ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
    rospy.loginfo("波特率：%s",baud)

    send = send_messges()  # 创建DataProcessor类的实例

    rospy.Subscriber('/cmd_vel', Twist, send.callback1)  # 创建订阅者，订阅'topic1'话题，回调函数为callback1
    rospy.Subscriber('/rosdata', rosdata, send.callback2)  # 创建订阅者，订阅'topic2'话题，回调函数为callback2

    rospy.spin()  # 进入ROS的循环等待，监听消息的到达和回调函数的调用                                                                          
