import rospy
import struct
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal

import math
import time
baud=115200
# 创建一个serial对象
global ser
ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
rospy.loginfo("波特率：%s",baud)

def receive(self):
    try:
        received_data = ser.read(3)  # 使用逗号分隔数据  # 读取9个字节的数据
        while len(received_data) == 3:
            unpacked_data = struct.unpack('si', received_data)  # 解包成两个浮点数

            t1 = int(unpacked_data[0])


            print('Received data:', t1)

            break

    except ValueError:
        rospy.logwarn("接收到的数据格式错误！")
        pass

    except serial.SerialException as e:
        #   print('Serial communication error:', e)

        #   serial_port.close()
        #   serial_port.open()
        #   print("new")
        pass