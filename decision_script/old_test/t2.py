# coding=utf-8
# !/usr/bin/env python
import rospy
import sys
import time
import serial
import os
from math import pow, sqrt, pi
from tf_conversions import transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
import actionlib
import std_msgs
from std_msgs.msg import String
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist,PoseStamped
from zimiao2.msg import ROSdata

reload(sys)
sys.setdefaultencoding('utf8')
k = 0

point = [
    [-0.015, 1.133, k],
]

###串口配置
global ser
ser = serial.Serial(
    port='/dev/ttyUSB0',  # 串口设备路径
    baudrate=115200,        # 波特率
    bytesize=8,           # 数据位
    stopbits=1,           # 停止位
    timeout=2             # 超时时间
)


###导航类
class navigation:
    def __init__(self):
        self.pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.linear_speed_x = 0  # 初始化数据变量为0
        self.angular_speed_z = 0
        self.yaw = 0
        self.pitch = 0
        # self.yaw2= 0
        # self.pitch2= 0
        self.dis = 0
        self.shoot_model = 10
        self.model = 4
        self.is_aimed = False
        self.is_target_exists = False
        self.t1 = 0
        self.t2 = 0



        self.x0 = 0
        self.y0 = 0
        # self.z0 = 0
        self.c0 = 'A'

        self.x1 = 0
        self.y1 = 0
        # self.z1 = 0
        self.c1 = 'A'

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))
    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")
    # def _feedback_cb(self, feedback):
    # rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)
    def pub_move_goal(self, p):  # 传入x y w 有使用rospy.spin()阻塞
        x, y, w = p
        rospy.loginfo("[Navi] goto %s %s %s", x, y, w)

        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.stamp = rospy.Time.now()
        nav_goal.target_pose.header.frame_id = "map"
        nav_goal.target_pose.pose.position.x = x
        nav_goal.target_pose.pose.position.y = y

        q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi)
        nav_goal.target_pose.pose.orientation.x = q[0]
        nav_goal.target_pose.pose.orientation.y = q[1]
        nav_goal.target_pose.pose.orientation.z = q[2]
        nav_goal.target_pose.pose.orientation.w = q[3]

        # 发送导航目标给MoveBase action服务器，并设置回调函数来处理结果和反馈
        self.move_base.send_goal(nav_goal, self._done_cb, self._active_cb)

        result = self.move_base.wait_for_result(rospy.Duration(45))  # 等待导航目标的完成，设置超时时间为45秒

        if not result:
            self.move_base.cancel_goal()
            # rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()  # 获取导航的最终状态
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)

        # rospy.spin()
        return True
    def pub_vel_change(self):
        if data is None:
            self.linear_speed_x = 0
            self.angular_speed_z = 0
        self.linear_speed_x = data.linear.x  # 将接收到的数据赋值给变量
        self.angular_speed_z = data.angular.z
        self.send_data()  # 调用处理数据的方法
        self.yaw = data.yaw
        self.pitch = data.pitch
        self.dis = data.dis
        self.aimed = data.is_aimed
        self.target_exists = data.is_target_exists
        self.t1 = time.time()
        self.send_data()  # 调用处理数据的方法



    def send_data(self):
        if self is None:  # 如果数据1和数据2都为None,给予赋值
            self.linear_speed = 0
            self.angular_speed = 0
            self.yaw = 0
            self.pitch = 0
            self.dis = 0
            self.is_aimed = False
            self.is_target_exists = False
        # 在这里处理接收到的数据
        rospy.loginfo("move_speed:%s 和%s,and %s,sm,%s",
                      self.linear_speed_x,
                      self.model,
                      self.angular_speed_z,
                      self.shoot_model)

        # rospy.loginfo("fight:%s 和%s,and %s,%s,and %s",self.yaw,self.pitch,self.dis,self.is_aimed,self.is_target_exists,)

        if self.yaw == 0 and self.pitch == 0:
            self.t2 = time.time()
            t = self.t2 - self.t1
            print('time', t)
            if t >= 1:
                self.model = 4
                self.shoot_model = 10
                t = 0
                self.t1 = 0
            # self.t2=0


        else:
            self.linear_speed_x = 0
            self.angular_speed_z = 0
            self.model = 2
            self.shoot_model = 12

        data_bytes = struct.pack('fffiffi',
                                 self.yaw,
                                 self.pitch,
                                 self.dis,
                                 self.shoot_model,
                                 self.linear_speed_x,
                                 self.angular_speed_z,
                                 self.model)
        # rospy.loginfo("发送的字节：%s",data_bytes)
        try:
            ser.write(data_bytes)  # 将数据发送到串口
            # ser.write(data_str.encode())
        except serial.SerialException as e:
            print('Serial communication error:', e)
            ser.close()
            try:
                os.system('sudo chmod 777 /dev/ttyUSB*')
                ser.open()
            except serial.SerialException as e:
                pass
            print("new")
        except ValueError:
            rospy.logwarn("发送的数据格式错误！")
            pass
        except TypeError:
            rospy.logwarn("发送类型错误")
            pass

        # 清除数据，以便下一次
        self.linear_speed = 0
        self.angular_speed = 0
        self.yaw = 0
        self.pitch = 0
        # self.t1=0
        self.dis = 0
        self.shootmodel = 10
        self.model = 4
        self.is_aimed = False
        self.is_target_exists = False
    def pub_shoot(ROSdata):
        if target_distence1>target_distence2:
            shoot_target= target_id2
        else:
            shoot_target=target_id1
        rospy.Publisher('/rosdata',ROSdata,queue_size=10)
    def move(t):
        if t <= 60:
            for i in range(2):
                print(i)
                nav_result = nav.pub_move_goal(point[i])
        elif t>60 and t <= 120:
            for i in range(2,4):
                print(i)
                nav_result = nav.pub_move_goal(point[i])
        elif t>120 and t <= 180:
            for i in range(4,6):
                print(i)
                nav_result = nav.pub_move_goal(point[i])
        elif t>180 and t <=240:
            for i in range(6,8):
                print(i)
                nav_result = nav.pub_move_goal(point[i])
        else:



if __name__ == '__main__':

    rospy.init_node('pose_publisher')
    if ser.isOpen():
        rospy.loginfo("串口已打开")
    nav = navigation()
    rospy.Subscriber('/cmd_vel', Twist, nav.pub_vel_change(), queue_size=10)  # 创建订阅者，订阅'topic1'话题，回调函数为callback1
    rospy.Subscriber('/rosdata', ROSdata, nav.pub_shoot(), queue_size=10)


    # send.send_data()
    rate = rospy.Rate(3)
    while True:
        for i in range(2):
            print(i)
            #nav_result = nav.pub_move_goal(point[i])
            time.sleep(1)
            print('--------------------next-----------------------')
            time.sleep(1)

        rospy.spin(10)
