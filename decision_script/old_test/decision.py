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
#from zimiao2.msg import ROSdata

#reload(sys)
#sys.setdefaultencoding('utf8')
k = 3.14

point = [
[0.297, 0.424,  -1.615,],
[1.782, -1.001, 1.508],
[-1.545, -4.363,  3.025],
[-5.012, 0.801, -1.421],
[-0.015, 1.133, k],
]

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
        self.begin=0

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))
    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    # def _feedback_cb(self, feedback):
        # rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def pub_move_goal(self, point,time_need,time_start,waiting_time):  # 传入x y w 有使用rospy.spin()阻塞
        """

        :param point: x坐标 y坐标 w:弧度制角度，rviz内发点有set_pose，可以直接读到
        :param time_need:
        :param time_start: 比赛开始时间
        :param waiting_time:需要设置等待时间，等待时间应当小于等于两次命令间时常
        :return: 1 成功
                 0 时间未到
                 2 超时失败
                 3 其他状态失败
        """
        time_now=time.time()-time_start
        if abs(time_now-time_need)<2:
            x, y, w = point
            rospy.loginfo("[Navi] goto %s %s %s", x, y, w)

            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.stamp = rospy.Time.now()
            nav_goal.target_pose.header.frame_id = "map"
            nav_goal.target_pose.pose.position.x = x
            nav_goal.target_pose.pose.position.y = y

            q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi) # 只有绕yaw的旋转角度
            #q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi)
            nav_goal.target_pose.pose.orientation.x = q[0]
            nav_goal.target_pose.pose.orientation.y = q[1]
            nav_goal.target_pose.pose.orientation.z = q[2]
            nav_goal.target_pose.pose.orientation.w = q[3]

            # 发送导航目标给MoveBase action服务器，并设置回调函数来处理结果和反馈
            self.move_base.send_goal(nav_goal, self._done_cb, self._active_cb)

            result = self.move_base.wait_for_result(rospy.Duration(waiting_time))  # 等待导航目标的完成，设置超时时间为45秒

            if not result:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                return_flag= 2
            else:
                state = self.move_base.get_state()  # 获取导航的最终状态
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("reach goal %s succeeded!" % point)
                    return_flag= 1
                else:
                    return_flag= 3

        else:
            return_flag= 0

        return return_flag
    '''
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

    '''

    '''
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
     
    '''
    def receive(self):
        try:
            received_data = ser.read(3)  # 使用逗号分隔数据  # 读取9个字节的数据
            if len(received_data) == 3:

                unpacked_data = struct.unpack('Hs', received_data)  # 解包成两个浮点数

                self.t1 = unpacked_data[0]
                self.begin = unpacked_data[1]

                rospy.loginfo("Received data: %s,%s" %self.begin %self.t1 )
                #print('Received data:', self.begin,self.t1)
                if self.t1 !=0:
                    return 1
                #break

            return 0
        except ValueError:
            rospy.logwarn("接收到的数据格式错误！")
            return 1

        except serial.SerialException as e:
            #   print('Serial communication error:', e)

            #   serial_port.close()
            #   serial_port.open()
            #   print("new")
            rospy.loginfo("error serial")
            pass

if __name__ == '__main__':
    print("---------")
    rospy.loginfo("decision begin")
    baud = 115200
    global ser
    ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
    rospy.loginfo("串口已打开")
    rospy.loginfo("波特率：%s", baud)

    rospy.init_node('decision')
    nav = navigation()

    while True:
        rospy.loginfo("----wait for begin-----")
        begin=nav.receive()
        if begin==1:
            t0 = time.time()
            print("receive",t0)
            #break

    #if ser.isOpen():
    #    rospy.loginfo("串口已打开")

    #rospy.Subscriber('/cmd_vel', Twist, nav.pub_vel_change(), queue_size=10)  # 创建订阅者，订阅'topic1'话题，回调函数为callback1
    #rospy.Subscriber('/rosdata', ROSdata, nav.pub_shoot(), queue_size=10)
    # send.send_data()
    #rate = rospy.Rate(30)
    flag_reach0 = 0
    flag_reach1 = 0
    flag_reach2 = 0
    flag_reach3 = 0
    flag_reach4 = 0
    flag_reach5 = 0
    while True:
        """
        :param point:
        :param time_need:
        :param time_now:
        :param waiting_time:需要设置等待时间，等待时间应当小于等于两次命令间时常
        :return: 1 成功
                 0 时间未到
                 2 超时失败
                 3 其他状态失败
        """
        # 现在比赛时间 等于 现在时间戳 减 开始时间戳
        move_result0=nav.pub_move_goal(point[0],0,t0,30)
        if move_result0 ==1:
            flag_reach0=1


        move_result1=nav.pub_move_goal(point[1],30, t0, 30)
        if move_result1 ==1:
            flag_reach1=1


        move_result2=nav.pub_move_goal(point[2],60, t0, 30)
        if move_result2 ==1:
            flag_reach2=1
        if move_result2 ==2 or move_result2==3:
            print("try again")
            move_result2 = nav.pub_move_goal(point[2], t0, t0, 30)

        move_result3=nav.pub_move_goal(point[3],90, t0, 30)
        if move_result3 ==1:
            flag_reach3=1

        #print("end")
    #rospy.spin(10)
