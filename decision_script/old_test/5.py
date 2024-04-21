#!/usr/bin/env python
# coding=utf-8   #
import rospy
import struct
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
from zimiao2.msg import ROSdata
import math
import time
from multiprocessing import Queue
import threading
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

k = 3.14

point = [
[0.297, 0.424,  -1.615,],
[1.782, -1.001, 1.508],
[-1.545, -4.363,  3.025],
[-5.012, 0.801, -1.421],
[-0.015, 1.133, k],
]

class navigation:
    def __init__(self):
        self.pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.dis = 0
        self.is_aimed = False
        self.is_target_exists = False
        self.begin = 0

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    # def _feedback_cb(self, feedback):
    # rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def pub_move_goal(self, point, time_need, time_start, waiting_time):  # 传入x y w 有使用rospy.spin()阻塞
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
        time_now = time.time() - time_start
        if abs(time_now - time_need) < 2:
            x, y, w = point
            rospy.loginfo("[Navi] goto %s %s %s", x, y, w)

            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.stamp = rospy.Time.now()
            nav_goal.target_pose.header.frame_id = "map"
            nav_goal.target_pose.pose.position.x = x
            nav_goal.target_pose.pose.position.y = y

            q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi)  # 只有绕yaw的旋转角度
            # q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi)
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
                return_flag = 2
            else:
                state = self.move_base.get_state()  # 获取导航的最终状态
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("reach goal %s succeeded!" % point)
                    return_flag = 1
                else:
                    return_flag = 3

        else:
            return_flag = 0

        return return_flag

    def receive(self):
        try:
            received_data = ser.read(3)  #  读取3个字节的数据
            if len(received_data) == 3:

                unpacked_data = struct.unpack('Hs', received_data)  # 解包成两个浮点数

                game_t = unpacked_data[0]
                self.begin = unpacked_data[1]

                rospy.loginfo("Received data: %s,%s" % self.begin % self.t1)
                # print('Received data:', self.begin,self.t1)
                if game_t != 0:
                    rospy.loginfo("game start")
                    return 1

            return 0
        except ValueError:
            rospy.logwarn("接收到的数据格式错误！")
            return 0

        except serial.SerialException as e:
            #   print('Serial communication error:', e)

            #   serial_port.close()
            #   serial_port.open()
            #   print("new")
            rospy.loginfo("error serial")
            return 0


class send_messges:
    baud = 115200

    # ser=serial.Serial('/dev/USB0', baud, timeout=1)
    def __init__(self):
        self.linear_speed_x = 0  # 初始化数据变量为0
        self.linear_speed_y = 0
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

        self.begin = 0

        self.x0 = 0
        self.y0 = 0
        # self.z0 = 0
        self.c0 = 'A'

        self.x1 = 0
        self.y1 = 0
        # self.z1 = 0
        self.c1 = 'A'

    def callback1(self, data):
        if data is None:
            self.linear_speed_x = 0
            self.linear_speed_y = 0
            self.angular_speed_z = 0
        else:
            self.linear_speed_x = data.linear.x  # 将接收到的数据赋值给变量
            self.linear_speed_y = data.linear.y
            self.angular_speed_z = data.angular.z
            self.send_data()  # 调用处理数据的方法

    def callback2(self, data):
        self.yaw = data.yaw
        self.pitch = data.pitch
        self.dis = data.dis
        self.aimed = data.is_aimed
        self.target_exists = data.is_target_exists
        self.t1 = time.time()
        self.send_data()  # 调用处理数据的方法

    def send_data(self):
        if self is None:  # 如果数据1和数据2都为None,给予赋值
            self.linear_speed_x = 0
            self.linear_speed_y = 0
            self.angular_speed_z = 0
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
            if t >= 0.8:
                self.model = 4
                self.shoot_model = 0
                t = 0
                self.t1 = 0
            # self.t2=0


        else:
            self.linear_speed_x = 0
            self.linear_speed_y = 0
            self.angular_speed_z = 0
            self.model = 2
            self.shoot_model = 1

        # self.linear_speed_y=1
        data_bytes = struct.pack('fffifffi',
                                 self.yaw,
                                 self.pitch,
                                 self.dis,
                                 self.shoot_model,
                                 self.linear_speed_x,
                                 self.linear_speed_y,
                                 self.angular_speed_z,
                                 self.model)
        # rospy.loginfo("发送的字节：%s",data_bytes)
        try:
            ser.write(data_bytes)  # 将数据发送到串口
            # ser.write(data_str.encode())

        except serial.SerialException as e:
            print('Serial communication error:', e)

            ser.close()
            ser.open()
            print("new")

        except ValueError:
            rospy.logwarn("接收到的数据格式错误！")
            pass

        # 清除数据，以便下一次
        self.linear_speed_x = 0
        self.linear_speed_y = 0
        self.angular_speed_z = 0
        self.yaw = 0
        self.pitch = 0
        # self.t1=0
        self.dis = 0
        self.shootmodel = 10
        # self.model=10
        self.is_aimed = False
        self.is_target_exists = False

    def receive(self):
        try:
            received_data = ser.read(3)  # 使用逗号分隔数据  # 读取9个字节的数据
            #print(len(received_data))
            if len(received_data) == 3:
                unpacked_data = struct.unpack('Hs', received_data)  # 解包成两个浮点数
                self.begin = unpacked_data[0]
                print('Received data:', self.begin)

                # break

        except ValueError:
            rospy.logwarn("接收到的数据格式错误！")
            pass
        except serial.SerialException as e:
            rospy.logwarn('Serial communication error:', e)
            #   print('Serial communication error:', e)
            #   serial_port.close()
            #   serial_port.open()
            #   print("new")
            pass



def publish_initial_pose():
    # rospy.init_node('amcl_initial_pose_publisher', anonymous=True)
    pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(0.5)  # 等待1秒，确保发布者和订阅者之间建立连接

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"

    # 设置初始位置的坐标
    initial_pose.pose.pose.position.x = 7.0  # 设置x坐标
    initial_pose.pose.pose.position.y = 8.5  # 设置y坐标
    initial_pose.pose.pose.position.z = 0.0  # 设置z坐标

    # 设置初始位置的方向
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    pub_amcl.publish(initial_pose)
    rospy.loginfo("发布初始位置%s,%s", initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y)

    #number = 0  # 发布初始化激活 之前使用初始化激活后，等待c坂回应再往下发
    #q = struct.pack('i', number)
    #ser.write(q)


if __name__ == '__main__':

    baud = 115200
    global ser
    ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
    rospy.loginfo("波特率：%s", baud)

    nav = navigation()
    send = send_messges()  # 创建类的实例

    rospy.init_node('cmd_vel_to_serial_node')
    # try:
        # publish_initial_pose()
    # except rospy.ROSInterruptException:
        # pass
    rospy.Subscriber('/cmd_vel', Twist, send.callback1, queue_size=20)  # 创建订阅者，订阅'topic1'话题，回调函数为callback1
    rospy.Subscriber('/rosdata', ROSdata, send.callback2,queue_size=20) # 创建订阅者，订阅'topic2'话题，回调函数为callback2
    rate = rospy.Rate(30)  # max of frame 80-90

    flag_reach0 = 0
    flag_reach1 = 0
    flag_reach2 = 0
    flag_reach3 = 0
    flag_reach4 = 0
    flag_reach5 = 0
    while not rospy.is_shutdown(): #决策线程
        if begin!=1:
            data_bytes = struct.pack('i', 0)
            ser.write(data_bytes)  # 将数据发送到串口
            rospy.loginfo("----wait for begin-----")
            begin = nav.receive()
            if begin == 1:
                t0 = time.time()
                rospy.loginfo("game begin")
                rospy.loginfo("%s",t0)
                #print("receive", t0)
        else:
            """
            :param point:
            :param time_need:
            :param time_begin:
            :param waiting_time:需要设置等待时间，等待时间应当小于等于两次命令间时常
            :return: 1 成功
                     0 时间未到
                     2 超时失败
                     3 其他状态失败
            """
            # move_result2现在比赛时间 等于 现在时间戳 减 开始时间戳
            move_result0 = nav.pub_move_goal(point[0], 0, t0, 30)
            if move_result0 == 1:
                flag_reach0 = 1

            move_result1 = nav.pub_move_goal(point[1], 30, t0, 30)
            if move_result1 == 1:
                flag_reach1 = 1

            move_result2 = nav.pub_move_goal(point[2], 60, t0, 30)
            if move_result2 == 1:
                flag_reach2 = 1
            if move_result2 == 2 or move_result2 == 3:
                print("try again")
                move_result2 = nav.pub_move_goal(point[2], t0, t0, 30)

            move_result3 = nav.pub_move_goal(point[3], 90, t0, 30)
            if move_result3 == 1:
                flag_reach3 = 1

        rate.sleep()
    rospy.spin()  # 进入ROS的循环等待监听消息的到达和回调函数的调用
