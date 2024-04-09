#!/usr/bin/env python
# coding=GB2312  
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
from multiprocessing import Process
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
[-0.497, -0.063,  -0.033,],
[1.539, -0.202, -0.013]
]

begin =0 
t0=0


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

    def pub_move_goal(self, point, time_need, time_start, waiting_time):  # ����x y w ��ʹ��rospy.spin()����
        """
        :param point: x���� y���� w:�����ƽǶȣ�rviz�ڷ�����set_pose������ֱ�Ӷ���
        :param time_need:
        :param time_start: ������ʼʱ��
        :param waiting_time:��Ҫ���õȴ�ʱ�䣬�ȴ�ʱ��Ӧ��С�ڵ������������ʱ��
        :return: 1 �ɹ�
                 0 ʱ��δ��
                 2 ��ʱʧ��
                 3 ����״̬ʧ��
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

            q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi)  # ֻ����yaw����ת�Ƕ�
            # q = transformations.quaternion_from_euler(0.0, 0.0, w / 180.0 * pi)
            nav_goal.target_pose.pose.orientation.x = q[0]
            nav_goal.target_pose.pose.orientation.y = q[1]
            nav_goal.target_pose.pose.orientation.z = q[2]
            nav_goal.target_pose.pose.orientation.w = q[3]

            # ���͵���Ŀ���MoveBase action�������������ûص��������������ͷ���
            self.move_base.send_goal(nav_goal, self._done_cb, self._active_cb)

            result = self.move_base.wait_for_result(rospy.Duration(waiting_time))  # �ȴ�����Ŀ�����ɣ����ó�ʱʱ��Ϊ45��

            if not result:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                return_flag = 2
            else:
                state = self.move_base.get_state()  # ��ȡ����������״̬
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("reach goal %s succeeded!" % point)
                    return_flag = 1
                else:
                    return_flag = 3

        else:
            return_flag = 0

        return return_flag

    def receive_start(self):
        try:
            received_data = ser.read(3)  #  ��ȡ3���ֽڵ�����
            print("receive")
            if len(received_data) == 3:

                unpacked_data = struct.unpack('Hs', received_data)  # ���������������

                game_t = unpacked_data[0]
                self.begin = unpacked_data[1]

                rospy.loginfo("Received data: %s,%s" , game_t ,self.begin)
                # print('Received data:', self.begin,self.t1)
                if game_t != 0:
                    rospy.loginfo("game start")
                    return 1

            return 0
        except ValueError:
            rospy.logwarn("���յ������ݸ�ʽ����")
            return 0

        except serial.SerialException as e:
            rospy.loginfo("error serial")
            return 0



class send_messges:
    baud = 115200

    # ser=serial.Serial('/dev/USB0', baud, timeout=1)
    def __init__(self):
        self.linear_speed_x = 0  # ��ʼ�����ݱ���Ϊ0
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
            self.linear_speed_x = data.linear.x  # �����յ������ݸ�ֵ������
            self.linear_speed_y = data.linear.y
            self.angular_speed_z = data.angular.z
            self.t2=time.time()
            #self.send_data()  # ���ô������ݵķ���

    def callback2(self, data):
        self.yaw = data.yaw
        self.pitch = data.pitch
        self.dis = data.dis
        self.aimed = data.is_aimed
        self.target_exists = data.is_target_exists
        self.t1 = time.time()
        #self.send_data()  # ���ô������ݵķ���

    def send_data(self):
        if self is None:  # �������1������2��ΪNone,���踳ֵ
            self.linear_speed_x = 0
            self.linear_speed_y = 0
            self.angular_speed_z = 0
            self.yaw = 0
            self.pitch = 0
            self.dis = 0
            self.is_aimed = False
            self.is_target_exists = False

        # �����ﴦ����յ�������
        #rospy.loginfo("move_speed:%s ��%s,and %s,sm,%s",
         #             self.linear_speed_x, 
         #             self.angular_speed_z,
         #             self.shoot_model)

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
        rospy.loginfo("���͵��ֽڣ�%s,%s,%s,%s,%s,%s,%s,%s",self.yaw,self.pitch, self.dis,self.linear_speed_x,self.linear_speed_y,self.angular_speed_z,self.shoot_model,self.model)

        try:
            ser.write(data_bytes)  # �����ݷ��͵�����

        except serial.SerialException as e:
            print('Serial communication error:', e)

            ser.close()
            ser.open()
            print("new")
            pass

        except ValueError:
            rospy.logwarn("���յ������ݸ�ʽ����")
            pass

        # ������ݣ��Ա���һ��
        self.linear_speed_x = 0
        self.linear_speed_y = 0
        self.angular_speed_z = 0
        self.yaw = 0
        self.pitch = 0
        self.dis = 0
        #self.shootmodel = 0
        # self.model=10
        self.is_aimed = False
        self.is_target_exists = False

    def receive(self):
        try:
            received_data = ser.read(3)  # ʹ�ö��ŷָ�����  # ��ȡ9���ֽڵ�����
            #print(len(received_data))
            if len(received_data) == 3:
                unpacked_data = struct.unpack('Hs', received_data)  # ���������������
                self.begin = unpacked_data[0]
                print('Received data:', self.begin)

                # break

        except ValueError:
            rospy.logwarn("���յ������ݸ�ʽ����")
            pass
        except serial.SerialException as e:
            rospy.logwarn('Serial communication error:', e)
            #   print('Serial communication error:', e)
            #   serial_port.close()
            #   serial_port.open()
            #   print("new")
            pass

    def model_decision(self):
        t1=time.time()-self.t1
        t2=time.time()-self.t2
        #print(t1,t2)
        if self.yaw!=0 or self.pitch!=0:
            self.shoot_model=1
            self.model=2
        elif self.yaw == 0 and self.pitch==0 and (self.linear_speed_x != 0 or self.linear_speed_y != 0 or self.angular_speed_z != 0) and t1>0.3:
            self.shoot_model=0
            self.model=4
        elif self.yaw == 0 and self.pitch==0 and (self.linear_speed_x != 0 or self.linear_speed_y != 0 or self.angular_speed_z != 0) and t1<0.3:
            self.shoot_model=1
            self.model=2
        elif self.yaw == 0 and self.pitch==0 and (self.linear_speed_x == 0 and self.linear_speed_y == 0 and self.angular_speed_z == 0) and t1>0.3 and t2>2:
            self.shoot_model=0
            self.model=2
        #rospy.loginfo("model %s,%s",self.model,self.shoot_model)
        self.send_data()

def decision(ser_msg_queue):
        nav = navigation()
        flag_reach0 = 0
        flag_reach1 = 0
        flag_reach2 = 0
        flag_reach3 = 0
        flag_reach4 = 0
        flag_reach5 = 0
        begin=0
        t0=0
    
        while True:
            time.sleep(0.1)
            if begin!=1:
                time.sleep(0.1)
                begin=ser_msg_queue.get()
                if begin == 1:
                    t0 = time.time()
                    rospy.loginfo("game begin")
                    rospy.loginfo("%s",t0)

            else:
                
                """
                :param point: Ҫȥ�ĵ�
                :param time_need: ʲôʱ��ȥ
                :param time_begin: ����ʲôʱ��ʼ
                :param waiting_time:��Ҫ���õȴ�ʱ�䣬�ȴ�ʱ��Ӧ��С�ڵ������������ʱ��
                :return: 1 �ɹ�
                        0 ʱ��δ��
                        2 ��ʱʧ��
                        3 ����״̬ʧ��
                """
                # move_result2���ڱ���ʱ�� ���� ����ʱ��� �� ��ʼʱ���
                move_result0 = nav.pub_move_goal(point[0], 0, t0, 30)
                if move_result0 == 1:
                    flag_reach0 = 1

                move_result1 = nav.pub_move_goal(point[1], 30, t0, 30)
                if move_result1 == 1:
                    flag_reach1 = 1

                #move_result2 = nav.pub_move_goal(point[2], 60, t0, 30)
                #if move_result2 == 1:
                    #flag_reach2 = 1
                #if move_result2 == 2 or move_result2 == 3:
                    #print("try again")
                    #move_result2 = nav.pub_move_goal(point[2], t0, t0, 30)

                #move_result3 = nav.pub_move_goal(point[3], 90, t0, 30)
                #if move_result3 == 1:
                    #flag_reach3 = 1


def publish_initial_pose():
    # rospy.init_node('amcl_initial_pose_publisher', anonymous=True)
    pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(0.5)  # �ȴ�1�룬ȷ�������ߺͶ�����֮�佨������

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"

    # ���ó�ʼλ�õ�����
    initial_pose.pose.pose.position.x = 7.0  # ����x����
    initial_pose.pose.pose.position.y = 8.5  # ����y����
    initial_pose.pose.pose.position.z = 0.0  # ����z����

    # ���ó�ʼλ�õķ���
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    pub_amcl.publish(initial_pose)
    #rospy.loginfo("������ʼλ��%s,%s", initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y)

    #number = 0  # ������ʼ������ ֮ǰʹ�ó�ʼ������󣬵ȴ�c���Ӧ�����·�
    #q = struct.pack('i', number)
    #ser.write(q)


if __name__ == '__main__':

    rospy.init_node('cmd_vel_to_serial_node')
    global ser
    baud = 115200
    ser = serial.Serial('/dev/ttyUSB1', baud, timeout=0.1)
    rospy.loginfo("�����ʣ�%s", baud)

    ser_msg_queue =Queue()

    #detect_init()

    move_decision=Process(target=decision,args=(ser_msg_queue,))
    

    #nav = navigation()
    send = send_messges()  # �������ʵ��
    
    # try:
        # publish_initial_pose()
    # except rospy.ROSInterruptException:
        # pass
    move_decision.start()
    rospy.Subscriber('/cmd_vel', Twist, send.callback1, queue_size=10)  
    rospy.Subscriber('/rosdata', ROSdata, send.callback2,queue_size=10) #������Ϣ
    rate = rospy.Rate(20)  # max of frame 80-90
    
    nav = navigation()
    begin=0
    while not rospy.is_shutdown(): #����
        send.model_decision()
        if begin!=1:
            begin=nav.receive_start()
            ser_msg_queue.put(begin)
        #time.sleep(0.5)
        
        rate.sleep()
    rospy.spin()  # ����ROS��ѭ���ȴ�������Ϣ�ĵ���ͻص������ĵ���
