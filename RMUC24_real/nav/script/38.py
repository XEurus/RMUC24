#!/usr/bin/env python
# coding=utf-8   #
import rospy
import struct
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
#from zimiao2.msg import ROSdata
#from std_msgs import float
#from std_msgs import bool
import math
import time
import os

class send_messges:
    baud=115200
    #ser=serial.Serial('/dev/ttyUSB1', baud, timeout=1)
    def __init__(self):
        self.linear_speed_x = 0  # 初始化数据变量为0
        self.angular_speed_z = 0
        self.yaw= 0
        self.pitch= 0
        #self.yaw2= 0
        #self.pitch2= 0
        self.dis= 0 
        self.shoot_model=10
        self.model=4
        self.is_aimed = False
        self.is_target_exists = False
        self.t1=0
        self.t2=0
        self.t3=0
        self.t4=0

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
            self.linear_speed_x=0
            self.angular_speed_z=0
        self.linear_speed_x = data.linear.x  # 将接收到的数据赋值给变量
        self.angular_speed_z = data.angular.z
        self.t3=time.time()
        self.send_data()  # 调用处理数据的方法

    def callback2(self, data):
        self.yaw = data.yaw 
        self.pitch = data.pitch
        self.dis = data.dis
        self.aimed = data.is_aimed
        self.target_exists = data.is_target_exists
        self.t1=time.time()
        self.send_data()  # 调用处理数据的方法
    

    def send_data(self):
        if self is None :  # 如果数据1和数据2都为None,给予赋值
            self.linear_speed = 0
            self.angular_speed= 0
            self.yaw= 0
            self.pitch= 0
            self.dis= 0
            self.is_aimed = False
            self.is_target_exists = False
            
        # 在这里处理接收到的数据
        rospy.loginfo("move_speed:%s 和%s,and %s,sm,%s",self.linear_speed_x,self.model,self.angular_speed_z,self.shoot_model)
        
        #rospy.loginfo("fight:%s 和%s,and %s,%s,and %s",self.yaw,self.pitch,self.dis,self.is_aimed,self.is_target_exists,)

        if self.yaw == 0 and self.pitch== 0 :
            self.t2=time.time()
            t=self.t2-self.t1
            print ('time',t)
            if t>=1:
                if self.linear_speed_x==0 and self.angular_speed_z==0:
                    self.t4=time.time()
                    t2=self.t4-self.t3
                    if t2>7:
                        self.model=2
                        self.shoot_model=10
                        t2=0
                        self.t3=0
                    else:
                        self.model=4
                        self.shoot_model=10
                        t=0
                        self.t1=0
            #self.t2=0
            else:
                self.linear_speed_x=0
                self.angular_speed_z=0
                self.model=2
                self.shoot_model=12
            
        data_bytes =struct.pack('fffiffi',
        self.yaw,
        self.pitch,
        self.dis,
        self.shoot_model,
        self.linear_speed_x,
        self.angular_speed_z,
        self.model)
        #rospy.loginfo("发送的字节：%s",data_bytes)
        #try:
            #ser.write(data_bytes) # 将数据发送到串口
            #ser.write(data_str.encode())
            
        #except serial.SerialException as e:
            #print('Serial communication error:', e)
            
            #ser.close()
            #try:
                
                #os.system('sudo chmod 777 /dev/ttyUSB*')
                #ser.close()
                #ser.open()
            #except serial.SerialException as e:
                #pass
            #print("new")
            
        #except ValueError:
            #rospy.logwarn("发送的数据格式错误！")
            #pass
            
            
        #except TypeError:
            #rospy.logwarn("发送类型错误")
            #pass
            
  	
        # 清除数据，以便下一次
        self.linear_speed = 0  
        self.angular_speed= 0
        self.yaw= 0
        self.pitch=0
        #self.t1=0
        self.dis= 0 
        self.shootmodel=10
        #self.model=10
        self.is_aimed = False
        self.is_target_exists = False


        try:
            #if ser.isOpen() :
            received_data = ser.read(9)  # 使用逗号分隔数据  # 读取9个字节的数据
            while len(received_data) == 9:
                unpacked_data = struct.unpack('ffs', received_data)  # 解包成两个浮点数

                self.x1 = float(unpacked_data[0])
                self.y1 = float(unpacked_data[1])
                # self.z1 = float(unpacked_data[2])
                if len(unpacked_data)==3:
                    self.c1 = unpacked_data[2]

                print('Received data:', self.x1,self.y1,self.c1)

                if self.x0!=self.x1 and self.y0!=self.y1 : # 判断发布

                    nav_goal = MoveBaseActionGoal()
                    nav_goal.goal.target_pose.header.stamp = rospy.Time.now()
                    nav_goal.goal.target_pose.header.frame_id = "map"
                    nav_goal.goal.target_pose.pose.position.x = self.x1
                    nav_goal.goal.target_pose.pose.position.y = self.y1
                    nav_goal.goal.target_pose.pose.orientation.w = -1.0
                    nav_goal.goal.target_pose.pose.orientation.z = 0

                    pub_nav = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
                    pub_nav.publish(nav_goal)
                    rospy.sleep(0.1)
                    rospy.loginfo("发布导航目标：x=%s, y=%s",self.x1,self.y1)

                    self.x0 = self.x1
                    self.y0 = self.y1
                break

        except ValueError:
            rospy.logwarn("接收到的数据格式错误！")
            pass
        #except TypeError:
         #   rospy.logwarn("接收类型错误")
          #  pass
            

        except serial.SerialException as e:
            print('Serial communication error:', e)
            
            #ser.close()
            #try:
                
                #os.system('sudo chmod 777 /dev/ttyUSB*')
                #ser.close()
                #ser.open()
                
            #except serial.SerialException as e:
                #print("-------faile--------")
                #pass
                
            print("new")
            pass

def publish_initial_pose():
    rospy.init_node('amcl_initial_pose_publisher', anonymous=True)
    pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(0.5)  # 等待1秒，确保发布者和订阅者之间建立连接

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"

    # 设置初始位置的坐标
    initial_pose.pose.pose.position.x = 22  # 设置x坐标
    initial_pose.pose.pose.position.y = 8.5  # 设置y坐标
    initial_pose.pose.pose.position.z = 0.0  # 设置z坐标

    # 设置初始位置的方向
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = -1.0
    initial_pose.pose.pose.orientation.w = 0

    pub_amcl.publish(initial_pose)
    rospy.loginfo("发布初始位置%s,%s",initial_pose.pose.pose.position.x,initial_pose.pose.pose.position.y)


    number=0  # 发布初始化激活
    q=struct.pack('i',number)
    ser.write(q)

    #received_data = ser.read(9)  # 使用逗号分隔数据  # 读取9个字节的数据
    #while len(received_data) == 9:
    #    unpacked_data = struct.unpack('ffs', received_data)  # 解包成两个浮点数
#
 #       x1 = float(unpacked_data[0])
  #      y1 = float(unpacked_data[1])
  #      # self.z1 = float(unpacked_data[2])
   #     if len(unpacked_data) == 3:
    #        c1 = unpacked_data[2]
#
 #       nav_goal = MoveBaseActionGoal()
  #      nav_goal.goal.target_pose.header.stamp = rospy.Time.now()
   #     nav_goal.goal.target_pose.header.frame_id = "map"
    #    nav_goal.goal.target_pose.pose.position.x = x1
     #   nav_goal.goal.target_pose.pose.position.y = y1
      #  nav_goal.goal.target_pose.pose.orientation.w = 1.0

       # pub_nav = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        #pub_nav.publish(nav_goal)
        ##rospy.sleep(0.1)
        #rospy.loginfo("发布导航目标：x=%s, y=%s", x1, y1)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_serial_node')
    baud = 115200
    # 创建一个serial对象
    global ser
    ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
    # ser = serial.Serial('/dev/ttyUSB1', baud, timeout=1)
    rospy.loginfo("波特率：%s", baud)
    #try:
        #publish_initial_pose()
    #except rospy.ROSInterruptException:
        #pass
    #publish_initial_pose()

    #global model=4
    #global shoot_model=0
    
    # 创建一个ROS节点

    send = send_messges()  # 创建DataProcessor类的实例
    rospy.Subscriber('/cmd_vel', Twist, send.callback1,queue_size=10)
    # 创建订阅者，订阅'topic1'话题，回调函数为callback1
   # if cmd.get_num_connections()<=0:
       # send.send_data()
    
    #rospy.Subscriber('/rosdata', ROSdata, send.callback2,queue_size=10) # 创建订阅者，订阅'topic2'话题，回调函数为callback2
    send.send_data()
    rate=rospy.Rate(3)
   #timer=rospy.Timer(rospy.Duration(0.1),send.send_data)
    while not rospy.is_shutdown():
       # send.send_data()
        send.shoot_model=10
        baud = 115200
        ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)

        number=0  # 发布初始化激活
        q=struct.pack('i',number)
        ser.write(q)
        #rate.sleep()
    rospy.spin()  # 进入ROS的循环等待监听消息的到达和回调函数的调用                                                                          
