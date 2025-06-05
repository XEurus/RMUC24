#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import pcl_helper
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf
import time

def pcl_to_ros(pcl_array):
    return pcl_helper.pcl_to_ros(pcl_array)

def ros_to_pcl(ros_msg):
    return pcl_helper.ros_to_pcl(ros_msg)

def icp_match(source_cloud_pcl, target_cloud_pcl):
    icp = source_cloud_pcl.make_IterativeClosestPoint()
    icp.set_MaximumIterations(100)  # 设置最大迭代次数为100
    icp.set_TransformationEpsilon(1e-3)  # 设置变换停止的容差
    icp.set_EuclideanFitnessEpsilon(1e-3)  # 设置欧几里得适应度停止的容差
    print("开始匹配")
    t1=time.time()
    converged, transf, estimate, fitness = icp.icp(source_cloud_pcl, target_cloud_pcl)
    print("匹配时间：",time.time()-t1)
    return transf, fitness if converged else (None, 0)

class RobotRelocator:
    def __init__(self, robot_laser_topic, static_map_topic, initialpose_topic):
        #elf.robot_cloud_topic = robot_cloud_topic
        self.static_map_topic = static_map_topic
        self.robot_laser_topic= robot_laser_topic
        self.initialpose_topic = initialpose_topic
        self.map_cloud = None
        self.robot_cloud = None
        self.last_publish_time = rospy.Time.now()
        self.publish_interval = rospy.Duration(30)  # Min publish interval: 30 seconds

    def initialize_subscribers_and_publishers(self):
        self.laser_sub = rospy.Subscriber(self.robot_laser_topic, LaserScan, self.scan_callback)
        self.laser_projector = LaserProjection()
        #self.cloud_sub = rospy.Subscriber(self.robot_cloud_topic, PointCloud2, self.cloud_callback)
        self.map_sub = rospy.Subscriber(self.static_map_topic, PointCloud2, self.map_callback)
        self.initialpose_pub = rospy.Publisher(self.initialpose_topic, PoseWithCovarianceStamped, queue_size=1)

    def map_callback(self, msg):
        self.map_cloud = msg

    def scan_callback(self, msg):
        # 使用LaserProjection将LaserScan转换为PointCloud2
        self.robot_cloud =self.laser_projector.projectLaser(msg)
        
    def ipc_caculate(self):
        robot_cloud=ros_to_pcl(self.robot_cloud)
        map_cloud=ros_to_pcl(self.map_cloud)
        transformation, fitness = icp_match(robot_cloud, map_cloud)
        self.publish_initial_pose(transformation)
                

    def publish_initial_pose(self, transformation):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"

        initial_pose.pose.pose.position.x = transformation[0, 3]
        initial_pose.pose.pose.position.y = transformation[1, 3]
        initial_pose.pose.pose.position.z = transformation[2, 3]

        quaternion = tf.transformations.quaternion_from_matrix(transformation)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]

        self.initialpose_pub.publish(initial_pose)

def main():
    rospy.init_node('robot_relocator', anonymous=True)
    relocator = RobotRelocator('/robot_cloud', '/static_map', '/initialpose')
    relocator.initialize_subscribers_and_publishers()
    #rospy.spin()

if __name__ == '__main__':
    main()
