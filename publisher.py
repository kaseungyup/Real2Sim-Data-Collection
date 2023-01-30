import rospy, sys, os, time
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))
from std_msgs.msg import String, Float32MultiArray
from ros_np_multiarray import ros_np_multiarray as rnm

class IMURPYPublisher():
    def __init__(self):
        self.imu_rpy_pub = rospy.Publisher('imu_rpy', Float32MultiArray, queue_size=10)

    def run(self, data):
        self.imu_rpy_pub.publish(data)

    def publish_once(self, data):
         while not rospy.is_shutdown():
            rate = rospy.Rate(30)
            connections = self.imu_rpy_pub.get_num_connections()
            if connections > 0:
                self.run(data)
                rospy.loginfo('{} Published'.format(data))
                break

class AprilTagPublisher():
    def __init__(self):
        self.april_traj_pub = rospy.Publisher('apriltag_trajectory', Float32MultiArray, queue_size=10)

    def run(self, data):
        self.april_traj_pub.publish(data)

    def publish_once(self, data):
         while not rospy.is_shutdown():
            rate = rospy.Rate(30)
            connections = self.april_traj_pub.get_num_connections()
            if connections > 0:
                self.run(data)
                rospy.loginfo('{} Published'.format(data))
                break

class FlagDataPublisher():
    def __init__(self):
        self.flag_pub = rospy.Publisher('flag', String, queue_size=10)

    def run(self, data):
        data = "%s" % data
        self.flag_pub.publish(data)

    def publish_once(self, data):
        while not rospy.is_shutdown():
            rate = rospy.Rate(30)
            connections = self.flag_pub.get_num_connections()
            if connections > 0:
                self.run(data)
                rospy.loginfo('{} Published'.format(data))
                break

class SimTrajPublisher():
    def __init__(self):
        self.sim_traj_pub = rospy.Publisher('simulation_trajectory', Float32MultiArray, queue_size=10)

    def run(self, data):
        self.sim_traj_pub.publish(data)

    def publish_once(self, data):
         while not rospy.is_shutdown():
            rate = rospy.Rate(30)
            connections = self.sim_traj_pub.get_num_connections()
            if connections > 0:
                self.run(data)
                rospy.loginfo('{} Published'.format(data))
                break

class AnchorPublisher():
    def __init__(self):
        self.anchor_pub = rospy.Publisher('anchor', Float32MultiArray, queue_size=10)

    def run(self, data):
        self.anchor_pub.publish(data)

    def publish_once(self, data):
         while not rospy.is_shutdown():
            rate = rospy.Rate(30)
            connections = self.anchor_pub.get_num_connections()
            if connections > 0:
                self.run(data)
                rospy.loginfo('{} Published'.format(data))
                break
            
if __name__ == "__main__":
    rospy.init_node('publisher', anonymous=True)
    test_traj = rnm.to_multiarray_f32(np.array([[[1,2,3],[4,5,6],[7,8,9]]]).reshape(3,3,1))
    Flag = FlagDataPublisher()
    SimTraj = SimTrajPublisher()
    for i in range(100):
        SimTraj.publish_once(test_traj)
        Flag.publish_once(5)       