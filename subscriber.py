import rospy
from std_msgs.msg import String, Float32MultiArray
from classes.timer import Timer

class IMURPYSubscriber():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num = 0
        self.traj = []
        self.imu_rpy_sub = rospy.Subscriber("imu_rpy", Float32MultiArray, self.callback)

    def callback(self, data):
        self.isReady_rpy = True
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.num = int(data.layout.dim[2].size)
        self.traj = data.data

class AprilTagSubscriber():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num = 0
        self.traj = []
        self.april_traj_sub = rospy.Subscriber("apriltag_trajectory", Float32MultiArray, self.callback)

    def callback(self, data):
        self.isReady_tag = True
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.num = int(data.layout.dim[2].size)
        self.traj = data.data

class FlagDataSubscriber():
    def __init__(self):
        self.flag     = 0
        self.flag_sub = rospy.Subscriber("flag", String, self.callback)
    
    def callback(self, data):
        self.flag = int(data.data.split()[0])
    
class SimTrajSubscriber():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num    = 0
        self.traj = []
        self.sim_traj_sub = rospy.Subscriber("simulation_trajectory", Float32MultiArray, self.callback)

    def callback(self, data):
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.num    = int(data.layout.dim[2].size)
        self.traj = data.data

class AnchorSubscriber():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.traj = []
        self.sim_traj_sub = rospy.Subscriber("anchor", Float32MultiArray, self.callback)

    def callback(self, data):
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.traj = data.data
    
if __name__ == "__main__":

    # def callback(data):
    #     # print("HI")
    #     print(data.data.split()[0])
    # rospy.init_node(name="subscribe", anonymous=True)
    # listen = rospy.Subscriber("flag", String, callback)
    # rospy.spin()
    # # while True:
    # while True:
    rospy.init_node(name='subscriber', anonymous=True)    
    FlagData = FlagDataSubscriber()
    SimTraj  = SimTrajSubscriber()
    while True:
        print(FlagData.flag)