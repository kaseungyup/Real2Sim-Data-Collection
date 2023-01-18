import rospy
from std_msgs.msg import String, Float32MultiArray
from classes.timer import Timer

class IMURPYSubscriber():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num = 0
        self.traj = []
        self.isReady_rpy = False
        self.init_subscriber()

    def init_subscriber(self):
        self.imu_rpy_sub = rospy.Subscriber("imu_rpy", Float32MultiArray, self.callback)
        while self.isReady_rpy is False: rospy.sleep(1e-3)

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
        self.isReady_tag = False
        self.init_subscriber()

    def init_subscriber(self):
        self.april_traj_sub = rospy.Subscriber("apriltag_trajectory", Float32MultiArray, self.callback)
        while self.isReady_tag is False: rospy.sleep(1e-3)

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
    
if __name__ == "__main__":


    rospy.init_node(name='subscriber', anonymous=True)    
    # FlagData = FlagDataSubscriber()
    # SimTraj  = SimTrajSubscriber()
    # tick     = 0
    # timer = Timer(HZ=1, MAX_SEC=100)
    # timer.start()
    # while timer.is_notfinished():
    #     if timer.do_run():
    #         print(FlagData.flag)

    apriltag = AprilTagSubscriber()
    print(apriltag.length)