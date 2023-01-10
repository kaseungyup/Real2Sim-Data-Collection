import rospy
from std_msgs.msg import String, Float32MultiArray

class ApriltagData():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num = 0
        self.traj = []
        self.isReady_tag = False
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_tag = "apriltag_position"
        self.sub_tag = rospy.Subscriber(self.topic_sub_tag, Float32MultiArray, self.callback)
        while self.isReady_tag is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_tag = True
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.num = int(data.layout.dim[2].size)
        self.traj = data.data

class RPYData():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num = 0
        self.traj = []
        self.isReady_rpy = False
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_rpy = "rpy"
        self.sub_rpy = rospy.Subscriber(self.topic_sub_rpy, Float32MultiArray, self.callback)
        while self.isReady_rpy is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_rpy = True
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.num = int(data.layout.dim[2].size)
        self.traj = data.data

class FlagData():
    def __init__(self):
        self.flag = 0
        self.isReady_flag = False
        self.init_subscriber()
    
    def init_subscriber(self):
        self.topic_sub_flag = "flag"
        self.sub_flag = rospy.Subscriber(self.topic_sub_flag, String, self.callback)
        while self.isReady_flag is False: rospy.sleep(1e-3)
    
    def callback(self, data):
        self.isReady_flag = True
        array = data.data.split()
        self.flag = int(array[0])

class SimulationData():
    def __init__(self):
        self.length = 0
        self.height = 0
        self.num = 0
        self.traj = []
        self.isReady_sim = False
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_sim = "simulation_position"
        self.sub_tag = rospy.Subscriber(self.topic_sub_sim, Float32MultiArray, self.callback)
        while self.isReady_sim is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_sim = True
        self.length = int(data.layout.dim[0].size)
        self.height = int(data.layout.dim[1].size)
        self.num = int(data.layout.dim[2].size)
        self.traj = data.data