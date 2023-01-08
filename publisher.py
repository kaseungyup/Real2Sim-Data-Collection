import rospy
from std_msgs.msg import String, Float32MultiArray

def apriltag_publisher(apriltag_data):
    pub = rospy.Publisher('apriltag_position', Float32MultiArray, queue_size=10)
    pub.publish(apriltag_data)
    
def rpy_publisher(rpy_data):
    pub = rospy.Publisher('rpy', Float32MultiArray, queue_size=10)
    pub.publish(rpy_data)

def flag_publisher(flag):
    pub = rospy.Publisher('flag', String, queue_size=10)

    msg = "%s" % flag
    pub.publish(msg)

def simulation_publisher(sim_traj):
    pub = rospy.Publisher('simulation_position', Float32MultiArray, queue_size=10)
    pub.publish(sim_traj)

