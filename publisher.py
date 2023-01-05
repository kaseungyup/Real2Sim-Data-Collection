import rospy
from std_msgs.msg import String, Float32MultiArray

def apriltag_publisher(x_pos, y_pos, yaw):
    pub = rospy.Publisher('apriltag_position', String, queue_size=10)
    rospy.init_node('apriltag_publisher', anonymous=True)

    pos = "%s %s %s" % (x_pos, y_pos, yaw)
    pub.publish(pos)
    
def rpy_publisher(roll, pitch, yaw):
    pub = rospy.Publisher('rpy', String, queue_size=10)
    rospy.init_node('rpy_publisher', anonymous=True)

    rpy = "%s %s %s" % (roll, pitch, yaw)
    pub.publish(rpy)

def flag_publisher(flag):
    pub = rospy.Publisher('flag', String, queue_size=10)
    rospy.init_node('flag_publisher', anonymous=True)

    msg = "%s" % flag
    pub.publish(msg)

def simulation_publisher(sim_traj):
    pub = rospy.Publisher('simulation_position', Float32MultiArray, queue_size=10)
    rospy.init_node('simulation_publisher', anonymous=True)

    traj = sim_traj
    pub.publish(traj)

