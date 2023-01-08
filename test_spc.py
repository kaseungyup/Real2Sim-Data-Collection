import rospy
import numpy as np
from ros_np_multiarray import ros_np_multiarray as rnm

from classes.timer import Timer
from publisher import flag_publisher, simulation_publisher
from subscriber import ApriltagData, RPYData

rospy.init_node('spc', anonymous=True)

tmr_plot = Timer(_name='Plot',_HZ=1,_MAX_SEC=100,_VERBOSE=True)
tmr_plot.start()

test_traj = np.load("test_traj.npy")
data = rnm.to_multiarray_f32(test_traj)
print("Initialization complete")

tick = 0

while tmr_plot.is_notfinished():
    if tmr_plot.do_run():
        if tick < 5:
            print("STEP 1")
            flag_publisher(0)
            tick += 1

        else: 
            print("STEP 2")
            flag_publisher(1)
            simulation_publisher(data)
            apriltag = ApriltagData()
            # rpy = RPYData()

            apriltag_current = np.array([[apriltag.x, apriltag.y, apriltag.yaw]])
            print("apriltag data: ", apriltag_current)
            
            # rpy_current = np.array([[rpy.r_data, rpy.p_data, rpy.y_data]])
            # print("rpy data: ", rpy_current)
