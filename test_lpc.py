import rospy
import numpy as np

from classes.timer import Timer
from publisher import apriltag_publisher, rpy_publisher
from subscriber import FlagData, SimulationData

rospy.init_node('lpc', anonymous=True)
tmr_plot = Timer(_name='Plot',_HZ=1,_MAX_SEC=np.inf,_VERBOSE=True)
tmr_plot.start()
flag = FlagData()

while tmr_plot.is_notfinished():
    while tmr_plot.do_run():
        tick = tmr_plot.tick
        print("STEP 1")

        if flag.flag:
            apriltag_publisher(0.01*tick, 0.01*tick, 0.01*tick)
            print("STEP 2")
            sim_traj = SimulationData()
            sim_data = np.array(sim_traj.traj).reshape((sim_traj.length, sim_traj.height))
            print(sim_data.shape)
        else:
            apriltag_publisher(0,0,0)


        # rpy_publisher(0.02*tick, 0.02*tick, 0.02*tick)

