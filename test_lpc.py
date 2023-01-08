import rospy
import numpy as np

from classes.timer import Timer
from subscriber import FlagData, SimulationData

rospy.init_node('lpc', anonymous=True)
tmr_plot = Timer(_name='Plot',_HZ=1,_MAX_SEC=np.inf,_VERBOSE=True)
tmr_plot.start()
flag = FlagData()

zero_tick = 0
one_tick = 0

while tmr_plot.is_notfinished():
    if tmr_plot.do_run():
        if flag.flag:
            if one_tick == 0: # reset variables and initialize
                zero_tick = 0
                one_tick += 1
                print("check 2")
            else:
                sim_traj = SimulationData()
                sim_data = np.array(sim_traj.traj).reshape((sim_traj.length, sim_traj.height))
                print(sim_data.shape)

        else:
            if zero_tick == 0: # trajectory just ended
                one_tick = 0
                zero_tick += 1
                print("check 1")
            else:
                print(tmr_plot.tick)

