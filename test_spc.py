import rospy
import numpy as np

from ros_np_multiarray import ros_np_multiarray as rnm
from classes.timer import Timer
from publisher import flag_publisher, simulation_publisher

rospy.init_node('spc', anonymous=True)

tmr_plot = Timer(_name='Plot',_HZ=1,_MAX_SEC=100,_VERBOSE=True)
tmr_plot.start()

test_traj = np.ones(shape=(502,2))
traj_data = rnm.to_multiarray_f32(test_traj)
print("Initialization complete")

while tmr_plot.is_notfinished():
    if tmr_plot.do_run():
        if tmr_plot.tick < 5:
            flag_publisher(0)

        else: 
            flag_publisher(1)
            simulation_publisher(traj_data)
            print(tmr_plot.tick)