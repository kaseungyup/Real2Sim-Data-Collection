import os, time, rospy
import numpy as np

from classes.timer import Timer
from publisher import FlagDataPublisher, SimTrajPublisher
from subscriber import AprilTagSubscriber, IMURPYSubscriber
from ros_np_multiarray import ros_np_multiarray as rnm

if __name__ == '__main__':

    Hz = 50
    max_sec = 6
    n_real_roll = 2

    rospy.init_node('spc', anonymous=True)
    FlagDataPublisher  = FlagDataPublisher()
    SimTrajPublisher   = SimTrajPublisher()

    test_traj = np.load("test_xy_yaw.npy")
    test_traj = test_traj - test_traj[0,:]
    sim_traj = []
    for i in range(n_real_roll):
        sim_traj.append(test_traj)

    traj_publish = rnm.to_multiarray_f32(np.array(sim_traj))

    while True:
        FlagDataPublisher.publish_once(0)

        answer = str(input("Do you want to roll out real world? (y/n): "))
        if answer.lower() == 'y':
            SimTrajPublisher.publish_once(traj_publish)

            for real_idx in range(n_real_roll):
                answer = str(input("Do you prepare your snapbot? (y/n): "))
                if answer.lower() == 'y':
                    tmr = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=max_sec)
                    tmr.start()
                    while tmr.is_notfinished():
                        if tmr.do_run():
                            FlagDataPublisher.publish_once(1)
                    tmr.end()
                    FlagDataPublisher.publish_once(0)


            aprilTagSubscriber = AprilTagSubscriber()
            # imurpySubscriber = IMURPYSubscriber()

            if aprilTagSubscriber.traj != []:
                real_traj = np.array(aprilTagSubscriber.traj).reshape(aprilTagSubscriber.length, aprilTagSubscriber.height, -1)
                print("real traj shape: ", real_traj.shape)

            # if imurpySubscriber.traj != []:
            #     rpy_data = np.array(imurpySubscriber.traj).reshape(imurpySubscriber.length, imurpySubscriber.height, -1)
            #     print("final orientation || roll: %.2f, pitch: %.2f, yaw: %.2f"%(rpy_data[-1,-1,0],rpy_data[-1,-1,1],rpy_data[-1,-1,2]))
