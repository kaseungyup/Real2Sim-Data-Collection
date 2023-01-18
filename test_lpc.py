import os, time, rospy
import numpy as np

from time import time, localtime
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from classes.timer import Timer
from classes.visualizerclass import VisualizerClass
from publisher import IMURPYPublisher, AprilTagPublisher
from subscriber import FlagDataSubscriber, SimTrajSubscriber
from ros_np_multiarray import ros_np_multiarray as rnm


if __name__ == '__main__':

    Hz = 25
    desired_len = int(Hz*300/50)
    n_mahony = 10 # number of previous data for mahony filter
    D2R = np.pi/180
    R2D = 180/np.pi

    # Create folders
    CURR_FOLDER = os.getcwd()
    tm = localtime(time())
    DATA_FOLDER = os.path.join(CURR_FOLDER , "data")
    if not os.path.isdir(DATA_FOLDER): os.mkdir(DATA_FOLDER)
    DATA_FOLDER_TIME = os.path.join(CURR_FOLDER , "data", "%d%02d%02d-%02d:%02d:%02d"%(tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec))
    os.mkdir(DATA_FOLDER_TIME)
    stl_path = 'file://' + CURR_FOLDER + '/ROS_viz_engine/snapbot_low_resol.stl'

    # ROS, RViz initialization
    rospy.init_node('lpc', anonymous=True)
    print("1. Start visualization_engine.")
    V = VisualizerClass(name='simple viz',HZ=Hz)

    # Publisher, Subscriber initializatoin
    FlagData = FlagDataSubscriber()
    SimTraj  = SimTrajSubscriber()
    AprilTag = AprilTagPublisher()

    # Loop variables
    zero_tick = 0
    one_tick = 0
    epoch = 0
    n_real = 2
    
    # Apriltag data
    apriltag_batch = []
    prev_real_x, prev_real_y, prev_real_yaw = 0,0,0
    rpy_batch = []

    # Timer initialization
    tmr = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    tmr.start()
    print("2. Start Timer")

    while tmr.is_notfinished():
        if tmr.do_run():
            if FlagData.flag: # new trajectory starts
                if one_tick == 0: # reset variables and initialize
                    zero_tick = 0
                    one_tick += 1

                    # visualizer
                    V.reset_lines()
                    V.reset_meshes()

                    # variables to calculate and publish
                    xy_yaw_data = np.empty(shape=(0,3))

                    # simulation trajectory
                    sim_data = np.array(SimTraj.traj).reshape((SimTraj.length, SimTraj.height, SimTraj.num))
                    curr_traj = sim_data[epoch%n_real,:,:]
                    print(curr_traj.shape)
                    V.append_line(x_array=curr_traj[:,0],y_array=curr_traj[:,1],z=0.0,r=0.01,
                        frame_id='map',color=ColorRGBA(1.0,0.0,0.0,1.0),marker_type=Marker.LINE_STRIP)
                    print("4. Simulation data published")

                    # create folder
                    tm = localtime(time())
                    DATA_FOLDER_CURRENT = os.path.join(DATA_FOLDER_TRIAL, "%d. %d%02d%02d-%02d:%02d:%02d"%((epoch%n_real),tm.tm_year,tm.tm_mon,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec))
                    os.mkdir(DATA_FOLDER_CURRENT)
                    print("5. Ready to start")

                    # Check epoch
                    epoch += 1

                else: # while trajectory is running
                    one_tick += 1

                    # Calculate real x, y
                    real_x, real_y, real_yaw = np.sin(one_tick)/10, one_tick/40, one_tick/100
                    prev_real_x, prev_real_y, prev_real_yaw = real_x, real_y, real_yaw
                   
                    # Append data
                    xy_yaw_data = np.append(xy_yaw_data, np.array([[real_x, real_y, real_yaw]]), axis=0)

                    # Visualizer
                    V.delete_meshes()
                    V.append_mesh(x=real_x,y=real_y,z=0,scale=0.7,dae_path=stl_path,
                        frame_id='map', color=ColorRGBA(1.0,1.0,1.0,0.5),
                        roll=0,pitch=0*D2R,yaw=real_yaw)
                    V.publish_meshes()
                    V.publish_lines()


            else: # trajectory has ended 
                if epoch%n_real == 0: # start/end of each trial
                    if zero_tick == 0:
                        one_tick = 0
                        zero_tick += 1

                        # Create folders
                        DATA_FOLDER_TRIAL = os.path.join(DATA_FOLDER_TIME, "Trial %d"%(int(np.floor(epoch/n_real))))
                        os.mkdir(DATA_FOLDER_TRIAL)

                        if epoch != 0: # end of current trial
                            # Save apriltag data as .npy file
                            np.save(os.path.join(DATA_FOLDER_CURRENT, "xy_yaw.npy"), xy_yaw_data)

                            # Append apriltag data batch
                            if xy_yaw_data.shape[0] > desired_len:
                                apriltag_traj = xy_yaw_data[:desired_len,:]
                            else:
                                apriltag_traj = np.zeros(shape=(desired_len,xy_yaw_data.shape[1]))
                                apriltag_traj[:xy_yaw_data.shape[0],:] = xy_yaw_data
                                apriltag_traj[xy_yaw_data.shape[0]:,:] = xy_yaw_data[-1,:]

                            apriltag_traj = apriltag_traj - [apriltag_traj[0,0], apriltag_traj[0,1], 0]
                            apriltag_batch.append(apriltag_traj)
                            apriltag_batch_ros = rnm.to_multiarray_f32(np.array(apriltag_batch[-n_real:]))
                            
                            # Publish apriltag data to SPC
                            answer = str(input("Are you ready to publish data? (y/n): "))
                            while answer.lower() == 'y':
                                AprilTag.publish_once(apriltag_batch_ros)
                                answer = str(input("Continue? (y/n): "))
                                print("Publishing Data")

                        print("3. Waiting")
                        print("Start new cycle")

                    else:
                        # Subscribe flag/trajectory from SPC
                        flag = FlagData.flag
                        sim_len = SimTraj.length

                        if epoch != 0:
                            # Visualize real trajectory
                            V.append_line(x_array=apriltag_traj[:,0]-apriltag_traj[0,0],y_array=apriltag_traj[:,1]-apriltag_traj[0,1],z=0.0,r=0.01,
                                frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
                            V.publish_lines()
                                    
                else: # Waiting for next round
                    if zero_tick == 0:
                        one_tick = 0
                        zero_tick += 1
                        print("3. Waiting")
                        print("Number: ", epoch%n_real)

                        np.save(os.path.join(DATA_FOLDER_CURRENT, "xy_yaw.npy"), xy_yaw_data)

                        # Append apriltag data batch
                        if xy_yaw_data.shape[0] > desired_len:
                            apriltag_traj = xy_yaw_data[:desired_len,:]
                            print("Wrong trajectory length. Desired: %d, Actual: %d"%(desired_len,xy_yaw_data.shape[0]))
                            print("%d data removed."%(xy_yaw_data.shape[0]-desired_len))
                        else:
                            apriltag_traj = np.zeros(shape=(desired_len,xy_yaw_data.shape[1]))
                            apriltag_traj[:xy_yaw_data.shape[0],:] = xy_yaw_data
                            apriltag_traj[xy_yaw_data.shape[0]:,:] = xy_yaw_data[-1,:]
                            print("Wrong trajectory length. Desired: %d, Actual: %d"%(desired_len,xy_yaw_data.shape[0]))
                            print("%d data appended."%(desired_len-xy_yaw_data.shape[0]))

                        apriltag_traj = apriltag_traj - [apriltag_traj[0,0], apriltag_traj[0,1], 0]
                        apriltag_batch.append(apriltag_traj)
                    
                    else:
                        # Visualize real trajectory
                        V.append_line(x_array=apriltag_traj[:,0],y_array=apriltag_traj[:,1],z=0.0,r=0.01,
                            frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
                        V.publish_lines()

        rospy.sleep(1e-8)

    V.delete_meshes()
    V.delete_lines()

