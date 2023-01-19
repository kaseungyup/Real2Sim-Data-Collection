import os, serial, rospy
import numpy as np

from time import time, localtime
from collections import deque
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from classes.timer import Timer
from classes.visualizerclass import VisualizerClass
from filter.mahony import Mahony
from utils.utils_real import quaternion_to_vector
from publisher import IMURPYPublisher, AprilTagPublisher
from subscriber import FlagDataSubscriber, SimTrajSubscriber
from ros_np_multiarray import ros_np_multiarray as rnm


if __name__ == '__main__':

    Hz = 25
    desired_len = int(Hz*300/50)
    n_mahony = 40 # number of previous data for mahony filter
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

    # IMU variables
    IMU_USB_NUMBER = 0
    ser = serial.Serial('/dev/ttyUSB{}'.format(IMU_USB_NUMBER), 115200, timeout=1)
    IMU_REAL_TIME = True

    # ROS, RViz initialization
    rospy.init_node('lpc', anonymous=True)
    print("1. Start visualization_engine.")
    V = VisualizerClass(name='simple viz',HZ=Hz)

    # Publisher, Subscriber initializatoin
    FlagData = FlagDataSubscriber()
    SimTraj  = SimTrajSubscriber()
    AprilTag = AprilTagPublisher()
    IMURPY = IMURPYPublisher()

    # Loop variables
    zero_tick = 0
    one_tick = 0
    epoch = 0
    n_real = 2
    
    # Apriltag data
    apriltag_batch = []
    prev_real_x, prev_real_y, prev_real_yaw = 0,0,0

    rpy_batch = []
    acc_array = [0,0,0]
    gyro_array = [0,0,0]

    # Timer initialization
    tmr = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    tmr.start()
    print("2. Start Timer")

    while tmr.is_notfinished():
        if tmr.do_run():
            if FlagData.flag: # new trajectory starts
                if one_tick == 0: # reset variables and initialize
                    zero_tick = 0

                    # visualizer
                    V.reset_lines()
                    V.reset_meshes()

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

                # Calculate real x, y
                real_x, real_y, real_yaw = 0, one_tick/40, one_tick/100
                prev_real_x, prev_real_y, prev_real_yaw = real_x, real_y, real_yaw

                # Read IMU data
                line = ser.readline()
                imu_raw_data = line.decode('unicode_escape')
                imu_raw_data = imu_raw_data.split()
                
                try:
                    acc_array = [-float(imu_raw_data[1].replace('\x00','')), float(imu_raw_data[0].replace('\x00','')), float(imu_raw_data[2].replace('\x00',''))]
                    gyro_array = [float(imu_raw_data[3].replace('\x00','')), float(imu_raw_data[4].replace('\x00','')), float(imu_raw_data[5].replace('\x00',''))]
                except:
                    pass

                acc_data.append(acc_array)
                gyro_data.append(gyro_array)                        

                # Calculate RPY data
                if IMU_REAL_TIME:
                    if len(acc_data) > n_mahony:
                        acc_data.popleft()
                    if len(gyro_data) > n_mahony:
                        gyro_data.popleft()

                    orientation_mahony = Mahony(gyr=list(gyro_data), acc=list(acc_data))
                    q_mahony = orientation_mahony.Q[-1,:]
                    roll_raw, pitch_raw, yaw_raw = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
                    yaw_val += yaw_raw

                    roll_data = (np.pi-roll_raw)*R2D # degrees
                    pitch_data = -pitch_raw*R2D # degrees
                    yaw_data = yaw_val*100/Hz/n_mahony*R2D # degrees
                
                # Append data
                xy_yaw_data = np.append(xy_yaw_data, np.array([[real_x, real_y, real_yaw]]), axis=0)
                if IMU_REAL_TIME:
                    rpy_data = np.append(rpy_data, np.array([[roll_data, pitch_data, yaw_data]]), axis=0)
                    print(rpy_data[-1,:])

                # Visualizer
                V.delete_meshes()
                if IMU_REAL_TIME:
                    V.append_mesh(x=real_x-xy_yaw_data[0,0],y=real_y-xy_yaw_data[0,1],z=0,scale=1.0,dae_path=stl_path,
                        frame_id='map', color=ColorRGBA(1.0,1.0,1.0,0.5),
                        roll=roll_data*D2R,pitch=pitch_data*D2R,yaw=yaw_data*D2R)
                else:
                    V.append_mesh(x=real_x-xy_yaw_data[0,0],y=real_y-xy_yaw_data[0,1],z=0,scale=1.0,dae_path=stl_path,
                        frame_id='map', color=ColorRGBA(1.0,1.0,1.0,0.5),
                        roll=0,pitch=0,yaw=real_yaw)
                V.publish_meshes()
                V.publish_lines()

                one_tick += 1


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
                            np.save(os.path.join(DATA_FOLDER_CURRENT, "rpy.npy"), rpy_data)

                            # Append data batch
                            if xy_yaw_data.shape[0] > desired_len:
                                apriltag_traj = xy_yaw_data[:desired_len,:]
                                print("Wrong apriltag trajectory length. Desired: %d, Actual: %d"%(desired_len,xy_yaw_data.shape[0]))
                                print("%d data removed."%(xy_yaw_data.shape[0]-desired_len))
                            elif xy_yaw_data.shape[0] < desired_len:
                                apriltag_traj = np.zeros(shape=(desired_len,xy_yaw_data.shape[1]))
                                apriltag_traj[:xy_yaw_data.shape[0],:] = xy_yaw_data
                                apriltag_traj[xy_yaw_data.shape[0]:,:] = xy_yaw_data[-1,:]
                                print("Wrong apriltag trajectory length. Desired: %d, Actual: %d"%(desired_len,xy_yaw_data.shape[0]))
                                print("%d data appended."%(desired_len-xy_yaw_data.shape[0]))
                            else:
                                print("Desired apriltag trajectory length achieved.")
                                apriltag_traj = xy_yaw_data

                            apriltag_traj = apriltag_traj - [apriltag_traj[0,0], apriltag_traj[0,1], 0]
                            apriltag_batch.append(apriltag_traj)
                            apriltag_batch_ros = rnm.to_multiarray_f32(np.array(apriltag_batch[-n_real:]))

                            if not IMU_REAL_TIME:
                                orientation_mahony = Mahony(gyr=gyro_data, acc=acc_data)
                                for i in range(len(gyro_data)):
                                    q_mahony = orientation_mahony.Q[i,:]
                                    roll_raw, pitch_raw, yaw_raw = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
                                    rpy_data = np.append(rpy_data, np.array([[roll_raw, pitch_raw, yaw_val+yaw_raw]]), axis=0)
                            
                            if rpy_data.shape[0] > desired_len:
                                imu_rpy = rpy_data[:desired_len,:]
                                print("Wrong IMU rpy length. Desired: %d, Actual: %d"%(desired_len,rpy_data.shape[0]))
                                print("%d data removed."%(rpy_data.shape[0]-desired_len))
                            elif rpy_data.shape[0] < desired_len:
                                imu_rpy = np.zeros(shape=(desired_len,rpy_data.shape[1]))
                                imu_rpy[:rpy_data.shape[0],:] = rpy_data
                                imu_rpy[rpy_data.shape[0]:,:] = rpy_data[-1,:]
                                print("Wrong IMU rpy length. Desired: %d, Actual: %d"%(desired_len,rpy_data.shape[0]))
                                print("%d data appended."%(desired_len-rpy_data.shape[0]))
                            else:
                                print("Desired IMU rpy length achieved.")
                                imu_rpy = rpy_data

                            rpy_batch.append(imu_rpy)
                            rpy_batch_ros = rnm.to_multiarray_f32(np.array(rpy_batch[-n_real:]))
                            
                            # Publish apriltag data to SPC
                            answer = str(input("Are you ready to publish data? (y/n): "))
                            while answer.lower() == 'y':
                                AprilTag.publish_once(apriltag_batch_ros)
                                IMURPY.publish_once(rpy_batch_ros)
                                answer = str(input("Continue? (y/n): "))
                                print("Publishing Data")

                        # variables to calculate and publish
                        xy_yaw_data = np.empty(shape=(0,3))
                        rpy_data = np.empty(shape=(0,3))
                        if IMU_REAL_TIME:
                            acc_data = deque()
                            gyro_data = deque()
                            yaw_val = 0.0
                        else:
                            acc_data = []
                            gyro_data = []
                            yaw_val = 0.0

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
                        np.save(os.path.join(DATA_FOLDER_CURRENT, "rpy.npy"), rpy_data)

                        # Append data batch
                        if xy_yaw_data.shape[0] > desired_len:
                            apriltag_traj = xy_yaw_data[:desired_len,:]
                            print("Wrong apriltag trajectory length. Desired: %d, Actual: %d"%(desired_len,xy_yaw_data.shape[0]))
                            print("%d data removed."%(xy_yaw_data.shape[0]-desired_len))
                        elif xy_yaw_data.shape[0] < desired_len:
                            apriltag_traj = np.zeros(shape=(desired_len,xy_yaw_data.shape[1]))
                            apriltag_traj[:xy_yaw_data.shape[0],:] = xy_yaw_data
                            apriltag_traj[xy_yaw_data.shape[0]:,:] = xy_yaw_data[-1,:]
                            print("Wrong apriltag trajectory length. Desired: %d, Actual: %d"%(desired_len,xy_yaw_data.shape[0]))
                            print("%d data appended."%(desired_len-xy_yaw_data.shape[0]))
                        else:
                            print("Desired apriltag trajectory length achieved.")
                            apriltag_traj = xy_yaw_data

                        apriltag_traj = apriltag_traj - [apriltag_traj[0,0], apriltag_traj[0,1], 0]
                        apriltag_batch.append(apriltag_traj)
                        apriltag_batch_ros = rnm.to_multiarray_f32(np.array(apriltag_batch[-n_real:]))

                        if not IMU_REAL_TIME:
                            orientation_mahony = Mahony(gyr=gyro_data, acc=acc_data)
                            for i in range(len(gyro_data)):
                                q_mahony = orientation_mahony.Q[i,:]
                                roll_raw, pitch_raw, yaw_raw = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
                                rpy_data = np.append(rpy_data, np.array([[roll_raw, pitch_raw, yaw_val+yaw_raw]]), axis=0)
                        
                        if rpy_data.shape[0] > desired_len:
                            imu_rpy = rpy_data[:desired_len,:]
                            print("Wrong IMU rpy length. Desired: %d, Actual: %d"%(desired_len,rpy_data.shape[0]))
                            print("%d data removed."%(rpy_data.shape[0]-desired_len))
                        elif rpy_data.shape[0] < desired_len:
                            imu_rpy = np.zeros(shape=(desired_len,rpy_data.shape[1]))
                            imu_rpy[:rpy_data.shape[0],:] = rpy_data
                            imu_rpy[rpy_data.shape[0]:,:] = rpy_data[-1,:]
                            print("Wrong IMU rpy length. Desired: %d, Actual: %d"%(desired_len,rpy_data.shape[0]))
                            print("%d data appended."%(desired_len-rpy_data.shape[0]))
                        else:
                            print("Desired IMU rpy length achieved.")
                            imu_rpy = rpy_data

                        rpy_batch.append(imu_rpy)
                        rpy_batch_ros = rnm.to_multiarray_f32(np.array(rpy_batch[-n_real:]))

                        # variables to calculate and publish
                        xy_yaw_data = np.empty(shape=(0,3))
                        rpy_data = np.empty(shape=(0,3))
                        if IMU_REAL_TIME:
                            acc_data = deque()
                            gyro_data = deque()
                            yaw_val = 0.0
                        else:
                            acc_data = []
                            gyro_data = []
                            yaw_val = 0.0
                    
                    else:
                        # Visualize real trajectory
                        V.append_line(x_array=apriltag_traj[:,0],y_array=apriltag_traj[:,1],z=0.0,r=0.01,
                            frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
                        V.publish_lines()

        rospy.sleep(1e-8)

    V.delete_meshes()
    V.delete_lines()

