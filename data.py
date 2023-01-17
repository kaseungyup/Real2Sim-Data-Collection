import cv2, os, serial, rospy
import numpy as np
import pyrealsense2 as rs

from time import time, localtime
from collections import deque
from imutils.video import WebcamVideoStream
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from classes.timer import Timer
from classes.visualizerclass import VisualizerClass
from filter.mahony import Mahony
from utils.utils_tps import get_tps_mat, get_real_xy_yaw
from utils.utils_real import quaternion_to_vector
from publisher import IMURPYPublisher, AprilTagPublisher
from subscriber import FlagDataSubscriber, SimTrajSubscriber
from ros_np_multiarray import ros_np_multiarray as rnm


if __name__ == '__main__':

    Hz = 25
    desired_len = int(Hz*300/50)
    n_mahony = 10 # number of previous data for mahony filter
    D2R = np.pi/180
    R2D = 180/np.pi

    CURR_FOLDER = os.getcwd()
    DATA_FOLDER = os.path.join(CURR_FOLDER , "data")
    if not os.path.isdir(DATA_FOLDER): os.mkdir(DATA_FOLDER)
    stl_path = 'file://' + CURR_FOLDER + '/ROS_viz_engine/snapbot_super_low_resol.stl'

    REALSENSE_CAMERA_NUMBER = 12
    # EGOCENTRIC_CAMERA_NUMBER = 4
    # IMU_USB_NUMBER = 0
    # ser = serial.Serial('/dev/ttyUSB{}'.format(IMU_USB_NUMBER), 115200, timeout=1)
    """
    pipeline = rs.pipeline()
    config = rs.config()
    align_to = rs.stream.color
    align = rs.align(align_to)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    tps_coef = get_tps_mat(pipeline, align)
    """
    rospy.init_node('subscriber', anonymous=True)
    print("1. Start visualization_engine.")
    V = VisualizerClass(name='simple viz',HZ=Hz)

    FlagData = FlagDataSubscriber()
    SimTraj  = SimTrajSubscriber()
    AprilTag = AprilTagPublisher()
    IMURPY   = IMURPYPublisher()

    zero_tick = 0
    one_tick = 0
    epoch = 0
    max_epoch = 2

    apriltag_batch = []
    prev_real_x, prev_real_y, prev_real_yaw = 0,0,0
    rpy_batch = []

    acc_array = [0,0,0]
    gyro_array = [0,0,0]

    tmr = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    tmr.start()
    print("2. Start Timer")

    while tmr.is_notfinished():
        if tmr.do_run():
            if FlagData.flag: # new trajectory starts
                if one_tick == 0: # reset variables and initialize
                    zero_tick = 0
                    one_tick += 1

                    # Visualizer
                    V.reset_lines()
                    V.reset_meshes()

                    # variables to calculate and publish
                    xy_yaw_data = np.empty(shape=(0,3))
                    # rpy_data = np.empty(shape=(0,3))
                    # acc_data = deque()
                    # gyro_data = deque()
                    # yaw_val = 0.0

                    # simulation trajectory
                    sim_data = np.array(SimTraj.traj).reshape((SimTraj.length, SimTraj.height, SimTraj.num))
                    curr_traj = sim_data[epoch%max_epoch,:,:]
                    print(curr_traj.shape)
                    """
                    V.append_line(x_array=curr_traj[:,0],y_array=curr_traj[:,1],z=0.0,r=0.01,
                        frame_id='map',color=ColorRGBA(1.0,0.0,0.0,1.0),marker_type=Marker.LINE_STRIP)
                    """
                    print("4. Simulation data published")

                    DATA_FOLDER_EPOCH = os.path.join(DATA_FOLDER_TIME, "Epoch %d"%(epoch%max_epoch))
                    os.mkdir(DATA_FOLDER_EPOCH)

                    # RS_CAMERA_FOLDER = os.path.join(DATA_FOLDER_EPOCH, "realsense_camera")
                    # EGO_CAMERA_FOLDER = os.path.join(DATA_FOLDER_EPOCH, "egocentric_camera")
                    # os.mkdir(RS_CAMERA_FOLDER)
                    # os.mkdir(EGO_CAMERA_FOLDER)

                    # manage video data
                    # rs_video = WebcamVideoStream(src=REALSENSE_CAMERA_NUMBER).start()
                    # rs_frame_width = 640
                    # rs_frame_height = 480
                    # rs_size = (rs_frame_width, rs_frame_height)
                    # rs_result = cv2.VideoWriter(os.path.join(RS_CAMERA_FOLDER, "rs.mp4"),
                    #             cv2.VideoWriter_fourcc('m','p','4','v'), Hz, rs_size)

                    # ego_video = WebcamVideoStream(src=EGOCENTRIC_CAMERA_NUMBER).start()
                    # ego_frame_width = 640
                    # ego_frame_height = 480
                    # ego_size = (ego_frame_width, ego_frame_height)
                    # ego_result = cv2.VideoWriter(os.path.join(EGO_CAMERA_FOLDER, "ego.mp4"),
                    #             cv2.VideoWriter_fourcc('m','p','4','v'), Hz, ego_size)

                    print("5. Camera Initialized")

                    # Check epoch
                    epoch += 1

                else: # while trajectory is running
                    one_tick += 1

                    # Realsense camera video
                    # rs_frame = rs_video.frame
                    # rs_result.write(rs_frame)

                    # Egocentric camera video
                    # ego_frame = ego_video.frame
                    # ego_frame_flip = cv2.flip(ego_frame, -1)
                    # ego_result.write(ego_frame_flip)

                    # Calculate real x, y
                    try:
                        """real_x, real_y, real_yaw = get_real_xy_yaw(tps_coef, pipeline)"""
                        prev_real_x, prev_real_y, prev_real_yaw = real_x, real_y, real_yaw
                    except:
                        real_x, real_y, real_yaw = prev_real_x, prev_real_y, prev_real_yaw


                    # Read IMU data
                    # line = ser.readline()
                    # imu_raw_data = line.decode('unicode_escape')
                    # imu_raw_data = imu_raw_data.split()
                    # if len(imu_raw_data) == 6:
                    #     acc_array = [-float(imu_raw_data[1].replace('\x00','')), float(imu_raw_data[0].replace('\x00','')), float(imu_raw_data[2].replace('\x00',''))]
                    #     gyro_array = [float(imu_raw_data[3].replace('\x00','')), float(imu_raw_data[4].replace('\x00','')), float(imu_raw_data[5].replace('\x00',''))]
                    # acc_data.append(acc_array)
                    # gyro_data.append(gyro_array)

                    # if len(acc_data) > n_mahony:
                    #     acc_data.popleft()
                    # if len(gyro_data) > n_mahony:
                    #     gyro_data.popleft()

                    # Calculate RPY data
                    # orientation_mahony = Mahony(gyr=list(gyro_data), acc=list(acc_data))
                    # q_mahony = orientation_mahony.Q[-1,:]
                    # roll_raw, pitch_raw, yaw_raw = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
                    # yaw_val += yaw_raw

                    # roll_data = (np.pi-roll_raw)*R2D # degrees
                    # pitch_data = -pitch_raw*R2D # degrees
                    # yaw_data = yaw_val*100/Hz/n_mahony*R2D # degrees
                   
                    # Append data
                    xy_yaw_data = np.append(xy_yaw_data, np.array([[real_x, real_y, real_yaw]]), axis=0)
                    # rpy_data = np.append(rpy_data, np.array([[roll_data, pitch_data, yaw_data]]), axis=0)

                    # Visualizer
                    """V.append_mesh(x=real_x-xy_yaw_data[0,0],y=real_y-xy_yaw_data[0,1],z=0,scale=1.0,dae_path=stl_path,
                        frame_id='map', color=ColorRGBA(1.0,1.0,1.0,0.5),
                        roll=0,pitch=0*D2R,yaw=0)"""
                        # roll=roll_data*D2R,pitch=pitch_data*D2R,yaw=yaw_data*D2R)

                    """V.publish_meshes()
                    V.publish_lines()"""


            else: # trajectory has ended 
                if epoch%max_epoch == 0: # start/end of each cycle
                    if zero_tick == 0:
                        one_tick = 0
                        zero_tick += 1

                        tm = localtime(time())
                        DATA_FOLDER_TIME = os.path.join(DATA_FOLDER, "%d%02d%02d-%02d:%02d:%02d" % (tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec))
                        if not os.path.isdir(DATA_FOLDER_TIME): os.mkdir(DATA_FOLDER_TIME)

                        if epoch != 0: # end of current cycle
                            # save videos and variables
                            # rs_video.stop()
                            # rs_result.release()
                            # ego_video.stop()
                            # ego_result.release()

                            np.save(os.path.join(DATA_FOLDER_EPOCH, "xy_yaw.npy"), xy_yaw_data)
                            # np.save(os.path.join(DATA_FOLDER_EPOCH, "rpy.npy"), rpy_data)

                            # Append batch
                            if xy_yaw_data.shape[0] > desired_len:
                                apriltag_traj = xy_yaw_data[:desired_len,:]
                            else:
                                apriltag_traj = np.zeros(shape=(desired_len,xy_yaw_data.shape[1]))
                                apriltag_traj[:xy_yaw_data.shape[0],:] = xy_yaw_data
                                apriltag_traj[xy_yaw_data.shape[0]:,:] = xy_yaw_data[-1,:]

                            apriltag_traj = apriltag_traj - [apriltag_traj[0,0], apriltag_traj[0,1], 0]
                            apriltag_batch.append(apriltag_traj)
                            # rpy_batch.append(rpy_data[:sim_len,:])

                            apriltag_batch_ros = rnm.to_multiarray_f32(np.array(apriltag_batch[-max_epoch:]))
                            # rpy_batch_ros = rnm.to_multiarray_f32(np.array(rpy_batch[-max_epoch:]))
                            
                            answer = str(input("Are you ready to publish data? (y/n): "))
                            while answer.lower() == 'y':
                                AprilTag.publish_once(apriltag_batch_ros)
                                # rpy_publisher(rpy_batch_ros)
                                answer = str(input("Continue? (y/n): "))
                                print("Publishing Data")

                        print("3. Waiting")
                        print("Start new cycle")

                    else:
                        flag = FlagData.flag
                        sim_len = SimTraj.length
                        # real trajectory
                        """V.append_line(x_array=apriltag_traj[:,0],y_array=apriltag_traj[:,1],z=0.0,r=0.01,
                            frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
                        V.publish_lines()"""
                                   
                else:
                    if zero_tick == 0:
                        one_tick = 0
                        zero_tick += 1
                        print("3. Waiting")
                        print("epoch: ", epoch%max_epoch)

                        # save videos and variables
                        # rs_video.stop()
                        # rs_result.release()
                        # ego_video.stop()
                        # ego_result.release()

                        np.save(os.path.join(DATA_FOLDER_EPOCH, "xy_yaw.npy"), xy_yaw_data)
                        # np.save(os.path.join(DATA_FOLDER_EPOCH, "rpy.npy"), rpy_data)

                        # Append batch
                        if xy_yaw_data.shape[0] > desired_len:
                            apriltag_traj = xy_yaw_data[:desired_len,:]
                        else:
                            apriltag_traj = np.zeros(shape=(desired_len,xy_yaw_data.shape[1]))
                            apriltag_traj[:xy_yaw_data.shape[0],:] = xy_yaw_data
                            apriltag_traj[xy_yaw_data.shape[0]:,:] = xy_yaw_data[-1,:]

                        apriltag_traj = apriltag_traj - [apriltag_traj[0,0], apriltag_traj[0,1], 0]
                        apriltag_batch.append(apriltag_traj)
                        # rpy_batch.append(rpy_data[:sim_len,:])
                    
                    """else:
                        # real trajectory
                        V.append_line(x_array=apriltag_traj[:,0],y_array=apriltag_traj[:,1],z=0.0,r=0.01,
                            frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
                        V.publish_lines()"""

        rospy.sleep(1e-8)

    V.delete_meshes()
    V.delete_lines()