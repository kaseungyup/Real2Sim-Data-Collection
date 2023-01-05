import cv2, os, serial, rospy
import numpy as np

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
from publisher import apriltag_publisher, rpy_publisher
from subscriber import FlagData, SimulationData

Hz = 50
n_mahony = 10 # number of previous data for mahony filter
D2R = np.pi/180
R2D = 180/np.pi

CURR_FOLDER = os.getcwd()
DATA_FOLDER = os.path.join(CURR_FOLDER , "data")
if not os.path.isdir(DATA_FOLDER): os.mkdir(DATA_FOLDER)
stl_path = 'file://' + CURR_FOLDER + '/ROS_viz_engine/snapbot.stl'

REALSENSE_CAMERA_NUMBER = 12
EGOCENTRIC_CAMERA_NUMBER = 4
IMU_USB_NUMBER = 1
ser = serial.Serial('/dev/ttyUSB{}'.format(IMU_USB_NUMBER), 115200, timeout=1)

if __name__ == '__main__':

    tps_coef = get_tps_mat()

    rospy.init_node('lpc_subscriber', anonymous=True)
    print("Start visualization_engine.")
    V = VisualizerClass(name='simple viz',HZ=Hz)

    flag = FlagData()
    sim_traj = SimulationData()
    zero_tick = 0
    one_tick = 0

    tmr_plot = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    tmr_plot.start()

    while tmr_plot.is_notfinished():
        if tmr_plot.do_run():
            if flag.flag: # new trajectory starts
                if one_tick == 0: # reset variables and initialize
                    zero_tick = 0
                    one_tick += 1

                    # Visualizer
                    V.reset_lines()
                    V.reset_meshes()

                    # variables to calculate and publish
                    xy_yaw_data = np.empty(shape=(0,3))
                    rpy_data = np.empty(shape=(0,3))
                    acc_data = deque()
                    gyro_data = deque()
                    yaw_val = 0.0

                    # simulation trajectory
                    sim_data = sim_traj.traj
                    len = len(sim_data)
                    sim_data = np.array(sim_data).reshape((int(len/3),3))
                    V.append_line(x_array=sim_data[:,0],y_array=sim_data[:,1],z=0.0,r=0.01,
                        frame_id='map',color=ColorRGBA(1.0,0.0,0.0,1.0),marker_type=Marker.LINE_STRIP)
                    V.publish_lines()

                    # manage folders
                    tm = localtime(time())
                    DATA_FOLDER_TIME = os.path.join(DATA_FOLDER, "%d%02d%02d-%02d:%02d:%02d" % (tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec))
                    RS_CAMERA_FOLDER = os.path.join(DATA_FOLDER_TIME, "realsense_camera")
                    EGO_CAMERA_FOLDER = os.path.join(DATA_FOLDER_TIME, "egocentric_camera")
                    os.mkdir(DATA_FOLDER_TIME)
                    os.mkdir(RS_CAMERA_FOLDER)
                    os.mkdir(EGO_CAMERA_FOLDER)

                    # manage video data
                    rs_video = WebcamVideoStream(src=REALSENSE_CAMERA_NUMBER).start()
                    rs_frame_width = rs_video.frame.shape[1]
                    rs_frame_height = rs_video.frame.shape[0]
                    rs_size = (rs_frame_width, rs_frame_height)
                    rs_result = cv2.VideoWriter(os.path.join(RS_CAMERA_FOLDER, "rs.mp4"),
                                cv2.VideoWriter_fourcc('m','p','4','v'), Hz, rs_size)

                    ego_video = WebcamVideoStream(src=EGOCENTRIC_CAMERA_NUMBER).start()
                    ego_frame_width = ego_video.frame.shape[1]
                    ego_frame_height = ego_video.frame.shape[0]
                    ego_size = (ego_frame_width, ego_frame_height)
                    ego_result = cv2.VideoWriter(os.path.join(EGO_CAMERA_FOLDER, "ego.mp4"),
                                cv2.VideoWriter_fourcc('m','p','4','v'), Hz, ego_size)

                else: # while trajectory is running
                    one_tick += 1

                    # Realsense camera video
                    rs_frame = rs_video.frame
                    rs_result.write(rs_frame)

                    # Egocentric camera video
                    ego_frame = ego_video.frame
                    ego_frame_flip = cv2.flip(ego_frame, -1)
                    ego_result.write(ego_frame_flip)

                    # Calculate real x, y
                    real_x, real_y, real_yaw = get_real_xy_yaw(tps_coef)

                    # Read IMU data
                    line = ser.readline()
                    imu_raw_data = line.decode('unicode_escape')
                    imu_raw_data = imu_raw_data.split()
                    acc_array = [-float(imu_raw_data[1]), float(imu_raw_data[0]), float(imu_raw_data[2])]
                    gyro_array = [float(imu_raw_data[3]), float(imu_raw_data[4]), float(imu_raw_data[5])]
                    acc_data.append(acc_array)
                    gyro_data.append(gyro_array)

                    if len(acc_data) > n_mahony:
                        acc_data.popleft()
                    if len(gyro_data) > n_mahony:
                        gyro_data.popleft()

                    # Calculate RPY data
                    orientation_mahony = Mahony(gyr=list(gyro_data), acc=list(acc_data))
                    q_mahony = orientation_mahony.Q[-1,:]
                    roll_raw, pitch_raw, yaw_raw = quaternion_to_vector(q_mahony[0],q_mahony[1],q_mahony[2],q_mahony[3])
                    yaw_val += yaw_raw

                    roll_data = (np.pi-roll_raw)*R2D # degrees
                    pitch_data = -pitch_raw*R2D # degrees
                    yaw_data = yaw_val*100/Hz/n_mahony*R2D # degrees
                    
                    # Append data
                    xy_yaw_data = np.append(xy_yaw_data, np.array([[real_x, real_y, real_yaw]]), axis=0)
                    rpy_data = np.append(rpy_data, np.array([[roll_data, pitch_data, yaw_data]]), axis=0)

                    # Publish data
                    apriltag_publisher(real_x, real_y)
                    rpy_publisher(roll_data, pitch_data, yaw_data)

                    # Visualizer
                    V.append_mesh(x=real_x,y=real_y,z=0,scale=1.0,dae_path=stl_path,
                        frame_id='map', color=ColorRGBA(1.0,1.0,1.0,0.5),
                        roll=roll_data*D2R,pitch=pitch_data*D2R,yaw=yaw_data*D2R)
                    V.publish_meshes()


            else: # trajectory has ended     
                if zero_tick == 0: # trajectory just ended
                    one_tick = 0
                    zero_tick += 1

                    # save videos and variables
                    rs_video.stop()
                    rs_result.release()
                    ego_video.stop()
                    ego_result.release()

                    np.save(os.path.join(DATA_FOLDER_TIME, "xy_yaw.npy"), xy_yaw_data)
                    np.save(os.path.join(DATA_FOLDER_TIME, "rpy.npy"), rpy_data)

                    # real trajectory
                    V.append_line(x_array=xy_yaw_data[:,0],y_array=xy_yaw_data[:,1],z=0.0,r=0.01,
                        frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
                    V.publish_lines()

                else: # waiting for next trajectory
                    zero_tick += 1

        rospy.sleep(1e-8)

    V.delete_meshes()
    V.delete_lines()