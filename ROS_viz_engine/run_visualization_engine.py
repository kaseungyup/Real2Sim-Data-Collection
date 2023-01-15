#!/usr/bin/env python

import rospy, signal, sys, time
import numpy as np
from datetime import datetime
from functools import partial
from matplotlib import cm

from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA, Int32
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from tf.transformations import quaternion_from_euler

D2R = np.pi / 180
R2D = 180 / np.pi

def signal_handler(timer, signal, frame):

    print("Exit the attention engine.\n")
    timer.finish()
    # sys.exit(0) # this is not necessary


class Timer(object):
    """ CONSTRUCTOR """
    def __init__(self,_name='timer',_HZ=10,_MAX_SEC=np.inf,_VERBOSE=True):
        self.name           = _name
        self.HZ             = _HZ
        self.max_sec        = _MAX_SEC
        self.VERBOSE        = _VERBOSE
        self.sec_next       = 0.0
        self.sec_period     = 1.0 / self.HZ
        self.sec_elps       = 0.0
        self.sec_elps_prev  = 0.0
        self.sec_elps_diff  = 0.0
        self.sec_elps_loop  = 0.0 # exact esec 
        self.tick           = 0.
        self.force_finish   = False
        self.DELAYED_FLAG   = False
        if self.VERBOSE & 0:
            print ("[%s] initialized [%d]HZ. MAX_SEC:[%.1fsec]."
                % (self.name, self.HZ, self.max_sec))
    """ START TITMER """
    def start(self):
        self.time_start     = datetime.now()
        self.sec_next       = 0.0
        self.sec_elps       = 0.0
        self.sec_elps_prev  = 0.0
        self.sec_elps_diff  = 0.0
        self.tick           = 0.
        if self.VERBOSE:
            print ("[%s] start ([%d]HZ. MAX_SEC:[%.1fsec])."
                % (self.name, self.HZ, self.max_sec))
    """ FORCE FINISH """
    def finish(self):
        self.force_finish = True
    """ CHECK FINISHED """
    def is_finished(self):
        self.time_diff = datetime.now() - self.time_start
        self.sec_elps  = self.time_diff.total_seconds()
        if self.force_finish:
            return True
        if self.sec_elps > self.max_sec:
            return True
        else:
            return False
    def is_notfinished(self):
        self.time_diff = datetime.now() - self.time_start
        self.sec_elps  = self.time_diff.total_seconds()
        if self.force_finish:
            return False
        if self.sec_elps > self.max_sec:
            return False
        else:
            return True
    """ RUN """
    def do_run(self):
        time.sleep(1e-8) # ADD A SMALL TIME DELAY
        self.time_diff = datetime.now() - self.time_start
        self.sec_elps  = self.time_diff.total_seconds()
        if self.sec_elps > self.sec_next:
            self.sec_next = self.sec_next + self.sec_period
            self.tick     = self.tick + 1
            """ COMPUTE THE TIME DIFFERENCE & UPDATE PREVIOUS ELAPSED TIME """
            self.sec_elps_diff = self.sec_elps - self.sec_elps_prev
            self.sec_elps_prev = self.sec_elps
            """ CHECK DELAYED """
            if (self.sec_elps_diff > self.sec_period*1.5) & (self.HZ != 0):
                if self.VERBOSE:
                    # print ("sec_elps_diff:[%.1fms]" % (self.sec_elps_diff*1000.0))
                    print ("[%s][%d][%.1fs] delayed! T:[%.1fms] But it took [%.1fms]. Acutally [%.1fms]"
                        % (self.name,self.tick, self.sec_elps, self.sec_period*1000.0, 
                        self.sec_elps_diff*1000.0,self.sec_elps_loop*1000.0))
                self.DELAYED_FLAG = True

            # Save when 
            self.time_run_start = datetime.now()
            return True
        else:
            self.DELAYED_FLAG = False
            return False

    # End of loop
    def end(self):
        time_diff = datetime.now() - self.time_run_start
        self.sec_elps_loop = time_diff.total_seconds()
        if (self.sec_elps_loop > self.sec_period) & (self.HZ != 0):
            # Print all the time
            print ("[%s] is REALLY delayed! T:[%.1fms] BUT IT TOOK [%.1fms]"
            % (self.name,self.sec_period*1000.0, self.sec_elps_loop*1000.0))               








class VisualizerClass(object):
    def __init__(self,name='vis',HZ=20):
        self.name = name
        self.HZ = HZ

        self.init_pub_markers()
        self.init_pub_texts()
        self.init_pub_meshes()

    def init_pub_markers(self):
        self.marker_array = MarkerArray()
        self.n_marker = 0
        self.MARKERS_MAX = 1000
        self.pub_markers = rospy.Publisher("/visualization_engine/markers",
                        MarkerArray, queue_size=5)

    def init_pub_texts(self):
        self.text_array = MarkerArray()
        self.n_text = 0
        self.TEXTS_MAX = 1000
        self.pub_texts = rospy.Publisher("/visualization_engine/texts",
                        MarkerArray, queue_size=5)

    def init_pub_meshes(self):
        self.mesh_array = MarkerArray()
        self.n_mesh = 0
        self.MESHES_MAX = 1000
        self.pub_meshes = rospy.Publisher("/visualization_engine/meshes",
                        MarkerArray, queue_size=5)
                    

    def append_marker(self,x=0.0,y=0.0,z=0.0,r=0.1,frame_id='map',
            color=ColorRGBA(0.0,1.0,0.0,0.5),marker_type=Marker.SPHERE):
        marker = Marker(
                type=marker_type,
                pose=Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                scale=Vector3(r, r, r),
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.ADD,
                color=color,
                lifetime=rospy.Duration(secs=1/self.HZ)
                )
        self.n_marker += 1
        if(self.n_marker > self.MARKERS_MAX):
             self.marker_array.markers.pop(0)
        self.marker_array.markers.append(marker) # append

    def append_text(self,x=0.0,y=0.0,z=0.0,r=0.1,text='text',
            frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5)):
        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                text=text,
                pose=Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                scale=Vector3(r, r, r),
                header=Header(frame_id=frame_id),
                action=Marker.ADD,
                color=color,
                lifetime=rospy.Duration(secs=1/self.HZ)
                )
        self.n_text += 1
        if(self.n_text > self.TEXTS_MAX):
             self.text_array.markers.pop(0)
        self.text_array.markers.append(marker) # append

    def append_mesh(self,x=0.0,y=0.0,z=0.0,scale=1.0,dae_path='duck.dae',
            frame_id='map',color=ColorRGBA(0.0,1.0,1.0,0.5),
            roll=0.0,pitch=0.0,yaw=0.0):
        marker = Marker(
                type=Marker.MESH_RESOURCE,
                mesh_use_embedded_materials=True,
                mesh_resource=dae_path,
                # pose=Pose(Point(x, y, z), Quaternion(roll, pitch, yaw, 1)),
                pose=Pose(Point(x, y, z), Quaternion(*quaternion_from_euler(roll,pitch,yaw))),
                scale=Vector3(scale, scale, scale),
                header=Header(frame_id=frame_id),
                action=Marker.ADD,
                color=color,
                lifetime=rospy.Duration(secs=1/self.HZ)
                )
        self.n_mesh += 1
        if(self.n_mesh > self.MESHES_MAX):
             self.mesh_array.markers.pop(0)
        self.mesh_array.markers.append(marker) # append

    def reset_markers(self):
        self.marker_array.markers = []
        self.n_marker = 0

    def reset_texts(self):
        self.text_array.markers = []
        self.n_text = 0

    def reset_meshes(self):
        self.mesh_array.markers = []
        self.n_mesh = 0

    def delete_markers(self,frame_id='map'):
        self.marker_array.markers = []
        self.n_marker = 0
        marker = Marker(
                id=0,
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.DELETEALL,
        )
        self.marker_array.markers.append(marker) # append
        self.pub_markers.publish(self.marker_array)

    def delete_texts(self,frame_id='map'):
        self.text_array.markers = []
        self.n_text = 0
        marker = Marker(
                id=0,
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.DELETEALL,
        )
        self.text_array.markers.append(marker) # append
        self.pub_texts.publish(self.text_array)

    def delete_meshes(self,frame_id='map'):
        self.mesh_array.markers = []
        self.n_mesh = 0
        marker = Marker(
                id=0,
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.DELETEALL,
        )
        self.mesh_array.markers.append(marker) # append
        self.pub_meshes.publish(self.mesh_array)

    def publish_markers(self):
        # update maker index
        for m_idx,m in enumerate(self.marker_array.markers):
            m.id = m_idx
        # publish
        self.pub_markers.publish(self.marker_array)

    def publish_texts(self):
        # update maker index
        for m_idx,m in enumerate(self.text_array.markers):
            m.id = m_idx
        # publish
        self.pub_texts.publish(self.text_array)    

    def publish_meshes(self):
        # update maker index
        for m_idx,m in enumerate(self.mesh_array.markers):
            m.id = m_idx
        # publish
        self.pub_meshes.publish(self.mesh_array)    



if __name__ == '__main__':

    # Initialize node
    rospy.init_node('visualization_engine', anonymous=True)
    print ("Start visualzation_engine.")
    tmr_plot = Timer(_name='Plot',_HZ=50,_MAX_SEC=np.inf,_VERBOSE=True)
    
    # Set terminator
    signal.signal(signal.SIGINT, partial(signal_handler,tmr_plot))

    # Visualizer
    V = VisualizerClass(name='simple viz',HZ=20)
    
    # Start the loop 



    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (20HZ)

            tick = tmr_plot.tick
            print ("Plot [%.3f]sec."%(tmr_plot.sec_elps))

            # Reset 
            V.reset_markers()            
            V.reset_texts()
            V.reset_meshes()
            
            # Append marker
            x,y,z,r = np.sin(tick/5),np.cos(tick/5),0,0.5
            V.append_marker(x=x,y=y,z=z,r=r,frame_id='map',
                color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.SPHERE)
            x,y,z,r = 0,np.sin(tick/5),np.cos(tick/5),0.5
            V.append_marker(x=x,y=y,z=z,r=r,frame_id='map',
                color=ColorRGBA(0.0,0.0,1.0,0.5),marker_type=Marker.CUBE)
            x,y,z,r = np.sin(tick/5),0,np.cos(tick/5),0.5
            V.append_marker(x=x,y=y,z=z,r=r,frame_id='map',
                color=ColorRGBA(0.0,1.0,0.0,0.5),marker_type=Marker.CYLINDER)

            # Append text
            text = "%.2fsec"%(tmr_plot.sec_elps)
            V.append_text(x=0,y=0,z=1.3,r=1.0,text=text,
                frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))

            # Append mesh
            dae_path = 'file:///home/rilab/Project/real2sim-data-collection/ROS_viz_engine/snapbot_super_low_resol.stl'
            n_person = 5
            for p_idx in range(n_person):
                color = cm.hsv( (float(p_idx)/n_person) )  # color not working (bug in kinetic)
                rgb = ColorRGBA(color[0],color[1],color[2],0.5)


                T = 5
                r = 3
                phase = 2*np.pi*p_idx/n_person
                x = r*np.cos(phase+2*np.pi*tick/20/T)
                y = r*np.sin(phase+2*np.pi*tick/20/T)
                z = 1.15
                V.append_mesh(x=x,y=y,z=z,
                    scale=0.36,dae_path=dae_path,
                    frame_id='map',color=rgb,
                    roll=0,pitch=0,yaw=180*D2R + (phase+2*np.pi*tick/20/T))
                V.append_text(x=x,y=y,z=z+1.2,r=0.3,text="[%d](%.2f,%.2f)"%(p_idx,x,y),
                    frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5))


            # Publish
            V.publish_markers()
            V.publish_texts()
            V.publish_meshes()
            tmr_plot.end()

        rospy.sleep(1e-8)

    # Exit handler here
    V.delete_markers()
    V.delete_texts()
    V.delete_meshes()
    