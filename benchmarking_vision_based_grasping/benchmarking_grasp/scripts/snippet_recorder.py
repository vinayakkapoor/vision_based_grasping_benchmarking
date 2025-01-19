#!/usr/bin/env python3

# Python program to illustrate 
# saving an operated video
  
# organize imports
import cv2
import rospy
import rospkg
import datetime
import os

def list_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    non_working_ports = []
    dev_port = 0
    working_ports = []
    available_ports = []
    while len(non_working_ports) < 10: # if there are more than 10 non working ports stop the testing. 
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports.append(dev_port)
            # print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                # print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                # print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1

    if len(working_ports) == 0:
        raise Exception 
    return working_ports[0]

class VideoRecoder:
    def __init__(self, camera_name=None):
        rp = rospkg.RosPack()
        package = "benchmarking_grasp"
        self.package_path = rp.get_path(package) 

        self.camera_name = camera_name
        if camera_name is None:
            self.camera_name = rospy.get_param("camera_name")

        self.cap = None
        self.out = None
        self.started = False
        self.stopped = True

        rospy.Timer(rospy.Duration(1/30), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("[Video Recorder] Node initialized")

    def timer_cb(self, event):
        start_recording = rospy.get_param("start_recording") 
        recording_folder = rospy.get_param("recording_folder")
        current_recording = rospy.get_param("current_recording") 

        if start_recording:
            if not self.started:
                output_file_folder = os.path.join(self.package_path, "recordings", recording_folder)
                if not os.path.exists(output_file_folder):
                    os.makedirs(output_file_folder)

                output_file_path = os.path.join(output_file_folder, current_recording + ".mp4")
                self.cap, self.out = self.start_capture(self.camera_name, output_file_path)
                self.started = True
                self.stopped = False

            # reads frames from a camera 
            # ret checks return at each frame
            ret, frame = self.cap.read() 
            if not ret:
                rospy.logerr("[VideoRecorder] No frame received")
            else:
                # output the frame
                self.out.write(frame) 
        else:
            if not self.stopped:
                self.on_shutdown()
                self.stopped = True
                self.started = False

    def start_capture(self, camera_name, file_name):
        fps = 30

        # This will return video from the first webcam on your computer.
        cap = cv2.VideoCapture(camera_name)  
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 75)
        cap.set(cv2.CAP_PROP_EXPOSURE, 315)

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(file_name, fourcc, fps, (1280, 720))
        return cap, out

    def on_shutdown(self):
        # Close the window / Release webcam
        self.cap.release()
        
        # After we release our webcam, we also release the output
        self.out.release() 

if __name__ == "__main__":
    rospy.init_node("video_recorder")
    rospy.sleep(5)

    port = list_ports()
    rospy.loginfo("[Video Recorder] Using port %s for recording videos", port)
    
    video_recorder = VideoRecoder(port)
    
    rospy.spin()