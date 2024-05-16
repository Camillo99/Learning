import subprocess, shlex, psutil
from time import sleep
import rospy
from ur_msgs.msg import IOStates
from std_msgs.msg import Int16, Float64MultiArray
from geometry_msgs.msg import PoseArray
import pickle

class ROSBag_record:
    def __init__(self):
        self.hole_color = None
        self.hole_pose = None
        self.EE_pose = None
        #create a command to be called in the console later on
        #start a rosbag record of the:
        #                             /joint_states topic
        #                             /Current_EE_position
        self.command = "rosbag record --output-name my_bag /joint_states /Current_EE_position "
        self.command = shlex.split(self.command)
        self.rosbag_proc = None
        #pubblisher to trigger the camera capture
        self.publisher = rospy.Publisher('Demo_status', Int16, queue_size=10)

        #subscribe to the color of the marker
        self.subscriber_1 = rospy.Subscriber("/hole_color", Float64MultiArray, self.callback1)
        #subscribe to the pose of the holes 
        self.subscriber_2 = rospy.Subscriber("/hole_pose", PoseArray, self.callback2)
        #subscribe to the current EE position
        self.subscriber_3 = rospy.Subscriber("/Current_EE_position", Float64MultiArray, self.callback3)


        val_1 = input("Press enter to start demonstration: ") 
        if val_1 == '':
            print('Acquiring vision data. Please Wait...')
            msg = Int16()
            msg.data = 1
            self.publisher.publish(msg)
            while 1:
                if self.hole_color != None and self.hole_pose != None and self.EE_pose != None:
                    #create a dictionary to store the holes information from the differents topics
                    memory = {'hole_pose':self.hole_pose, 'hole_color': self.hole_color, 'EE_pose': self.EE_pose}
                    with open("/home/camillo/workspace/Learning/scripts/Memory_1.pkl", "wb") as fp:
                        pickle.dump(memory, fp)
                    input("Vision Data acquired. Press enter to continue...")
                    self.rosbag_proc = subprocess.Popen(self.command)
                    print("Recording... (press any key to stop)")
                    input()
                    for proc in psutil.process_iter():
                        if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
                            proc.send_signal(subprocess.signal.SIGINT)
                    self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
                    msg = Int16()
                    msg.data = 0
                    self.publisher.publish(msg)
                    break
        else:
            pass
    
    # callbacks to store data from each topics
    def callback1(self, msg):
        self.hole_color  = msg.data

    def callback2(self, msg):
        self.hole_pose  = msg

    def callback3(self, msg):
        self.EE_pose = msg
        
    # def callback(self, data):
    #     self.digital_pin_2 = data.digital_out_states[2].state

    #     if self.digital_pin_2 != self.digital_old: 
    #         if self.digital_pin_2:
    #             self.rosbag_proc = subprocess.Popen(self.command)
    #             self.digital_old = self.digital_pin_2
    #             print("Recording...", "|", self.digital_pin_2)
    #             pass 

    #         else:
    #             for proc in psutil.process_iter():
    #                 if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
    #                     proc.send_signal(subprocess.signal.SIGINT)
    #             self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
    #             self.digital_old = self.digital_pin_2
    #             print("Bag_Ready", "|", self.digital_pin_2)
    #             pass
    #     else:
    #         pass
            
        
    # def listner(self):
    #     rospy.Subscriber("/io_and_status_controller/io_states", IOStates, self.callback)
    #     rospy.spin()



if __name__ == '__main__':

    rospy.init_node('rosbag_recorder')
    print("recorder activated")
    c = ROSBag_record()
    # c.listner()