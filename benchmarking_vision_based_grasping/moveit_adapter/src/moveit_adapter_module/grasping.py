import rospy
from std_msgs.msg import Float64
from franka_gripper.msg import GraspActionGoal

class Gripper:
    def __init__(self, sim_mode):
        self.sim_mode = sim_mode
        
        if self.sim_mode:
            self.finger1_pub = rospy.Publisher('/panda_finger1_controller/command', Float64, queue_size=1)
            self.finger2_pub = rospy.Publisher('/panda_finger2_controller/command', Float64, queue_size=1)
        else:
            self.grasp_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1) 
        rospy.sleep(1)
        
    def grasp(self, width):
        if self.sim_mode:
            finger1_data = Float64()
            finger2_data = Float64()
            finger1_data.data = width
            finger2_data.data = width
            
            self.finger1_pub.publish(finger1_data)
            self.finger2_pub.publish(finger2_data)        
        else:
            grasp_data = GraspActionGoal()
            grasp_data.goal.width = width
            grasp_data.goal.force = 0.7
            grasp_data.goal.speed = 0.2
            grasp_data.goal.epsilon.inner = 0.5
            grasp_data.goal.epsilon.outer = 0.5
    
            self.grasp_pub.publish(grasp_data)                
