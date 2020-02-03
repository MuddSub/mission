import rospy
import smach
import smach_ros
import rospkg
import imp
import sys
from Axis import Axis
from std_msgs.msg import Bool


class MonitorStart(smach.State):
        
        def __init__(self, target=True):
                smach.State.__init__(self, outcomes=['True', 'False'])
                self.startSub = rospy.Subscriber("start", Bool, self.startCB)   
                self.start = None
                self.target = target

        def startCB(self, data):
                self.start = data.data

        def execute(self, userdata):
                rate = rospy.Rate(20)
                
                while not rospy.is_shutdown():
                        if self.start is not None and self.start == self.target:
                                return 'True' if self.start else 'False'
                        rate.sleep() 

class Reset(smach.State):

        def __init__(self):
                smach.State.__init__(self, outcomes=['success', 'abort'])
                self.thrusterPub = rospy.Publisher("thrustEnable", Bool, latch=True, queue_size=1)      
        

        def execute(self, userdata):
                self.thrusterPub.publish(data=False)
                return 'success'

class Zero(smach.State):
	def __init__(self, heave, yaw):
		smach.State.__init__(self, outcomes=['success'])
		self.heave = heave
		self.yaw = yaw
		self.thrusterPub = rospy.Publisher("thrustEnable", Bool, latch=True, queue_size=1)      
	def execute(self, userdata):
		self.heave.setZero()
		self.yaw.setZero()
		self.thrusterPub.publish(data=True)
		return 'success'

class Dead(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['success', 'kill'])
        def execute(self, userdata):
                self.__init__()
                try:
                        while not rospy.is_shutdown():
                                rospy.sleep(2)
                                return 'success'
                except PreemptException:
			pass
