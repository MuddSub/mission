# define state Foo
import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool
import math

class Move(smach.State):
    def __init__(self, surge,sway):
        smach.State.__init__(self,
                             outcomes=['success'],
				input_keys=['angle','speed','moveTime'])

	self.surge = surge
	
	self.sway = sway
	

    def execute(self, userdata):
	
		surgeSpeed = userdata.speed * math.cos( math.radians(userdata.angle ))

		swaySpeed = userdata.speed * math.sin( math.radians(userdata.angle))

		self.surge.setControlEffort(surgeSpeed)
		self.sway.setControlEffort(swaySpeed)
		
		rospy.sleep(userdata.moveTime)	
	
		self.surge.setControlEffort(0)
		self.sway.setControlEffort(0)

		return 'success'

