# define state Foo
import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool


class RotateTo(smach.State):
    def __init__(self, yaw, increment=False, direction=1):
        smach.State.__init__(self,
                             outcomes=['success', 'abort'],
				input_keys=['angle', 'timeout'])
	self.yaw = yaw
	self.increment = increment
	self.direction = direction

    def execute(self, userdata):
		angle = userdata.angle
		if self.increment:
			angle += self.yaw.zeroedPlantState
		rospy.logwarn("RotateTo: angle: %f, direction: %d", angle, self.direction)
		angle *= self.direction
		self.yaw.setSetpoint(angle)
		loopRate = rospy.Rate(50)
		tStart = time()
		done = False
                successCount = 0
		while not done:
			
			if(abs((self.yaw.plantState%360) - (self.yaw.setpoint%360)) < 2):
				successCount += 1	
				if(successCount > 200):
					done = True
			elif time() -tStart > userdata.timeout:
				return 'abort'
			else:
				 successCount = 0
			loopRate.sleep()
		return 'success'

