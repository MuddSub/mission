import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool


class GoToDepth(smach.State):
    def __init__(self, heave):
        smach.State.__init__(self,
                             outcomes=['success', 'abort'],
				input_keys=['depth', 'timeout', 'increment', 'direction'])
        self.heave = heave

    def execute(self, userdata):
		setpoint = userdata.depth
		if userdata.increment:
			setpoint = self.heave.zeroedPlantState + userdata.depth * userdata.direction

		self.heave.setSetpoint(setpoint)
		loopRate = rospy.Rate(50)
		tStart = time()
		done = False
                successCount = 0
		while not done:
			
			while not rospy.is_shutdown() and time() - tStart < 1:
				loopRate.sleep()
			if(abs(self.heave.plantState - self.heave.setpoint) < .025):
				rospy.logwarn("GoToDepth: at depth waiting for accuracy")
				successCount += 1	
				if(successCount > 100):
					done = True
			elif time() -tStart > userdata.timeout:
				return 'abort'
			else:
				 successCount = 0
			loopRate.sleep()
		return 'success'

