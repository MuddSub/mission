import rospy
import smach
import smach_ros
import rospkg
#depthPath = rospack.get_path('misssion_control') + "/src/Mission/Tasks"
#sys.path.append(depthPath)
#import GoToDepth
from vision.msg import gate
from std_msgs.msg import Bool, Float64
from time import time

"""
class Prequal(smach.State):
	
	# assume global data from allocation: angle, and error
	# p is the time before we check for reoritentation
	# r is the time we travel after we cannot see the gate bc of field of vision
	def __init__(self,surge,heave, yaw):
		smach.State.__init(self	,outcomes = ['success','abort'])
		self.surge =  surge
		self.yaw = yaw
		self.heave = heave

	def execute(self, userdata):
		
		rospy.loginfo("Start gate passing motion")
		self.surge.setZero()
		self.yaw.setZero()
		r=self.r
		p=self.p

		self.yaw.setSetpoint(angle)
		
		while( angle < error):
			self.surge.setSetpoint(p)

		self.surge.setSetpoint(r)
		if self.count<1:
			self.count+=1
			return 'pass_gate_1'
		else:
			return 'pass_gate_2'
"""

class Navigate(smach.State):

	def __init__(self,surge,yaw, initialDegree=9, timeToMarker= 60, timeBehindMarker=13, speed = 0.25):
		smach.State.__init__(self	,outcomes = ['success', 'abort'])
		self.surge =  surge
		self.yaw = yaw
		self.initialDegree = initialDegree
		self.timeToMarker = timeToMarker
		self.timeBehindMarker = timeBehindMarker
		self.speed = speed

	def execute(self, userdata):
		rospy.logwarn("1")

		rospy.logwarn("2")
		self.rotateTo(0)
		#rospy.sleep(0.5)
		rospy.logwarn("2.5")
		self.surge.setControlEffort(self.speed)
		rospy.logwarn("3")
		rospy.sleep(self.timeToMarker)
		self.surge.setControlEffort(0)
		rospy.logwarn("4")
		self.rotateTo(98.6)
		
		self.surge.setControlEffort(self.speed)
		rospy.sleep(self.timeBehindMarker)
		self.surge.setControlEffort(0)
	
		self.rotateTo(193)
		self.surge.setControlEffort(self.speed)
		rospy.sleep(300)
		'''
		self.yaw.setSetpoint(userdata.post_gate_input)
		
		while(userdata.post_gate_input < self.error):
			self.surge.setSetpoint(p)

		self.surge.setSetpoint(r)
		'''
		# now we transition to state machine pass_gate
		return "success"	

	def rotateTo(self, angle):
		count = 0
		self.yaw.setSetpoint(angle)
		done = False
		rate = rospy.Rate(50)
		while not done:
		#	rospy.logwarn("count %d, plant %d, setpoint %d", count, self.yaw.plantState%360, self.yaw.setpoint%360)
			if(abs((self.yaw.plantState%360) - (self.yaw.setpoint%360)) < 2):
				count += 1
				if(count > 100):
					rospy.logwarn("DONE")
					return True
			else:
				count = 0
			rate.sleep()			

class Localize(smach.State):
	def __init__(self, yaw):
		smach.State.__init__(self, outcomes = ['success', 'repeat','abort'])	
		self.localization = rospy.Subscriber("gateState", gate, self.gateCB)
		self.left, self.div, self.right = None, None, None
		self.leftConf, self.divConf, self.rightConf = None, None, None
		self.yaw = yaw
		
		self.pub = rospy.Publisher("gateReset", Bool, queue_size=1, latch=True)
	def gateCB(self,data):
		self.left = data.left
		self.div = data.div
		self.right = data.right
		
		self.leftConf = data.leftConf
		self.rightConf = data.rightConf
		self.divConf = data.divConf
	
	def execute(self, userdata):
		rospy.sleep(0.5)
		#look for the gate
		start = time()		
		self.pub.publish(data=True)
		prevLeftConf , prevRightConf = 0,0
		#TODO: at some point we should make this so we search
		while self.leftConf < 2.75 or self.rightConf < 2.75:
			if(prevLeftConf != self.leftConf or prevRightConf != self.rightConf):
				rospy.logwarn("left conf: {} right conf: {}".format(self.leftConf, self.rightConf))
			rospy.sleep(.01)
			prevLeftConf = self.leftConf
			prevRightConf = self.rightConf
			if time() - start > 100:
				return 'abort'
	
		goal = (self.left + self.right)/2 + self.yaw.zeroedPlantState
		rospy.logwarn("THE GOAL IS TO ROTATE TO:{}".format(goal))
		print(goal)
		self.yaw.setSetpoint( goal )
	
		
		self.rotateTo(goal)
		rospy.logwarn("ROTATED TO GATE")	
		return 'success'
        


	def rotateTo(self, angle):
                count = 0
                self.yaw.setSetpoint(angle)
                done = False
                rate = rospy.Rate(50)
                while not done:
                        #rospy.logwarn("count %d, plant %d, setpoint %d", count, self.yaw.plantState%360, self.yaw.setpoint%360)
                        if(abs((self.yaw.plantState%360) - (self.yaw.setpoint%360)) < 2):
                                count += 1
                                if(count > 100):
                                        rospy.logwarn("DONE")
                                        return True
                        else:
                                count = 0
                        rate.sleep()

		
class MoveToGate(smach.State):
	def __init__(self,surge, speed = 0.2):
		smach.State.__init__(self	,outcomes = ['success', 'abort','repeat'],input_keys=["count", "numReps"], output_keys=['count'])
		self.surge =  surge
		self.speed = speed

	def execute(self, userdata):
		userdata.count += 1
		self.surge.setControlEffort(self.speed)
		rospy.logwarn("COUNT %d", userdata.count)
		rospy.sleep(8)
		if userdata.count >= userdata.numReps:
			rospy.sleep(8)
			self.surge.setControlEffort(0)
			return 'success'
		else:	
			self.surge.setControlEffort(0)
			return 'repeat'
		
		
		
		
		
	
	
		
	
