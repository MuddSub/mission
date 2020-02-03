import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool
from RotateTo import *
from Move import *
from vision.msg import *

#determine if gate is in sight, get location
class Locate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['success','failure'], input_keys=['weightThreshold', 'timeout'], \
			             output_keys=['angle', 'div'])
		self.leftConf=1
		self.rightConf=1
		self.gateSub = rospy.Subscriber("gateState", gate, self.gateCB)
		self.gatePub = rospy.Publisher("gateReset", Bool, latch=True, queue_size=1)
		self.left = None
		self.right = None

	def gateCB(self, data):
		 self.leftConf = data.leftConf
		 self.rightConf = data.rightConf
		 self.divConf = data.divConf
		 self.left, self.right, self.div = data.left, data.right, data.div
	def execute(self, userdata):
		self.gatePub.publish(True)
		rospy.sleep(0.5)
		rate = rospy.Rate(20)
		startTime = time()
		count = 0
		while time() - startTime < 24 and not rospy.is_shutdown():
			rate.sleep()
			countTime = time()
			if count % 100 == 0:
				rospy.loginfo("Gate Localize: leftConf %f, rightConf %f, divConf %f",self.leftConf, self.rightConf, self.divConf)
			if self.leftConf > 13 and self.rightConf > 13:
				if self.divConf > 10:
					rospy.loginfo('Gate localize found divider')
					userdata.div = True
					leftDist = abs(self.div - self.left)
					rightDist = abs(self.div - self.right)
					if leftDist < rightDist:
						userdata.angle = (self.div + self.left) /2
					else:
						userdata.angle = (self.div + self.right)/2
				else:
					rospy.loginfo('Gate localize found two side bars but not the divider')
					userdata.div = False
					userdata.angle = (self.left + self.right) / 2
				return 'success'
		userdata.angle=None
		return 'failure'



class Latch(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes = ['dedReckon','search','continue'], input_keys = ['div','latchIn','angle'],output_keys = ['latchOut'])

	def execute(self,userdata):
		if userdata.div:
			userdata.latchOut = True
		if userdata.angle is None and userdata.latchIn:
			return 'dedReckon'
		if userdata.angle is not None:
			return 'continue'
		return 'search'

def getLocation(timeout,weightThreshold,yaw,direction):
	GetLocation = smach.StateMachine(outcomes=['success','abort'], output_keys=['angle'])
	GetLocation.userdata.timeout = timeout
	GetLocation.userdata.weightThreshold = weightThreshold
	GetLocation.userdata.incrementAngle = 20
	with GetLocation:
		smach.StateMachine.add('Locate',Locate(),
			transitions={'success':'success','failure':'RotateToFindGate'},
			remapping={'timeout':'timeout','weightThreshold':'weightThreshold', 'angle':'angle'})
		smach.StateMachine.add('RotateToFindGate',RotateTo(yaw, increment=True, direction=direction),
			transitions={'success':'Locate','abort':'abort'},
			remapping={'angle':'incrementAngle','timeout':'timeout'})

	return GetLocation



def approachGate( surge, sway, yaw, timeout):
	ApproachGate = smach.StateMachine(outcomes= ['success','abort','failure'], output_keys=['angle'])
	ApproachGate.userdata.angle=0
	ApproachGate.userdata.speed = 0.2
	ApproachGate.userdata.moveTime = 5
	ApproachGate.userdata.timeout = timeout
	ApproachGate.userdata.div = False
	ApproachGate.userdata.latch = False
	with ApproachGate:
		smach.StateMachine.add('Move',Move(surge,sway),
			transitions={'success':'Relocate',},#Kyle: changed existence to success or failure

			remapping = {'angle':'angle','speed':'speed','moveTime':'moveTime'})
		smach.StateMachine.add('Relocate', Locate(),
			transitions = {'success': 'RotateToSection','failure':'Latch'},
			remapping={'timeout':'timeout','angle':'angle', 'div':'div'})
		smach.StateMachine.add('Latch',Latch(),
			transitions={'dedReckon':'success','continue':'Move','search':'failure'},
			remapping={'latchIn':'latch','latchOut':'latch','angle':'angle', 'div':'div'})
		smach.StateMachine.add('RotateToSection', RotateTo(yaw,increment=True),
			transitions= {'success':'Latch', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'angle'})

	return ApproachGate







def StateMachine(surge, sway, yaw, timeout,weightThreshold,direction ):
	GetLocation = getLocation(timeout,weightThreshold,yaw,direction)
	ApproachGate = approachGate(surge,sway,yaw,timeout)

	Gate = smach.StateMachine(outcomes = ['success','abort'], output_keys=['angle_out'])
	Gate.userdata.angle = None
	Gate.userdata.timeout = timeout
	Gate.userdata.zero = 0
	Gate.userdata.moveTime = 5
	Gate.userdata.speed = 0.3
	with Gate:
		smach.StateMachine.add('GetLocation',GetLocation,
					transitions={'success':'RotateToGate','abort':'abort'},
					remapping={'angle':'angle'})
		smach.StateMachine.add('RotateToGate', RotateTo(yaw, increment=True),
					transitions={'success':'ApproachGate','abort':'abort'},
					remapping={'timeout':'timeout','angle':'angle'})
		smach.StateMachine.add('ApproachGate',ApproachGate,
					transitions = {'success': 'GoToGate','failure':'GetLocation', 'abort':'abort'},
					remapping = {'angle':'angle_out'})
		smach.StateMachine.add('GoToGate',Move(surge,sway),
					transitions = {'success': 'success'},
					remapping = {'angle':'zero','speed':'speed','moveTime':'moveTime'})

	return Gate
