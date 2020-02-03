import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool
from RotateTo import *
from GoToDepth import *
from Move import *
from vision.msg import *
from math import ceil
import copy
class Localize(smach.State):
	# if twoBuoy is true, then it needs two confidences. else just the confidence specificed
	# left is the left gate
	def __init__(self, twoBuoy,left,center=False):
		smach.State.__init__(self,outcomes = ['success','failure'],
					output_keys = ['angle'])
		self.buoySub = rospy.Subscriber('buoyState', buoy, self.buoyCB)
		self.twoBuoy=twoBuoy
		self.left=left
		self.center = center
		self.firstYaw, self.firstYawConf = None, 1
		self.secondYaw, self.secondYawConf = None, 1
		self.resetPub = rospy.Publisher("buoyReset", Bool, queue_size=1, latch=True)
	def buoyCB(self, data):
                self.firstYaw, self.firstYawConf = data.firstYaw, data.firstYawConf
                self.secondYaw, self.secondYawConf = data.secondYaw, data.secondYawConf

	def execute(self, userdata):
		self.resetPub.publish(data=True)
		rospy.sleep(0.5)
		startTime = time()
		rate = rospy.Rate(20)
		while time() - startTime < 30 and not rospy.is_shutdown():
			rospy.loginfo("Buoy Localization: First Conf: %f, Second Conf: %f", self.firstYawConf, self.secondYawConf)
			if self.twoBuoy:
				if self.firstYawConf > 3.35 and self.secondYawConf > 3.35:
					rospy.loginfo("Buoy Localization: Found two targets")
					if self.center:
						angle = sum(self.firstYaw, self.secondYaw)/2
					if self.left:
						angle = min(self.firstYaw,self.secondYaw)
						userdata.angle = angle
					else:
						angle = max(self.firstYaw,self.secondYaw)
						userdata.angle = angle
					return 'success'

			elif self.firstYawConf > 3.35:
					rospy.loginfo("Buoy Localization: Found single target")
					angle = self.firstYaw
					userdata.angle = angle
					return 'success'
			rate.sleep()
		return 'failure'

class FindHeave(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failure', 'abort'],
				     input_keys = ['timeout', 'inDirection'], output_keys=['directionOut'])

		self.buoySub = rospy.Subscriber("buoyState", buoy, self.buoyCB)
		self.resetPub = rospy.Publisher("buoyReset", Bool, latch=True, queue_size=1)
                self.heave, self.heaveConf = None, 1
	def buoyCB(self, data):

                self.heave, self.heaveConf = data.firstHeave, data.firstYawConf

	def execute(self, userdata):

		inDirection = copy.deepcopy(userdata.inDirection)
                if inDirection is None:
			inDirection = 0
		self.resetPub.publish(data=True)
                while self.heaveConf > 1.5 and not rospy.is_shutdown():
                        rospy.sleep(0.01)

		startTime = time()
		success = False

		while time() - startTime < 7 and not rospy.is_shutdown():
			rospy.sleep(0.1)

		directionOut = -1 if self.heave > 0 else 1
		userdata.directionOut = directionOut
		success = True
		if not success:
			return 'abort'
		rospy.loginfo("Buoy FindHeave: INDIRECTION %d, DIROUT %d", inDirection, directionOut)
		if directionOut == inDirection or inDirection == 0:
			rospy.loginfo("Incrementing depth and re-trying")
			return 'failure'
		else:
			rospy.loginfo("Sign flipped, done finding depth")
			return 'success'


#class (smach.State):
#	def __init__(self):
#		smach.State.__init__(self,outcomes=
#left is whether it is left gate. Passes to localize
def searchRoutine(yaw,initialAngle,finalAngle,left=True, center=False):
	searchSpan = finalAngle - initialAngle
	direction = searchSpan > 0
	searchSpan = abs(searchSpan)
	numIt = ceil(searchSpan / 20)

	search = smach.Iterator(outcomes=['success', 'abort'],input_keys=[], output_keys=['outAngle'],
				it= lambda: range(0, int(numIt)),
				it_label='index',
				exhausted_outcome='abort')

	search.userdata.initialAngle = initialAngle

	with search:
		container_sm = smach.StateMachine(outcomes = ['success', 'abort','continue'],
						output_keys=['outAngle'])
		container_sm.userdata.timeout = 500
		container_sm.userdata.angle = 20
		container_sm.userdata.initialAngle = initialAngle
		container_sm.userdata.outAngle = 0
		with container_sm:
			smach.StateMachine.add('InitialRotate', RotateTo(yaw, increment=True, direction=1),
				transitions={'success':'Localize','abort':'abort'},
				remapping={'timeout':'timeout','angle':'initialAngle'})
			smach.StateMachine.add('Rotate', RotateTo(yaw,increment=True, direction=direction),
				transitions={'success':'continue', 'abort':'abort'},
				remapping = {'timeout':'timeout', 'angle':'angle'})
			smach.StateMachine.add('Localize', Localize(True, left, center=center),
				transitions={'success':'success', 'failure':'Rotate'},
				remapping={'angle':'outAngle'})
		smach.Iterator.set_contained_state('SEARCH', container_sm, loop_outcomes=['continue'])

	return search

def findFirstBuoy(yaw, heave, timeout, center=False):
	firstBuoy = smach.StateMachine(outcomes = ['success','abort'], input_keys=['angle_in'])
	firstBuoy.userdata.timeout = timeout
	firstBuoy.userdata.initialDepth = .7
	firstBuoy.userdata.neg90 = -90
	firstBuoy.userdata.chosenBuoy = None
	firstBuoy.userdata.true = True
	firstBuoy.userdata.false = False
	firstBuoy.userdata.leftAngle = 0

	with firstBuoy:
		smach.StateMachine.add('GoToDepth', GoToDepth(heave),
			transitions = {'success': 'FirstRotate', 'abort':'abort'},
			remapping = {'timeout':'timeout','depth':'initialDepth', 'increment':'false','direction':'true'})

		smach.StateMachine.add('FirstRotate',RotateTo(yaw,increment=True),
			transitions = {'success': 'Localize', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'angle_in'})

		smach.StateMachine.add('Localize', Localize(True,True, center=center),
			transitions = {'success':'RotateToBuoy', 'failure':'SearchRotate'},
			remapping = {'angle':'leftAngle'})

		smach.StateMachine.add('SearchRotate',RotateTo(yaw, increment=True),
			transitions = {'success':'Search', 'abort':'abort'},
			remapping={'timeout':'timeout','angle':'neg90'})

		smach.StateMachine.add('Search', searchRoutine(yaw, 0, 180, center=center),
			transitions = {'success':'RotateToBuoy', 'abort':'abort'},
			remapping = {'outAngle':'leftAngle'})

		smach.StateMachine.add('RotateToBuoy', RotateTo(yaw, increment=True),
			transitions = {'success':'success', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'leftAngle'})

	return firstBuoy



def heaveSearch(heave):

	HeaveSearch = smach.StateMachine(outcomes=['success','abort'])

	HeaveSearch.userdata.depthInc = 0.2
	HeaveSearch.userdata.direction = None
	HeaveSearch.userdata.timeout = 314159265
	HeaveSearch.userdata.true = True
	with HeaveSearch:
		smach.StateMachine.add('FindHeave', FindHeave(),
			transitions = {'success':'success', 'failure':'IncrementDepth', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'inDirection':'direction',\
				     'directionOut':'direction'})

		smach.StateMachine.add('IncrementDepth', GoToDepth(heave),
			transitions= {'success':'FindHeave', 'abort':'abort'},
			remapping = {'depth':'depthInc', 'timeout':'timeout', 'increment':'true', 'direction':'direction'})

	return HeaveSearch

def bumpBuoy(surge, sway, yaw, timeout):

        bump = smach.StateMachine(outcomes=['success','abort'])
	numIt = 3
	with bump:
		bump.userdata.back = 180
		bump.userdata.speed = 0.25
		bump.userdata.backTime = 20
		bump.userdata.zero = 0
		bump.userdata.bumpTime = 10

		it = smach.Iterator(outcomes=['success', 'abort'], input_keys=[], output_keys=[],
                                it= lambda: range(0, int(numIt)),
                                it_label='index',
                                exhausted_outcome='success')
		with it:
			container_sm = smach.StateMachine(outcomes=['success','abort','continue'])

        		container_sm.userdata.moveTime = 6
        		container_sm.userdata.zero = 0
	        	container_sm.userdata.speed = 0.3
			container_sm.userdata.angle = None
			container_sm.userdata.timeout = timeout

			with container_sm:
				smach.StateMachine.add('Move', Move(surge,sway),
					transitions={'success':'ReLocalize'},
					remapping = {'angle':'zero','speed':'speed','moveTime':'moveTime'})
				smach.StateMachine.add('ReLocalize', Localize(False,True),
					transitions={'success':'RotateTo', 'failure':'abort'},
					remapping={'outAngle':'angle'})
				smach.StateMachine.add('RotateTo', RotateTo(yaw),
					transitions={'success':'continue', 'abort':'abort'},
					remapping={'timeout':'timeout','angle':'angle'})
			smach.Iterator.set_contained_state("BumpIt", container_sm, loop_outcomes=['continue'])

		smach.StateMachine.add("BUMP_IT", it,
			transitions={'success':'FinalBump', 'abort':'abort'})

		smach.StateMachine.add("FinalBump", Move(surge,sway),
			transitions={'success':'MoveBack'},
			remapping={'angle':'zero','speed':'speed','moveTime':'bumpTime'})
		smach.StateMachine.add("MoveBack", Move(surge,sway),
			transitions={'success':'success'},
			remapping={'angle':'back','speed':'speed','moveTime':'backTime'})
	return bump

def findSecondBuoy(yaw, heave, timeout):
	findBuoySmach = searchRoutine(yaw, 45, 315, False)

	findSecond = smach.StateMachine(outcomes=['success','abort'])
	findSecond.userdata.timeout = timeout
	findSecond.userdata.target = None
	findSecond.userdata.angle = None
	findSecond.userdata.direction = 0
	findSecond.userdata.depthInc = 0.5
	findSecond.userdata.true = True
	with findSecond:
		smach.StateMachine.add("Search",findBuoySmach,
			transitions={'success':'RotateTo','abort':'abort'},
			remapping={'outAngle':'angle'})
		smach.StateMachine.add("RotateTo", RotateTo(yaw),
			transitions={'success':'FindHeave','abort':'abort'},
			remapping={'timeout':'timeout','angle':'angle'})

                smach.StateMachine.add('FindHeave', FindHeave(),
                        transitions = {'success':'success', 'failure':'IncrementDepth', 'abort':'abort'},
                        remapping = {'timeout':'timeout', 'inDirection':'direction',\
                                     'directionOut':'direction'})

                smach.StateMachine.add('IncrementDepth', GoToDepth(heave),
                        transitions= {'success':'FindHeave', 'abort':'abort'},
                        remapping = {'depth':'depthInc', 'timeout':'timeout', 'increment':'true', 'direction':'direction'})


	return findSecond
def StateMachine(surge, sway, heave, yaw, timeout):
	findCenter = findFirstBuoy(yaw, heave, timeout, center=True)
	findFirst = findFirstBuoy(yaw, heave, timeout)
	bumpIt = bumpBuoy(surge, sway, yaw, timeout)
	findSecond = findSecondBuoy(yaw,heave,timeout)
	findFirstHeave = heaveSearch(heave)
	Buoy = smach.StateMachine(outcomes=['success','abort'], input_keys=['angle_in'])
	Buoy.userdata.zero = 0
	Buoy.userdata.speed = 0.3
	Buoy.userdata.moveTime = 20
	Buoy.userdata.count = 0
	Buoy.userdata.halfSpin = 180

	with Buoy:
		smach.StateMachine.add('FindCenter', findCenter,
			transitions={'success':'MoveCenter','abort':'abort'},
			remapping={'angle_in':'angle_in'})

		smach.StateMachine.add('MoveCenter', Move(surge,sway),
                        transitions={'success':'FindFirst'},
                        remapping = {'angle':'zero','speed':'speed','moveTime':'moveTime'})

		smach.StateMachine.add('FindFirst', findFirst,
			transitions={'success':'FindHeave','abort':'abort'},
			remapping={'angle_in':'halfSpin'})
		smach.StateMachine.add('FindHeave', findFirstHeave,
			transitions={'success':'BumpIt','abort':'abort'})
		smach.StateMachine.add('BumpIt', bumpIt,
			transitions={'success':'FindSecond','abort':'abort'})

		smach.StateMachine.add('FindSecond',findSecond,
			transitions={'success':'BumpSecond','abort':'abort'})
		smach.StateMachine.add('BumpSecond', bumpIt,
			transitions={'success':'success','abort':'abort'})


	return Buoy
