#!/usr/bin/env python

import rospy
import smach
import smach_ros
import rospkg
import imp
import sys
rospack = rospkg.RosPack()
rospack.list()
taskPath = rospack.get_path('mission_control') + "/src/Mission/Tasks"
sys.path.append(taskPath)
from Axis import Axis
from std_msgs.msg import Bool, Float64
from BetterConcurrence import *
from UtilStates import *
from GoToDepth import *
from RotateTo import *
import Gate
import Buoy
from Move import *

#services
from mission_control.srv import *

import roslib; roslib.load_manifest('mission_control')



# main
def main():
	rospy.init_node('state_machine')
	rospack = rospkg.RosPack()
	rospy.sleep(2)	
	
	sway = Axis("sway")
	surge = Axis("surge")
	heave = Axis("heave")
	yaw = Axis ("yaw")

	heave.setEnabled(True)
	heave.setInput("DEPTH")
	
	yaw.setEnabled(False)
	yaw.setInput("IMU_POS")
	
	sway.setInput("IMU_POS")
	sway.setSetpoint(0)
	sway.setEnabled(False)	
	
	surge.setInput("IMU_POS")
	surge.setSetpoint(0)
	surge.setEnabled(0)
        #yaw.setSetpoint(0)	
	done = False
	direction = 1
	timeout = 31415926

	rospy.loginfo("Loaded PID Controllers")

         ######################################## Mission code ############################################	

	# Create a SMACH state machine
	Mission = smach.StateMachine(outcomes=['success', 'abort'])
	Mission.userdata.done = False
	Mission.userdata.count = 0
	Mission.userdata.numReps = 4
	Mission.userdata.depth = 0.4
	Mission.userdata.timeout = 234523413
	Mission.userdata.zero = 0
	Mission.userdata.false = False
	Mission.userdata.gateAngle = 0	
	Mission.userdata.speed = 0.31415
	# direction:=1 for clockwise
	gate = Gate.StateMachine(surge, sway, yaw, timeout, 2.75,direction=1)
	buoy = Buoy.StateMachine(surge,sway,heave,yaw,timeout)

	# Open the container
	with Mission:

		# Add states to the containe
		#smach.StateMachine.add("WaitForStart", smach_ros.MonitorState("/start", Bool, startCB), transitions={'valid':"GoToDepth", "invalid":"WaitForStart", "preempted":"WaitForStart"})
		smach.StateMachine.add('Zero', Zero(heave, yaw), transitions={'success':'GoToDepth'})
		smach.StateMachine.add('GoToDepth', GoToDepth(heave),
							   transitions={'success':'CorrectToZero',
											'abort':'abort'},
							   remapping={'depth':'depth', 'increment':'false'})
		smach.StateMachine.add('CorrectToZero',RotateTo(yaw),
							transitions={'success':'Buoy','abort':'abort'},
							remapping={'timeout':'timeout','angle':'zero'})									 
		smach.StateMachine.add('Buoy', buoy, transitions={'success':'success','abort':'abort'},
							remapping={'angle_in':'gateAngle'})											

		smach.StateMachine.add('Gate', gate, transitions={'success':'Buoy','abort':'abort'},
							remapping={'angle_out':'gateAngle'})

		smach.StateMachine.add("DedReckon", Move(surge, sway), transitions={'success':'success'}, remapping={'angle':'zero','speed':'speed','moveTime':'timeout'})

	###########################################Concurrence Container########################################

	def childTermCB(outcome_map):
		if outcome_map['MonitorKill'] == 'False':
			return True
		return False
			 

	def outcomeCB(outcome_map):
		if outcome_map['MonitorKill'] == 'False':
			rospy.logwarn("Kill switch triggered!")
			return 'kill'
		return 'success'

	SafeExecute = BetterConcurrence(outcomes = ['kill','abort', 'success'], default_outcome='kill', 
			 outcome_map={'success':{'Mission':'success'}, 'abort':{'Mission': 'abort'}, 'kill':{'MonitorKill':'False'}})
					
	with SafeExecute:
		BetterConcurrence.add("MonitorKill", MonitorStart(target=False))
		BetterConcurrence.add("Mission", Mission)

	



	######################################    TOP    #########################
	Top = smach.StateMachine(outcomes=['abort', 'success'])
	with Top:
		smach.StateMachine.add('WaitForStart', MonitorStart(), 
					transitions={'True':'Run', 'False':'WaitForStart'})
		smach.StateMachine.add("Run", SafeExecute, transitions={'success':'ResetSuccess', 'kill': 'ResetAbort', 'abort':'ResetAbort'})
		smach.StateMachine.add("ResetAbort", Reset(), transitions={'success':'WaitForStart', 'abort':'Dead'})
		smach.StateMachine.add("ResetSuccess", Reset(), transitions={'success':'WaitForReset', 'abort':'Dead'})
		smach.StateMachine.add("WaitForReset", MonitorStart(target=False), transitions={'True':'WaitForReset', 'False':'WaitForStart'})
		smach.StateMachine.add("Dead", Dead(), transitions={'success':'Dead', 'kill':'Dead'})

	ready = False
        while not ready:
                ready = rospy.wait_for_message("/start", Bool)
        rospy.loginfo("Ready to execute task")
	sis = smach_ros.IntrospectionServer('server_name', Top, '/SM_ROOT')
	sis.start()
	# Execute SMACH plan
        startPub = rospy.Publisher("thrustEnable", Bool, latch=True)	
	yaw.setEnabled(False)
	yaw.setControlEffort(0)
 	startPub.publish(data=False)	
	outcome = Top.execute()
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()
