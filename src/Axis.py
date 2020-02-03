import numpy as np
import rospy
import rospkg
from mission_control.srv import *
from std_msgs.msg import String, Float64

class Axis:
    _axis = ""
    _enabled = False
    _inputs = {"IMU_POS" : 0, "IMU_ACCEL" : 1, "DEPTH" : 2, "CAM_FRONT": 3, "CAM_BOTTOM" : 4, "LOCALIZE": 5}
    _zeros = {"IMU_POS" : 0, "IMU_ACCEL" : 0, "DEPTH" : 0, "CAM_FRONT": 0, "CAM_BOTTOM" : 0, "LOCALIZE": 0}
    plantState = 0
    _input = ""
    setpoint = 0
    zeroedPlantState = 0

    def __init__(self, name):
        self._axis = name
        paramName = "/TOPICS/"
        paramName +=  name.upper()
        self._sub = rospy.Subscriber("/none", Float64, self.plantStateCallback)

	zeroPub = rospy.Publisher("yawSetpointNorm", Float64, latch=True)
	zeroPub.publish(data=0)	

    def plantStateCallback(self, data):
        self.plantState =  data.data 
	self.zeroedPlantState = data.data - self._zeros[self._input]
    def updatePlantTopic(self):
        self._sub.unregister()
        paramName = "/TOPICS/"
        paramName += self._axis.upper()
        self.plantTopic = rospy.get_param(paramName)
        rospy.logwarn("Changing topic for axis {} to {}".format(self._axis, self.plantTopic))
        self._sub = rospy.Subscriber(self.plantTopic, Float64, self.plantStateCallback)

    def setEnabled(self, val=True):
        self._enabled = val
        rospy.wait_for_service('EnabledService')
        try:
            enabledServiceProxy = rospy.ServiceProxy('EnabledService', EnabledService)
            res = enabledServiceProxy(self._axis, val)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def setControlEffort(self, val=0):
        rospy.loginfo("Disabling %s control loop", self._axis)
        self.setEnabled(False);
        self._enabled = False
        rospy.logwarn("2.375")
        rospy.wait_for_service('ThrustOverrideService')
        rospy.logwarn("2.625")
        try:
            enabledServiceProxy = rospy.ServiceProxy('ThrustOverrideService', ThrustOverrideService)
            res = enabledServiceProxy(self._axis, val)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


    def setSetpoint(self, val=0):
        if(not self._enabled):
            rospy.logwarn("Make sure the loop is enabled")
        rospy.wait_for_service('SetpointService')
	self.setpoint = val + self._zeros[self._input] 
        rospy.loginfo("setpoint %f, plantstate %f, val %f, zero %f", float(self.setpoint), float(self.plantState), float(val), float(self._zeros[self._input]))
	try:
            enabledServiceProxy = rospy.ServiceProxy('SetpointService', SetpointService)
            res = enabledServiceProxy(self._axis, self.setpoint )
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


    def setZero(self, zero=None):
		sum = 0.0
		for i in range(10):
			sum += rospy.wait_for_message(self.plantTopic, Float64).data
		avg = sum / 10
		rospy.loginfo("Setting heave zero on {} to {}".format(self._input, avg))
		self._zeros[self._input] = avg

    def goTo(self, target, delay = 1):
        self.setSetpoint(target + self._zeros[self._input])
        tStart = rospy.get_time()
        while(rospy.get_time() - tStart < 1000*delay):
            rospy.sleep(0.1)

    def increment(self, target, delay = 1):
        self.setSetpoint(target + self.plantState + self._zeros[self._input])
        tStart = rospy.get_time()
        while(rospy.get_time() - tStart < 1000*delay):
            rospy.sleep(0.1)


    def setInput(self, val):
        rospy.loginfo(self._inputs[val])

        rospy.wait_for_service('InputTypeService')
        try:
            enabledServiceProxy = rospy.ServiceProxy('InputTypeService', InputService)
            res = enabledServiceProxy(self._axis, self._inputs[val])
            self._input = val
        except rospy.ServiceException, e:
            rospy.logwarn("Set input service call failed: %s", e)
	    return False
        self.updatePlantTopic()
        return res.success
