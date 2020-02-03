# define state Foo
import rospy
import smach
import smach_ros

class Foo(smach.State):
    def __init__(self, heave):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2'])
        self.heave = heave

    def execute(self, userdata):
        rospy.logwarn("HERE")
        self.heave.setZero()
        return 'outcome1'
