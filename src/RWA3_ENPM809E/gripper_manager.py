#!/usr/bin/env python

import rospy
from nist_gear.srv import VacuumGripperControl
from nist_gear.msg import VacuumGripperState


class GripperManager():
    def __init__(self, ns):
        self.ns = ns

    def activate_gripper(self):
        """
        Activate a robot's gripper to grasp objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service(self.ns + 'control')
        try:
            control = rospy.ServiceProxy(
                self.ns + 'control', VacuumGripperControl)
            result = control(True)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def deactivate_gripper(self):
        """
        Deactivate a robot's gripper to release objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service(self.ns + 'control')
        try:
            control = rospy.ServiceProxy(
                self.ns + 'control', VacuumGripperControl)
            result = control(False)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def is_object_attached(self):
        status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
        return status.attached