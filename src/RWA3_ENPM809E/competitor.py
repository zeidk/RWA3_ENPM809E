#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from nist_gear.msg import Order
from nist_gear.srv import AGVToAssemblyStation


class Competitor:
    def __init__(self):
        self.received_order = False
        self.orders = []
        self.order_sub = rospy.Subscriber(
            "/ariac/orders", Order, self.order_callback)

    @property
    def received_order(self):
        return self.received_order

    @property
    def orders(self):
        return self.orders

    def start_competition(self):
        rospy.loginfo("Waiting for competition to be ready...")
        rospy.wait_for_service('/ariac/start_competition')
        rospy.loginfo("Competition is now ready.")
        rospy.loginfo("Requesting competition start...")

        try:
            start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
            response = start()
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to start the competition: %s" % exc)
        if not response.success:
            rospy.logerr("Failed to start the competition: %s" % response)
        else:
            rospy.loginfo("Competition started!")

    def stop_competition(self):
        """
        Method to stop the competition. 
        You will usually not call this method yourself as the competition
        ends when all shipments have been submitted.
        """
        try:
            end = rospy.ServiceProxy('/ariac/end_competition', Trigger)
            response = end()
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to stop the competition: %s" % exc)
        if not response.success:
            rospy.logerr("Failed to stop the competition: %s" % response)
        else:
            rospy.loginfo("Competition ended")

    def process_kitting_shipment(self, kitting_shipment):
        """
        Grab all parts needed in kitting_shipment and store
        them in a list

        Args:
            kitting_shipment (KittingShipment): A kitting shipment from /ariac/orders

        Returns:
            List[Part]: List of parts
        """
        parts = []
        for product in kitting_shipment.products:
            parts.append(product)

        return parts

    def submit_kitting_shipment(self, agv, assembly_station, shipment_type):
        """ ROS service call to submit a kitting shipment
        Returns:
        bool: status of the service call
        """
        rospy.wait_for_service('/ariac/' + agv + '/submit_shipment')
        rospy.ServiceProxy('/ariac/' + agv + '/submit_shipment',
                           AGVToAssemblyStation)(assembly_station, shipment_type)

    def order_callback(self, msg):
        """
        Callback function to handle incoming orders on /ariac/orders

        Args:
            msg (Order): Messages published on /ariac/orders
        """
        self.orders.append(msg)
        self.received_order = True
