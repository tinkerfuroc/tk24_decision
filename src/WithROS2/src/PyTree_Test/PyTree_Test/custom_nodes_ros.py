
import py_trees as pytree

import sys
import threading
from typing import Any

from mock_msgs.srv import ScanFor
import rclpy
from rclpy.node import Node


# mock data class
class Location(object):
    def __init__(self) -> None:
        self.direction = None
        self.distance = None
    
    def __str__(self) -> str:
        return str(self.__dict__)

class ServiceHandler(pytree.behaviour.Behaviour):
    def __init__(self, 
                 name: str,
                 service_name:str,
                 service_type: Any,
                 ):
        super(ServiceHandler, self).__init__(name=name)
        self.service_name = service_name
        self.service_type = service_type

        self.data_guard = threading.Lock()
        self.node = None
        self.response = None
        self.client = None
    
    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.client = self.node.create_client(self.service_type, self.service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.debug('service not available, waiting again...')
    
    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        with self.data_guard:
            if self.clearing_policy == pytree.common.ClearingPolicy.ON_INITIALISE:
                self.msg = None
        

# behavior mocking the actual vision scanning
class ScanFor(ServiceHandler):

    def __init__(self, 
                 name: str,
                 goal_object
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(ScanFor, self).__init__(name, "scan_for", ScanFor)
        self.object = goal_object


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, kwargs)

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_client = self.attach_blackboard_client(name=f"ScanFor{self.object}", namespace="Locations")
        # register a key with the name of the object, with this client having write access
        self.bb_client.register_key(self.object, access=pytree.common.Access.WRITE)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup ScanFor {self.object}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        request = ScanFor.Request()
        request.name = self.object
        # setup things that needs to be cleared
        self.response = self.node.call_async(request)

        self.bb_client.set(self.object, Location(), overwrite=True)
        self.feedback_message = f"Initialized ScanFor {self.object}"

    def update(self):
        self.logger.debug(f"Update ScanFor {self.object}")
        if self.response.done():
            if self.response.status == 0:
                location = Location()
                location.direction = "FRONT"
                location.distance = 5

                self.bb_client.set(self.object, location, overwrite=True)
                self.feedback_message = f"Found object, sent location {location}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Scanning for {self.object} failed"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return pytree.common.Status.RUNNING


class GotoObject(pytree.behaviour.Behaviour):
    def __init__(self, name, goal_object):
        super(GotoObject, self).__init__(name)
        self.object = goal_object

        self.counter = 1

    def setup(self):
        self.client = self.attach_blackboard_client(name=f"Goto{self.object}", namespace="Locations")
        self.client.register_key(self.object, access=pytree.common.Access.READ)

        self.logger.debug(f"Setup GotoObject {self.object}")
    
    def initialise(self) -> None:
        self.location = None
        self.feedback_message = f"Initialized GotoObject {self.object}"

    def update(self):
        self.logger.debug(f"Update GotoObject {self.object}")

        if self.location is None:
            if not self.client.exists(self.object):
                self.feedback_message = f"Object {self.object} does not exist in blackboard!"
                return pytree.common.Status.FAILURE
            
            self.location : Location = self.client.get(self.object)

            if (self.location.direction is None) or (self.location.distance is None):
                self.feedback_message = f"Object {self.object} found in blakcboard but is None"
                return pytree.common.Status.FAILURE
            
            self.feedback_message = f"Object {self.object} location found, going towards {self.location}"
            return pytree.common.Status.RUNNING
        else:
            if self.counter > 5:
                self.feedback_message = f"Arrived at position of {self.object} ({self.location})"
                return pytree.common.Status.SUCCESS
            else:
                self.counter += 1
                return pytree.common.Status.RUNNING