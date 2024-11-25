from typing import Any
import py_trees as pytree

# mock data class
class Location(object):
    def __init__(self) -> None:
        self.direction = None
        self.distance = None
    
    def __str__(self) -> str:
        return str(self.__dict__)

# behavior mocking the actual vision scanning
class ScanFor(pytree.behaviour.Behaviour):

    def __init__(self, name, goal_object):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(ScanFor, self).__init__(name)
        self.object = goal_object


    def setup(self):
        """
        setup for the node, recursively called with tree.setup()
        """
        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.client = self.attach_blackboard_client(name=f"ScanFor{self.object}", namespace="Locations")
        # register a key with the name of the object, with this client having write access
        self.client.register_key(self.object, access=pytree.common.Access.WRITE)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup ScanFor {self.object}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        # setup things that needs to be cleared
        self.counter = 1
        self.client.set(self.object, Location(), overwrite=True)
        self.feedback_message = f"Initialized ScanFor {self.object}"

    def update(self):
        self.logger.debug(f"Update ScanFor {self.object}")

        if self.counter > 5:
            location = Location()
            location.direction = "FRONT"
            location.distance = 5

            self.client.set(self.object, location, overwrite=True)
            self.feedback_message = f"Found object, sent location {location}"

            return pytree.common.Status.SUCCESS
        else:
            self.counter += 1
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
            if not self.client.c(self.object):
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