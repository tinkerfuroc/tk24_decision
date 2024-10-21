import py_trees as pytree
import py_trees_ros as pytree_ros

from decision_msgs.srv import ScanFor

from .BaseBehaviors import ServiceHandler

# behavior mocking the actual vision scanning
class BtNode_ScanFor(ServiceHandler):

    def __init__(self, 
                 name: str,
                 bb_key: str,
                 goal_object
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_ScanFor, self).__init__(name, "scan_for", ScanFor)
        self.bb_key = bb_key
        self.object = goal_object


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_client = self.attach_blackboard_client(name=self.bb_key)
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
        self.response = self.client.call_async(request)

        self.bb_client.set(self.object, Location(), overwrite=True)
        self.feedback_message = f"Initialized ScanFor {self.object}"

    def update(self):
        self.logger.debug(f"Update ScanFor {self.object}")
        if self.response.done():
            if self.response.result().status == 0:
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
