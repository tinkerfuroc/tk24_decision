import py_trees as pytree

from tinker_decision_msgs.srv import Grasp, Drop
from geometry_msgs.msg import PointStamped

from .BaseBehaviors import ServiceHandler


class BtNode_Grasp(ServiceHandler):
    """
    Node for grasping an object with a specific prompt
    """
    def __init__(self, 
                 name: str,
                 bb_source: str,
                 service_name : str = "grasp",
                 prompt: str = None
                 ):
        """
        executed when creating tree diagram, therefor very minimal

        Args:
            name: name of the node (to be displayed in the tree)
            bb_source: blackboard key to a str prompt
            service_name: name of the service running Grasp
            prompt: optional, if given, skips reading from blackboard
        """
        super(BtNode_Grasp, self).__init__(name, service_name, Grasp)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.prompt = prompt


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.prompt is None:
            self.bb_read_client = self.attach_blackboard_client(name="Grasp Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup Grasp, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Grasp, with givne prompt {self.prompt}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        try:
            self.prompt = self.bb_read_client.get(self.bb_source)
            assert isinstance(self.prompt, str)
        except Exception as e:
            self.feedback_message = f"Grasp reading object name failed"
            raise e

        request = Grasp.Request()
        request.prompt = self.prompt
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Grasp for prompt {self.prompt}"

    def update(self):
        self.logger.debug(f"Update Grasp {self.prompt}")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Grasp Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Grasp failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still grasping..."
            return pytree.common.Status.RUNNING


class BtNode_Drop(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_source: str,
                 service_name : str = "drop",
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PointStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_Drop, self).__init__(name, service_name, Drop)
        self.bb_source = bb_source
        self.bb_read_client = None


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        self.bb_read_client = self.attach_blackboard_client(name="Drop Read")
        self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup Drop, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        try:
            bin_point = self.bb_read_client.get(self.bb_source)
            assert isinstance(bin_point, PointStamped)
        except Exception as e:
            self.feedback_message = f"Drop reading object name failed"
            raise e

        self.logger.debug(f"Initialized Drop for bin point {bin_point}")

        request = Drop.Request()
        request.bin_point = bin_point
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Drop"

    def update(self):
        self.logger.debug(f"Update Drop")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Drop Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Drop failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still dropping object..."
            return pytree.common.Status.RUNNING
