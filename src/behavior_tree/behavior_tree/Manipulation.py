import py_trees as pytree
import py_trees_ros as pytree_ros

from decision_msgs.srv import Grasp

from .BaseBehaviors import ServiceHandler


class BtNode_Grasp(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_source,
                 service_name : str = "grasp"
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_Grasp, self).__init__(name, service_name, Grasp)
        self.bb_source = bb_source
        self.rgb_image = None
        self.depth_image = None
        self.segment = None


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        self.bb_read_client = self.attach_blackboard_client(name="Grasp Read")
        self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup Grasp, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        try:
            object = self.bb_read_client.get(self.bb_source)
            self.rgb_image = object.rgb_image
            self.depth_image = object.depth_image
            self.segment = object.segments[0]
        except Exception as e:
            self.feedback_message = f"Grasp reading object name failed"
            raise e

        request = Grasp.Request()
        request.rgb_image = self.rgb_image
        request.depth_image = self.depth_image
        request.segments = [self.segment]
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Grasp"

    def update(self):
        self.logger.debug(f"Update Grasp {self.object}")
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
