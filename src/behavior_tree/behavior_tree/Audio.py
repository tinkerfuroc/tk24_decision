import py_trees as pytree
from py_trees.common import Status
import py_trees_ros as pytree_ros

from decision_msgs.srv import Announce

from .BaseBehaviors import ServiceHandler

class BtNode_Announce(ServiceHandler):
    def __init__(self, 
                 name : str,
                 bb_source : str,
                 service_name : str = "announce"
                 ):
        super(BtNode_Announce, self).__init__(name, service_name, Announce)
        self.bb_source = bb_source
        self.announce_msg = None
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.bb_read_client = self.attach_blackboard_client(name="Announce Read")
        self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        self.logger.debug(f"Setup Announce, reading from {self.bb_source}")
    
    def initialise(self):
        try:
            self.announce_msg = self.bb_read_client.get(self.bb_source)
        except Exception as e:
            self.feedback_message = f"Announce reading message failed"
            raise e

        request = Announce.Request()
        request.announcement_msg = self.announce_msg
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Announce for message {self.announce_msg}"

    def update(self) -> Status:
        self.logger.debug(f"Update Announce {self.announce_msg}")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Finished announcing {self.announce_msg}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Announce for {self.announce_msg} failed"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still announcing..."
            return pytree.common.Status.RUNNING