import py_trees as pytree
from py_trees.common import Status

from tinker_decision_msgs.srv import Announce, WaitForStart

from .BaseBehaviors import ServiceHandler

class BtNode_Announce(ServiceHandler):
    """
    Node for making an audio announcement, returns SUCCESS once announcement finished
    """
    def __init__(self, 
                 name : str,
                 bb_source : str,
                 service_name : str = "announce",
                 message : str = None
                 ):
        """
        Args:
            name: name of the node (to be displayed in the tree)
            bb_source: blackboard key for retrieving a str announcement message
            service_name: name of the service for Announce
            message: optional message, if given, skips reading from blackboard
        """

        # call parent initializer
        super(BtNode_Announce, self).__init__(name, service_name, Announce)
        
        # store parameters
        self.bb_source = bb_source
        self.bb_read_client = None
        self.announce_msg = message
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        # if no announcement message is given, set up a blackboard client to read from given key
        if not self.announce_msg:
            self.bb_read_client = self.attach_blackboard_client(name="Announce Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)
            self.logger.debug(f"Setup Announce, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Announce for message {self.announce_msg}")
    
    def initialise(self):
        super().initialise()

        # if no announcement message is given, read from the blackboard and verify the information
        if not self.announce_msg:
            try:
                self.announce_msg = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.announce_msg, str)
            except Exception as e:
                self.feedback_message = f"Announce reading message failed"
                raise e

        # initialize a request and set the annnouncement message
        request = Announce.Request()
        request.announcement_msg = self.announce_msg

        # send request to service and store the returned Future object
        self.response = self.client.call_async(request)

        # update feedback message
        self.feedback_message = f"Initialized Announce for message {self.announce_msg}"

    def update(self) -> Status:
        self.logger.debug(f"Update Announce {self.announce_msg}")
        # if the service is done, check its status
        if self.response.done():
            # if status is 0, all is well, return success
            if self.response.result().status == 0:
                self.feedback_message = f"Finished announcing {self.announce_msg}"
                return pytree.common.Status.SUCCESS
            # else, update feedback message to reflect the error message and return failure
            else:
                self.feedback_message = f"Announce for {self.announce_msg} failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        # if service is not done, simply return running
        else:
            self.feedback_message = "Still announcing..."
            return pytree.common.Status.RUNNING


class BtNode_WaitForStart(ServiceHandler):
    """
    Node to wait for an audio signal to start task, returns success once signal is received
    """
    def __init__(self, 
                 name : str,
                 service_name : str = "wait_for_start"
                 ):
        super(BtNode_WaitForStart, self).__init__(name, service_name, WaitForStart)
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.logger.debug(f"Setup waiting for start")
    
    def initialise(self):
        request = WaitForStart.Request()
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized wait for start"

    def update(self) -> Status:
        self.logger.debug(f"Update wait for start")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = "Started"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Wait for start failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still waiting for start..."
            return pytree.common.Status.RUNNING