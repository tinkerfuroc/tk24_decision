import py_trees as pytree

# from tinker_decision_msgs.srv import ObjectDetection
# from tinker_vision_msgs.srv import ObjectDetection

from behavior_tree.messages import *

from .BaseBehaviors import ServiceHandler


class BtNode_ScanFor(ServiceHandler):

    def __init__(self, 
                 name: str,
                 bb_source: str,
                 bb_namespace: str,
                 bb_key:str,
                 service_name : str = "object_detection",
                 object: str = None,
                 use_orbbec = True
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_ScanFor, self).__init__(name, service_name, ObjectDetection)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = object
        self.use_orbbec = use_orbbec


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)
        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"ScanFor", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        if not self.object:
            self.bb_read_client = self.attach_blackboard_client(name="ScanFor Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup ScanFor, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup ScanFor, for {self.object}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if not self.object:
            try:
                self.object = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.object, str)
            except Exception as e:
                self.feedback_message = f"ScanFor reading object name failed"
                raise e

        request = ObjectDetection.Request()
        request.prompt = self.object
        request.flags = "scan"
        if self.use_orbbec:
            request.camera = "orbbec"
        else:
            request.camera = "realsense"
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized ScanFor for {self.object}"

    def update(self):
        self.logger.debug(f"Update ScanFor {self.object}")
        if self.response.done():
            if self.response.result().status == 0:
                self.bb_write_client.set(self.bb_key, self.response.result(), overwrite=True)
                self.feedback_message = f"Found object, stored to blackboard {self.bb_namespace} / {self.bb_key}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Scanning for {self.object} failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return pytree.common.Status.RUNNING


class BtNode_FindObj(ServiceHandler):

    def __init__(self, 
                 name: str,
                 bb_source,
                 bb_namespace: str,
                 bb_key:str,
                 service_name:str = "object_detection",
                 object:str = None
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_FindObj, self).__init__(name, service_name, ObjectDetection)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = object


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.object is None:
            self.bb_read_client = self.attach_blackboard_client(name="FindObj Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"FindObj", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup FindObj, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.object is None:
            try:
                self.object = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.object, str)
            except Exception as e:
                self.feedback_message = f"FindObj reading object name failed"
                raise e

        request = ObjectDetection.Request()
        request.prompt = self.object
        request.flags = "find_for_grasp"
        request.camera = "realsense"
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized FindObj for {self.object}"

    def update(self):
        self.logger.debug(f"Update FindObj {self.object}")
        if self.response.done():
            if self.response.result().status == 0:
                self.bb_write_client.set(self.bb_key, self.response.result(), overwrite=True)
                self.feedback_message = f"Found object, stored to blackboard {self.bb_namespace} / {self.bb_key}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Find Obj for {self.object} failed with error code {self.response.result().status}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still finding obj..."
            return pytree.common.Status.RUNNING