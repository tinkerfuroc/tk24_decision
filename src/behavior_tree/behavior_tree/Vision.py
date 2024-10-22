import py_trees as pytree
import py_trees_ros as pytree_ros

from decision_msgs.srv import ScanFor, FindObj

from .BaseBehaviors import ServiceHandler


class BtNode_ScanFor(ServiceHandler):

    def __init__(self, 
                 name: str,
                 bb_source,
                 bb_namespace: str,
                 bb_key:str,
                 service_name : str = "scan_for"
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_ScanFor, self).__init__(name, service_name, ScanFor)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = None


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        self.bb_read_client = self.attach_blackboard_client(name="ScanFor Read")
        self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"ScanFor", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup ScanFor, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        try:
            self.object = self.bb_read_client.get(self.bb_source)
        except Exception as e:
            self.feedback_message = f"ScanFor reading object name failed"
            raise e

        request = ScanFor.Request()
        request.name = self.object
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
                self.feedback_message = f"Scanning for {self.object} failed"
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
                 service_name:str = "find_obj"
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_ScanFor, self).__init__(name, service_name, ScanFor)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = None


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

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
        try:
            self.object = self.bb_read_client.get(self.bb_source)
        except Exception as e:
            self.feedback_message = f"FindObj reading object name failed"
            raise e

        request = FindObj.Request()
        request.name = self.object
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
                self.feedback_message = f"Find Obj for {self.object} failed"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still finding obj..."
            return pytree.common.Status.RUNNING