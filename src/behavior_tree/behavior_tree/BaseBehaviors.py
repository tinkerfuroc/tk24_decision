import py_trees
import threading
from typing import Any

class ServiceHandler(py_trees.behaviour.Behaviour):
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
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug('service not available, waiting again...')
    
    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        with self.data_guard:
            if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
                self.msg = None