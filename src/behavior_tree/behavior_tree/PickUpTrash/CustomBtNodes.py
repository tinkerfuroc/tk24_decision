# from geometry_msgs.msg import PointStamped, PoseStamped
from behavior_tree.messages import *
import py_trees

from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor

class BtNode_ScanAndSave(BtNode_ScanFor):
    def __init__(self, name: str, 
                 bb_source: str, 
                 bb_key_point : str,
                 bb_key_type : str,
                 service_name: str = "object_detection", 
                 object: str = None
                 ):
        super().__init__(name, bb_source, "ScanFor", "Response", service_name, object)

        self.bb_key_point = bb_key_point
        self.bb_key_type = bb_key_type
        self.bb_point_client = None
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.bb_point_client = self.attach_blackboard_client(name=f"ScanForPoint Write")
        self.bb_point_client.register_key(self.bb_key_point, access=py_trees.common.Access.WRITE)
        self.bb_point_client.register_key(self.bb_key_type, access=py_trees.common.Access.WRITE)
    
    def update(self):
        self.logger.debug(f"Update ScanFor {self.object}")
        if self.response.done():
            if self.response.result().status == 0:
                self.logger.debug(f"Response: {self.response.result()}")
                self.bb_write_client.set(self.bb_key, self.response.result(), overwrite=True)
                if self.response.result().objects:
                    # create a PointStamped object based on the returned header and point
                    point_stamped = PointStamped()
                    point_stamped.point = self.response.result().objects[0].centroid
                    point_stamped.header = self.response.result().header


                    classfication = self.response.result().objects[0].cls

                    # store the PointStamped and the type to the given blackboard keys
                    self.bb_point_client.set(self.bb_key_point, point_stamped, overwrite=True)
                    self.bb_point_client.set(self.bb_key_type, classfication, overwrite=True)
                    
                    # update feedback message
                    self.feedback_message = f"Found objects, first object's centroid s  tored to {self.bb_key_point}, type {classfication} stored to {self.bb_key_type}"
                    return py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = f"Response does not contain an object"
                    return py_trees.common.Status.FAILURE
            else:
                self.feedback_message = f"Scanning for {self.object} failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return py_trees.common.Status.RUNNING



