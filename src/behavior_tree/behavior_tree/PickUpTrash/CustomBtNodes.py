# from geometry_msgs.msg import PointStamped, PoseStamped
import py_trees
from behavior_tree.messages import *

from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor
from behavior_tree.messages import ArmJointService
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler

class BtNode_ScanAndSave(BtNode_ScanFor):
    def __init__(self, name: str, 
                 bb_source: str, 
                 bb_key_point : str,
                 bb_key_type : str,
                 service_name: str = "object_detection", 
                 object: str = None,
                 filter_far = False,
                 use_orbbec = False,
                 ):
        super().__init__(name, bb_source, "ScanFor", "Response", service_name, object, use_orbbec)

        self.bb_key_point = bb_key_point
        self.bb_key_type = bb_key_type
        self.bb_point_client = None
        self.filter_far = filter_far
    
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

                    if self.filter_far:
                        if point_stamped.point.z > 2.5:
                            self.feedback_message = f"Detected Object is too far away"
                            return py_trees.common.Status.FAILURE

                    # store the PointStamped and the type to the given blackboard keys
                    self.bb_point_client.set(self.bb_key_point, point_stamped, overwrite=True)
                    self.bb_point_client.set(self.bb_key_type, classfication, overwrite=True)
                    
                    # update feedback message
                    self.feedback_message = f"Found objects, first object's centroid ({point_stamped}) stored to {self.bb_key_point}, type {classfication} stored to {self.bb_key_type}"
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


class BtNode_MoveArmSet(ServiceHandler):
    def __init__(self, name: str, 
                 service_name: str, 
                 arm_joint_pose: list[float] = [0., 0.6, 0., -0.3, 0., -44.4, 0.]
                 ):
        super().__init__(name, service_name, ArmJointService)
        self.arm_joint_pose = arm_joint_pose

        assert isinstance(self.arm_joint_pose, list) and len(self.arm_joint_pose) == 7
    
    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup MoveArm to initial position")

    def initialise(self):
        request = ArmJointService.Request()
        
        request.joint0 = self.arm_joint_pose[0]
        request.joint1 = self.arm_joint_pose[1]
        request.joint2 = self.arm_joint_pose[2]
        request.joint3 = self.arm_joint_pose[3]
        request.joint4 = self.arm_joint_pose[4]
        request.joint5 = self.arm_joint_pose[5]
        request.joint6 = self.arm_joint_pose[6]

        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized move arm joint for joints {self.arm_joint_pose}"
    
    def update(self):
        self.logger.debug(f"Update move arm joint")
        if self.response.done():
            if self.response.result().success:
                self.feedback_message = f"Move arm Successful"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Move arm failed"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"Still moving arm to {self.arm_joint_pose}..."
            return py_trees.common.Status.RUNNING
