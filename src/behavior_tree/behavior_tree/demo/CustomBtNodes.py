from behavior_tree.messages import PoseStamped
import py_trees
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
# from tinker_decision_msgs.srv import Goto, GotoGrasp, RelToAbs
# from geometry_msgs.msg import PointStamped, PoseStamped

from behavior_tree.messages import *
from nav_msgs.msg import Odometry

class BtNode_Goto_Iterate(BtNode_Goto):
    def __init__(self, name: str, targets: list[PoseStamped], service_name: str = "goto"):
        super().__init__(name, "", service_name, None)
        self.targets = targets
        self.idx = 0
    
    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.logger.debug(f"Set up goto iterating of length {len(self.targets)}")

    def initialise(self) -> None:
        request = Goto.Request()
        request.target = self.targets[self.idx]
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Goto iterating, currently at index {self.idx}"
        self.idx = (self.idx + 1) % len(self.targets)


class BtNode_MoveArmInitial(ServiceHandler):
    def __init__(self, name: str, 
                 service_name: str, 
                 ):
        super().__init__(name, service_name, ArmJointService)
        self.arm_joint_pose = [0., 0.6, 0., -0.3, 0., -44.4, 0.]
    
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
