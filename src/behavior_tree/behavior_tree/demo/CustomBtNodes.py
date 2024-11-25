from behavior_tree.messages import PoseStamped
import py_trees
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
# from tinker_decision_msgs.srv import Goto, GotoGrasp, RelToAbs
# from geometry_msgs.msg import PointStamped, PoseStamped

from behavior_tree.messages import *
from nav_msgs.msg import Odometry
import numpy as np

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

class AAA:
    def __init__(self, x, y, z):
        self.x = [1, 2, x]
        self.y = y * 2
        self.z = z


class BtNode_TestBB(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, bb_key: str):
        super().__init__(name)
        self.bb_key = bb_key
    
    def setup(self, **kwargs):
        self.logger.debug(f"Setup test node")
        self.bb_client = self.attach_blackboard_client(name="Test BB")
        self.bb_read_client = self.attach_blackboard_client(name="Test BB")
        self.bb_client.register_key(self.bb_key, access=py_trees.common.Access.WRITE)
        self.bb_read_client.register_key(self.bb_key, access=py_trees.common.Access.READ)

    def initialise(self):
        self.bb_client.set(self.bb_key, None, overwrite=True)
    
    def update(self):
        self.logger.debug(f"Update test node")

        if self.bb_client.exists(self.bb_key):
            print("key exists")
            prev = self.bb_read_client.get(self.bb_key)
        if isinstance(prev, AAA):
            self.logger.debug(f'prev: x-{prev.x}, y-{prev.y}, z-{prev.z}')
        else:
            self.logger.debug(f'prev: {prev}')

        x = np.random.randint(0, 4)
        if x < 2:
            nxt = AAA(np.random.randint(0, 10), np.random.randint(0, 10), np.random.randint(0, 10))
        elif x < 3:
            nxt = f"hello {np.random.randint(0, 10)}"
        else:
            nxt = None
        if isinstance(nxt, AAA):
            self.logger.debug(f'nxt: x-{nxt.x}, y-{nxt.y}, z-{nxt.z}')
        else:
            self.logger.debug(f'nxt: {nxt}')
        
        self.bb_client.set(self.bb_key, nxt, overwrite=True)
        print(py_trees.display.unicode_blackboard())
        return py_trees.common.Status.SUCCESS
        
