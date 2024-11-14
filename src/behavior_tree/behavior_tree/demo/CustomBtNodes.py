from behavior_tree.messages import PoseStamped
import py_trees
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto
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
        self.logger.debug(f"Set up goto iterating of length {len(self.targets)}")

    def initialise(self) -> None:
        request = Goto.Request()
        request.target = self.targets[self.idx]
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Goto iterating, currently at index {self.idx}"
        self.idx = (self.idx + 1) % len(self.targets)