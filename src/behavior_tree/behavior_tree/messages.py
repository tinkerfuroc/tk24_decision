from tinker_vision_msgs.srv import ObjectDetection
# from tinker_decision_msgs.srv import ObjectDetection
from tk_nav_interfaces.srv import Goto, GotoGrasp, RelToAbs
# from tinker_decision_msgs.srv import Goto, GotoGrasp, RelToAbs
from tinker_decision_msgs.srv import Drop
# from tinker_decision_msgs.srv import Grasp
from tinker_arm_msgs.srv import Grasp
# from tinker_decision_msgs.srv import Announce, WaitForStart
from tinker_audio_msgs.srv import TextToSpeech, WaitForStart

from geometry_msgs.msg import PointStamped, PoseStamped
from tinker_arm_msgs.srv import ArmJointService