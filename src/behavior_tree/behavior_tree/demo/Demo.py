import py_trees
import rclpy
from behavior_tree.messages import *
from behavior_tree.Constants import *

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_Drop, BtNode_MoveArm
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_WaitForStart
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto, BtNode_GotoGrasp, BtNode_RelToAbs
from behavior_tree.PickUpTrash.CustomBtNodes import BtNode_ScanAndSave

from std_msgs.msg import Header

from .CustomBtNodes import BtNode_Goto_Iterate, BtNode_MoveArmInitial

header = Header(stamp=rclpy.time.Time().to_msg(), frame_id="map")

POSE_LOOK_AT_BIN = PoseStamped(header=header,
                               pose=Pose(
                                 position=Point(x=-0.5781041668780074, y=-1.7940791189303245, z=0.24705713264789592),
                                 orientation=Quaternion(x=0.026694251266043457, y=-0.14872128006284202, z=0.927123250742749, w=0.3429458787301754)
                               ))
# -0.5781041668780074 -1.7940791189303245 0.24705713264789592
# 0.026694251266043457 -0.14872128006284202 0.927123250742749 0.3429458787301754

POSE_SCAN_FOR_TRASH = PoseStamped(header=header,
                               pose=Pose(
                                 position=Point(x=-0.30988842013134965, y=-2.310319077312238, z=0.09273813115592805),
                                 orientation=Quaternion(x=0.13708987438778422, y=0.05665367347669159, z=-0.5013310957760906, w=0.85244581061196)
                               ))
# -0.30988842013134965 -2.310319077312238 0.09273813115592805
# 0.13708987438778422 0.05665367347669159 -0.5013310957760906 0.85244581061196

TRASH_POSES : list[PoseStamped]= [
    PoseStamped(header=header,
                pose=Pose(
                  position=Point(x=1.0229603052139282, y=-3.1526005268096924, z=0.004986504092812538),
                  orientation=Quaternion(x=-0.030440579130694288, y=0.012413205189132026, z=-0.5101873850381369, w=0.8594347651957466)
                ))
]
# 1.0229603052139282 -3.1526005268096924 0.004986504092812538
# -0.030440579130694288 0.012413205189132026 -0.5101873850381369 0.8594347651957466
trash_idx = 0

BIN_POINT = PointStamped()

PROMPT_TRASH = "bottle . can"

# blackboard constants
KEY_MOVE_ARM = "move_arm"

DROP_POINT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="base_link"), point=Point(x=0.8, y=0.3, z=0.2))


KEY_POINT_BIN_REL = "point_bin_relative"


def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    move_arm = BtNode_MoveArm("Move arm", service_name=SRV_MOVE_ARM, arm_pose_bb_key=KEY_MOVE_ARM)
    find_trash = BtNode_FindObj("find trash", "", "", "trash", SRV_OBJ_DETECTION, PROMPT_TRASH)
    grasp = BtNode_Grasp("Grasp trash", "/trash", service_name=SRV_GRASP)
    root.add_children([move_arm, find_trash, grasp])
    return root

def createGraspWithRetry():
    root = py_trees.composites.Sequence(name="Pickup Trash", memory=True)
    # reset retry counter to 0
    reset_idx = BtNode_WriteToBlackboard("Reset retry index", "/", KEY_MOVE_ARM, None, 0)
    retry_grasp = py_trees.decorators.Retry("Retry grasp", createGraspOnce(), len(SCAN_POSES))
    root.add_children([reset_idx, retry_grasp])
    return root

def recursivePickUp():
    pickAndDrop = py_trees.composites.Sequence("pick up and drop", True)
    root = py_trees.decorators.Repeat(name="recursive pick up", child=pickAndDrop, num_success=len(TRASH_POSES))

    turn_around = BtNode_Goto("get to scanning position", None, SRV_GOTO, POSE_SCAN_FOR_TRASH)
    pickAndDrop.add_child(turn_around)

    scan = BtNode_ScanAndSave("scan for trash", None, "/point", "/type", SRV_OBJ_DETECTION, PROMPT_TRASH)
    pickAndDrop.add_child(scan)

    goto_trash = BtNode_Goto_Iterate("goto trash position", TRASH_POSES, SRV_GOTO)
    pickAndDrop.add_child(goto_trash)

    grasp = createGraspWithRetry()
    pickAndDrop.add_child(grasp)

    goto_bin = BtNode_Goto("go to trash can", None, SRV_GOTO, POSE_LOOK_AT_BIN)
    pickAndDrop.add_child(goto_bin)

    drop = BtNode_Drop("drop trash", "", SRV_DROP, BIN_POINT)
    pickAndDrop.add_child(drop)

    return root

    

def createDemo() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Demo", memory=True)

    reset_arm = BtNode_MoveArmInitial("Reset Arm", service_name=SRV_MOVE_ARM)
    root.add_child(reset_arm)

    announce_start = BtNode_Announce("Announce Ready", None, service_name=SRV_ANNOUNCE, message="Ready")
    wait_for_start = BtNode_WaitForStart("Wait for Start", service_name=SRV_WAIT_FOR_START)
    # root.add_children([announce_start, wait_for_start])
    # root.add_children([announce_start])

    pickUpLoop = recursivePickUp()
    root.add_child(pickUpLoop)

    root.add_child(py_trees.behaviours.Running("the end"))

    return root

def createDemoGrasp() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Demo Grasp", memory=True)

    reset_arm = BtNode_MoveArmInitial("Reset Arm", service_name=SRV_MOVE_ARM)
    root.add_child(reset_arm)

    announce_start = BtNode_Announce("Announce Ready", None, service_name=SRV_ANNOUNCE, message="Ready")
    wait_for_start = BtNode_WaitForStart("Wait for Start", service_name=SRV_WAIT_FOR_START)
    root.add_children([announce_start, wait_for_start])

    graspAndDrop = py_trees.composites.Sequence("grasp and drop", True)

    grasp = createGraspWithRetry()
    pickUpLoop = py_trees.decorators.Retry("Retry Grasp", grasp, 5)
    graspAndDrop.add_child(pickUpLoop)

    # find_bin = BtNode_ScanAndSave("Find trash can", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object="cardboard box")
    # graspAndDrop.add_child(find_bin)

    # drop = BtNode_Drop("Drop in bin", KEY_POINT_BIN_REL, SRV_DROP)
    drop = BtNode_Drop("drop trash", "", SRV_DROP, DROP_POINT)
    graspAndDrop.add_child(drop)

    root.add_child(py_trees.decorators.FailureIsSuccess("success wrapper", py_trees.decorators.Repeat("repeat until fail", graspAndDrop, -1)))

    root.add_child(py_trees.behaviours.Running("the end"))

    return root
