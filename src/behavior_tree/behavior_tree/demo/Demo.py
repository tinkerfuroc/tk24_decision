import py_trees
from behavior_tree.messages import *
from behavior_tree.Constants import *

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_Drop, BtNode_MoveArm
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_WaitForStart
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto, BtNode_GotoGrasp, BtNode_RelToAbs
from behavior_tree.PickUpTrash.CustomBtNodes import BtNode_ScanAndSave

from .CustomBtNodes import BtNode_Goto_Iterate

POSE_LOOK_AT_BIN = PoseStamped()
POSE_SCAN_FOR_TRASH = PoseStamped()

TRASH_POSES : list[PoseStamped]= []
trash_idx = 0

PROMPT_TRASH = "bottle"


KEY_MOVE_ARM = "move_arm"


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

    return root

    

def createDemo() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Demo", memory=True)

    announce_start = BtNode_Announce("Announce Ready", None, service_name=SRV_ANNOUNCE, message="Ready")
    wait_for_start = BtNode_WaitForStart("Wait for Start", service_name=SRV_WAIT_FOR_START)
    root.add_children([announce_start, wait_for_start])

    pickUpLoop = recursivePickUp()
    root.add_child(pickUpLoop)

    root.add_child(py_trees.behaviours.Running("the end"))

    return root

