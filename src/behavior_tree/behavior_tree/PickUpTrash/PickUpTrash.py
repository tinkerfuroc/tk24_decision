import py_trees as pytree

# from geometry_msgs.msg import PointStamped, PoseStamped
from behavior_tree.messages import *

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_Drop, BtNode_MoveArm
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_WaitForStart
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto, BtNode_GotoGrasp, BtNode_RelToAbs

from .CustomBtNodes import BtNode_ScanAndSave, BtNode_MoveArmInitial
from behavior_tree.Constants import *

# TODO: replace with actual pose of the 1m place
pose_1m = PoseStamped()

# TODO: replace with actual relative pose of turning 90 degrees
pose_turn90 = PoseStamped()

# prompts (subject to change)
PROMPT_ALL = "bottle . can"
# PROMPT_ALL = "giraffe"
# PROMPT_TRASH_BIN = "trashcan"
PROMPT_TRASH_BIN = "bowl"

# blackboard key constants (do NOT change!)
<<<<<<< HEAD
# KEY_POINT_BIN_ABS = "point_bin_abs"
KEY_POINT_BIN_REL = "point_bin_relative"
=======
KEY_POINT_BIN_REL = "point_bin_relative"
# KEY_POINT_BIN_ABS = "point_bin_abs"
>>>>>>> origin/ptrash_complGrasp
KEY_POINT_BIN_ABS = KEY_POINT_BIN_REL
KEY_POINT_TRASH = "point_trash"
KEY_TYPE_TRASH = "type_trash"
KEY_MOVE_ARM = "move_arm"


def createSearchForBin() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Search for trash bin", memory=True)

<<<<<<< HEAD
    if not WITHOUT_NAV_CONSTANTS:
        goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)
        root.add_child(goto_1m_pos)

    scan_for_bin = BtNode_ScanAndSave("Scan for bin", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN, filter_far=False)
    root.add_child(pytree.decorators.Retry("Retry", scan_for_bin, 999))

    # save_bin = BtNode_RelToAbs("Save absolute bin position", KEY_POINT_BIN_REL, KEY_POINT_BIN_ABS, True, service_name=SRV_REL_TO_ABS)
=======
    goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)
    root.add_child(goto_1m_pos)

    scan_for_bin = BtNode_ScanAndSave("Scan for bin", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN)
    root.add_child(scan_for_bin)

    # save_bin = BtNode_RelToAbs("Save absolute bin position", KEY_POINT_BIN_REL, KEY_POINT_BIN_ABS, True, service_name=SRV_REL_TO_ABS)

    # root.add_children([goto_1m_pos, scan_for_bin, save_bin])
>>>>>>> origin/ptrash_complGrasp

    return root


def createScanAndTurn():
    root = pytree.composites.Selector(name="Scan and Turn", memory=True)

    scan = BtNode_ScanAndSave("Scan & check if found", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    root.add_child(scan)

<<<<<<< HEAD
    if not WITHOUT_NAV_CONSTANTS:
        turn = BtNode_Goto("turn 90 degrees", None, service_name=SRV_GOTO, target=pose_turn90)
        turn_and_fail = pytree.decorators.SuccessIsFailure(name="success is fail", child=turn)
        root.add_child(turn_and_fail)
=======
    turn = BtNode_Goto("turn 90 degrees", None, service_name=SRV_GOTO, target=pose_turn90)
    turn_and_fail = pytree.decorators.SuccessIsFailure(name="success is fail", child=turn)
    root.add_child(turn_and_fail)
>>>>>>> origin/ptrash_complGrasp

    return root


def create_drop_node():
    root = pytree.composites.Sequence(name="Drop in Trash Can", memory=True)

<<<<<<< HEAD
    if DROP_EXISTS:
        # TODO: use absolute point of trash can
        goto_bin = BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP)

        find_bin = BtNode_ScanAndSave("Find trash can", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN)

        # TODO: replace with actual functioning node
        drop = BtNode_Drop("Drop in bin", KEY_POINT_BIN_REL, SRV_DROP)

        root.add_children([goto_bin, find_bin, drop])
    
    else:
        root.add_child(BtNode_Announce("Announce drop not working", None, service_name=SRV_ANNOUNCE, message="Drop service not working"))
=======
    goto_bin = BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP)
    root.add_child(goto_bin)

    find_bin = BtNode_ScanAndSave("Find trash can", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN)
    root.add_child(find_bin)

    drop = BtNode_Drop("Drop in bin", KEY_POINT_BIN_REL, SRV_DROP)
    root.add_child(drop)
>>>>>>> origin/ptrash_complGrasp

    return root

def pickAndDrop():
    # deprecated
    root = pytree.composites.Sequence(name="pickup trash and drop", memory=True)

    goto_trash = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)

    find_trash = BtNode_FindObj("find trash", KEY_TYPE_TRASH, "/", "trash", SRV_OBJ_DETECTION)

    grasp = BtNode_Grasp("Grasp trash", "/trash", service_name=SRV_GRASP)

    drop = create_drop_node()

    root.add_children([goto_trash, find_trash, grasp, drop])

    return root

def createScanAndGoto():
    root = pytree.composites.Sequence(name="Scan and Goto Trash", memory=True)
    scan_and_turn = createScanAndTurn()
    goto = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)
    root.add_children([scan_and_turn, goto])
    return root

def createGraspOnce():
    root = pytree.composites.Sequence(name="Grasp Once", memory=True)
    move_arm = BtNode_MoveArm("Move arm", service_name=SRV_MOVE_ARM, arm_pose_bb_key=KEY_MOVE_ARM)
    find_trash = BtNode_FindObj("find trash", KEY_TYPE_TRASH, "/", "trash", SRV_OBJ_DETECTION)
    grasp = BtNode_Grasp("Grasp trash", "/trash", service_name=SRV_GRASP)
    root.add_children([move_arm, find_trash, grasp])
    return root

def createGraspWithRetry():
    root = pytree.composites.Sequence(name="Pickup Trash", memory=True)
    # reset retry counter to 0
    reset_idx = BtNode_WriteToBlackboard("Reset retry index", "/", KEY_MOVE_ARM, None, 0)
    retry_grasp = pytree.decorators.Retry("Retry grasp", createGraspOnce(), len(SCAN_POSES))
    root.add_children([reset_idx, retry_grasp])
    return root

def createScanAndGraspWithRetry():
    root = pytree.composites.Sequence(name="Scan and Pickup", memory=True)
    root.add_child(BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP))
    root.add_child(createScanAndGoto())
    root.add_child(createGraspWithRetry())
    root.add_child(create_drop_node())
    root = pytree.decorators.Retry("Retry until success", root, 3)
    return root

def searchAndPickupOutside():
    root = pytree.composites.Sequence(name="Scan at 1m and pickup", memory=True)

    goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)

    scan = BtNode_ScanAndSave("Scan for trash", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    
    # goto_trash = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)

    # grasp = BtNode_Grasp("Grasp trash", KEY_TYPE_TRASH, service_name=SRV_GRASP)

    # drop = create_drop_node()

    pick_and_drop = pickAndDrop()

    # root.add_children([goto_1m_pos, scan, goto_trash, grasp, drop])

    root.add_children([goto_1m_pos, scan, pick_and_drop])
    
    return pytree.decorators.Repeat(name="Seach and Pickup at Outside", child=root, num_success=9)


def createPickUpTrashTree() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Pick Up Trash Root", memory=True)

    # reset_arm = BtNode_MoveArmInitial("Reset Arm", service_name=SRV_MOVE_ARM)
    # root.add_child(reset_arm)

    announce_start = BtNode_Announce("Announce Ready", None, service_name=SRV_ANNOUNCE, message="Ready")
    wait_for_start = BtNode_WaitForStart("Wait for Start", service_name=SRV_WAIT_FOR_START)

    search_for_bin = createSearchForBin()

    pickup_body = pytree.composites.Selector(name="Pickup until none left", memory=True)

    # search_at_bin = searchAndPickupAtBin()

    search_outside = pytree.decorators.FailureIsSuccess(name="succcess wrapper", child=searchAndPickupOutside())

    pickup_body.add_children([createScanAndGraspWithRetry(), search_outside])

    end_node = pytree.behaviours.Running(name="The end...")

    root.add_children([announce_start, wait_for_start, search_for_bin, pickup_body, end_node])
    # root.add_children([announce_start, search_for_bin, pickup_body, end_node])

    return root

# deprecated functions
# def searchAndPickupAtBin():
#     # deprecated
#     root = pytree.composites.Sequence(name="Scan at trash can and pickup", memory=True)
    
#     # TODO: use absolute point of trash can
#     goto_bin = BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP)

#     search = pytree.decorators.Retry(name="Keep Scanning and Turning", child=createScanAndTurn(), num_failures=4)

#     # goto_trash = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)

#     # grasp = BtNode_Grasp("Grasp trash", KEY_TYPE_TRASH, service_name=SRV_GRASP)

#     # drop = create_drop_node()

#     pick_and_drop = pickAndDrop()

#     # root.add_children([goto_bin, search, goto_trash, grasp, drop])
#     root.add_children([goto_bin, search, pick_and_drop])
    

#     return pytree.decorators.Repeat(name="Seach and Pickup at Bin", child=root, num_success=9)