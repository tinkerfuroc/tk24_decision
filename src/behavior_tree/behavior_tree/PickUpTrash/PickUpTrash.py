import py_trees as pytree

from geometry_msgs.msg import PointStamped, PoseStamped

from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_Drop
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_WaitForStart
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto, BtNode_GotoGrasp, BtNode_RelToAbs

from .CustomBtNodes import BtNode_ScanAndSave
from behavior_tree.Constants import *

# TODO: replace with actual pose of the 1m place
pose_1m = PoseStamped()

# TODO: replace with actual relative pose of turning 90 degrees
pose_turn90 = PoseStamped()

# prompts (subject to change)
PROMPT_ALL = "clear bottle . wrinkled paper . can"
PROMPT_TRASH_BIN = "trash bin"

# blackboard key constants (do NOT change!)
KEY_POINT_BIN_ABS = "point_bin_abs"
KEY_POINT_BIN_REL = "point_bin_relative"
KEY_POINT_TRASH = "point_trash"
KEY_TYPE_TRASH = "type_trash"


def createSearchForBin() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Search for trash bin", memory=True)

    goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)

    scan_for_bin = BtNode_ScanAndSave("Scan for bin", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN)

    save_bin = BtNode_RelToAbs("Save absolute bin position", KEY_POINT_BIN_REL, KEY_POINT_BIN_ABS, True, service_name=SRV_REL_TO_ABS)

    root.add_children([goto_1m_pos, scan_for_bin, save_bin])

    return root


def createScanAndTurn():
    root = pytree.composites.Selector(name="Scan and Turn", memory=True)

    scan = BtNode_ScanAndSave("Scan & check if found", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    # scan = pytree.behaviours.Running("running")

    turn = BtNode_Goto("turn 90 degrees", None, service_name=SRV_GOTO, target=pose_turn90)
    turn_and_fail = pytree.decorators.SuccessIsFailure(name="success is fail", child=turn)
    # turn_and_fail = pytree.behaviours.Success("success")

    root.add_children([scan, turn_and_fail])

    return root


def create_drop_node():
    root = pytree.composites.Sequence(name="Drop in Trash Can", memory=True)

    # TODO: use absolute point of trash can
    goto_bin = BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP)

    find_bin = BtNode_ScanAndSave("Find trash can", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN)

    # TODO: replace with actual functioning node
    drop = BtNode_Drop("Drop in bin", KEY_POINT_BIN_REL, SRV_DROP)

    root.add_children([goto_bin, find_bin, drop])

    return root


def searchAndPickupAtBin():
    root = pytree.composites.Sequence(name="Scan at trash can and pickup", memory=True)
    
    # TODO: use absolute point of trash can
    goto_bin = BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP)

    search = pytree.decorators.Retry(name="Keep Scanning and Turning", child=createScanAndTurn(), num_failures=4)

    goto_trash = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)

    grasp = BtNode_Grasp("Grasp trash", KEY_TYPE_TRASH, service_name=SRV_GRASP)

    drop = create_drop_node()

    root.add_children([goto_bin, search, goto_trash, grasp, drop])
    
    # root.add_child(pytree.behaviours.Failure("fail"))

    return pytree.decorators.Repeat(name="Seach and Pickup at Bin", child=root, num_success=9)
    # return root
    # return pytree.decorators.Repeat(name="Seach and Pickup at Bin", child=search, num_success=9)


def searchAndPickupOutside():
    root = pytree.composites.Sequence(name="Scan at 1m and pickup", memory=True)

    goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)

    scan = BtNode_ScanAndSave("Scan for trash", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    
    goto_trash = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)

    grasp = BtNode_Grasp("Grasp trash", KEY_TYPE_TRASH, service_name=SRV_GRASP)

    drop = create_drop_node()

    root.add_children([goto_1m_pos, scan, goto_trash, grasp, drop])
    
    return pytree.decorators.Repeat(name="Seach and Pickup at Outside", child=root, num_success=9)


def createPickUpTrashTree() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Pick Up Trash Root", memory=True)

    announce_start = BtNode_Announce("Announce Ready", None, service_name=SRV_ANNOUNCE, message="Ready")
    wait_for_start = BtNode_WaitForStart("Wait for Start", service_name=SRV_WAIT_FOR_START)

    search_for_bin = createSearchForBin()

    pickup_body = pytree.composites.Selector(name="Pickup until none left", memory=True)

    search_at_bin = searchAndPickupAtBin()

    search_outside = pytree.decorators.FailureIsSuccess(name="succcess wrapper", child=searchAndPickupOutside())

    pickup_body.add_children([search_at_bin, search_outside])

    end_node = pytree.behaviours.Running(name="The end...")

    root.add_children([announce_start, wait_for_start, search_for_bin, pickup_body, end_node])

    return root