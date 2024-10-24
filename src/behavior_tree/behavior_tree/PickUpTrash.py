import py_trees as pytree
import py_trees_ros as pytree_ros

from .Vision import BtNode_FindObj, BtNode_ScanFor
from .Manipulation import BtNode_Grasp
from .Audio import BtNode_Announce, BtNode_WaitForStart
from .BaseBehaviors import BtNode_WriteToBlackboard, BtNode_ClearBlackboard, BtNode_CheckIfEmpty



def createSearchForBin() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Search for trash bin", memory=True)
    goto_1m_pos = pytree_ros.action_clients.FromConstant("Goto 1m pos", "Mock_Goto", "mock_goto", "mock_dest")

    scan_for_bin = BtNode_ScanFor("Scan for bin", None, "Scan", "Bin", object="Trash Can")

    record_bin_pos = BtNode_WriteToBlackboard("Writing bin pos", "Positions", "Bin", None,  "Scan/Bin")

    root.add_children([goto_1m_pos, scan_for_bin, record_bin_pos])

    return root


def createScanAndTurn():
    root = pytree.composites.Sequence(name="Scan and Turn", memory=False)

    clear_bb = BtNode_ClearBlackboard(name="Clear Blackboard", bb_namespace="Scan", bb_key="FoundObjects")

    scan = BtNode_ScanFor(name="Scan", bb_source=None, bb_namespace="Scan", bb_key="FoundObjects", object="clear bottle . can . crumbled paper")

    check_found = pytree.composites.Selector(name="Check if scan found", memory=False)

    if_found = BtNode_CheckIfEmpty("check blackboard", "Scan/FoundObjects")
    ## TODO: replace with actual node that turns 90 degrees and returns with failure
    turn = pytree.behaviours.Failure(name="turn 90 degrees")

    check_found.add_children([if_found, turn])

    root.add_children([clear_bb, scan, check_found])

    return root


def create_grasp_node():
    root = pytree.composites.Sequence(name="Grasp", memory=False)

    #TODO: replace with actual functioning nodes
    move_to_grasp_pos = pytree.behaviours.Success(name="Move Arm to Scan Position")
    find_obj = BtNode_FindObj(name="Find object", bb_source=None, bb_namespace="Scan", bb_key="Grasp_scan")
    grasp = BtNode_Grasp(name="Grasp", bb_source="Scan/Grasp_scan")

    root.add_children([move_to_grasp_pos, find_obj, grasp])

    return root

def create_drop_node():
    root = pytree.composites.Sequence(name="Drop in Trash Can", memory=False)

    move_to_scan_pos = pytree.behaviours.Success(name="Move Arm to Scan Position")
    find_bin = BtNode_FindObj(name="Find trashcan", bb_source=None, bb_namespace="Scan", bb_key="TrashCan")

    move_to_drop_pos = pytree.behaviours.Success(name="Move Arm to Drop Trash Position")

    # TODO: replace with actual functioning node
    drop = pytree.behaviours.Success("Open Gripper")

    reset_arm = pytree.behaviours.Success("Reset Arm to normal position")

    root.add_children([move_to_scan_pos, find_bin, move_to_drop_pos, drop, reset_arm])

    return root


def searchAndPickupAtBin():
    root = pytree.composites.Sequence(name="Scan at trash can and pickup", memory=False)
    goto_pos = pytree_ros.action_clients.FromConstant("Goto Trash Can pos", "Mock_Goto", "mock_goto", "pos_TrashCan")

    search = pytree.decorators.Retry(name="Keep Scanning and Turning", child=createScanAndTurn(), num_failures=4)

    #TODO: replace with actual functioning nodes
    calc_stand_pos = pytree.behaviours.Success(name="Calc pickup stand location")
    goto_stand_pos = pytree_ros.action_clients.FromConstant("Goto stand location", "Mock_Goto", "mock_goto", "pos_StandPos")

    grasp = create_grasp_node()

    calc_drop_pos = pytree.behaviours.Success(name="Calc drop object position")
    goto_drop_pos = pytree_ros.action_clients.FromConstant("Goto drop trash location", "Mock_Goto", "mock_goto", "pos_DropCan")

    drop = create_drop_node()

    root.add_children([goto_pos, search, calc_stand_pos, goto_stand_pos, grasp, calc_drop_pos, goto_drop_pos, drop])
    return pytree.decorators.Repeat(name="Seach and Pickup at Bin", child=root, num_success=9)


def searchAndPickupOutside():
    root = pytree.composites.Sequence(name="Scan at 1m and pickup", memory=False)
    goto_pos = pytree_ros.action_clients.FromConstant("Goto Ouside pos", "Mock_Goto", "mock_goto", "pos_1m")

    scan = BtNode_ScanFor(name="Scan", bb_source=None, bb_namespace="Scan", bb_key="FoundObjects", object="clear bottle . can . crumbled paper")

    check_found = BtNode_CheckIfEmpty("check if blackboard contains results", "Scan/FoundObjects")

    #TODO: replace with actual functioning nodes
    calc_stand_pos = pytree.behaviours.Success(name="Calc pickup stand location")
    goto_stand_pos = pytree_ros.action_clients.FromConstant("Goto stand location", "Mock_Goto", "mock_goto", "pos_StandPos")

    grasp = create_grasp_node()

    calc_drop_pos = pytree.behaviours.Success(name="Calc drop object position")
    goto_drop_pos = pytree_ros.action_clients.FromConstant("Goto drop trash location", "Mock_Goto", "mock_goto", "pos_DropCan")

    drop = create_drop_node()

    root.add_children([goto_pos, scan, check_found, calc_stand_pos, goto_stand_pos, grasp, calc_drop_pos, goto_drop_pos, drop])
    return pytree.decorators.Repeat(name="Seach and Pickup at Outside", child=root, num_success=9)


def createPickUpTrashTree() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Pick Up Trash Root", memory=True)

    announce_start = BtNode_Announce("Announce Ready", None, message="Ready")
    wait_for_start = BtNode_WaitForStart("Wait for Start")

    search_for_bin = createSearchForBin()

    pickup_body = pytree.composites.Selector(name="Pickup until none left", memory=False)

    search_at_bin = searchAndPickupAtBin()

    search_outside = searchAndPickupOutside()

    pickup_body.add_children([search_at_bin, search_outside])

    root.add_children([announce_start, wait_for_start, search_for_bin, pickup_body])

    return root