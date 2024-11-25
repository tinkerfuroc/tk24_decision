import py_trees as pytree

# from geometry_msgs.msg import PointStamped, PoseStamped
from behavior_tree.messages import *

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_Drop, BtNode_MoveArm
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_WaitForStart
from behavior_tree.TemplateNodes.Navigation import BtNode_Goto, BtNode_GotoGrasp, BtNode_RelToAbs

from .CustomBtNodes import BtNode_ScanAndSave, BtNode_MoveArmSet
from behavior_tree.Constants import *


pose_1m = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=3.0526, y=0.1497, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.1498672, w=0.9887061)))
# pose_1m = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                       pose=Pose(position=Point(x=-0.4848339, y=13.9184657, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.9755, w=0.219867)))

pose_scan_left = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=3.815144419, y=0.750634, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.68261860, w=0.730774818)))
pose_scan_right = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.2840204, y=0.467063208, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.122546063, w=0.9924628)))
    


# pose_finish = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                           pose=Pose(position=Point(x=-0.96623, y=4.12413, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.919, w=0.395)))

pose_finish = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                          pose=Pose(position=Point(x=-0.8918205, y=4.319702, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.3529195, w=0.9356536)))
# pose_1m = pose_finish
# pose_finish = pose_1m

# TODO: replace with actual relative pose of turning 90 degrees
pose_turn90 = PoseStamped()

# prompts (subject to change)
# PROMPT_ALL = "bottle . can . white ball . blue ball . orange ball . yellow ball . plastic bag"
PROMPT_ALL = "bottle . can"
# PROMPT_ALL = "giraffe"
# PROMPT_TRASH_BIN = "cardboard box"
PROMPT_TRASH_BIN = "trashcan"

# blackboard key constants (do NOT change!)
# KEY_POINT_BIN_ABS = "point_bin_abs"
KEY_POINT_BIN_REL = "point_bin_relative"
KEY_POINT_BIN_ABS = KEY_POINT_BIN_REL
KEY_POINT_TRASH = "point_trash"
KEY_TYPE_TRASH = "type_trash"
KEY_MOVE_ARM = "move_arm"


def scanWithArm() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Scan with arm", memory=True)

    move_arm_up = BtNode_MoveArmSet("Move arm up", service_name=SRV_MOVE_ARM, arm_joint_pose=LOOK_POSE)
    root.add_child(move_arm_up)

    scan = BtNode_ScanAndSave("Scan with arm", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL, use_orbbec=False)
    root.add_child(scan)

    move_arm_back = BtNode_MoveArmSet("Move arm back", service_name=SRV_MOVE_ARM)
    root.add_child(move_arm_back)

    return root

def createSearchForBin() -> pytree.composites.Sequence:
    root = pytree.composites.Selector(name="Search for trash bin", memory=True)

    # if not WITHOUT_NAV_CONSTANTS and RUNNING_NAV:
    #     goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)
    #     root.add_child(goto_1m_pos)
    scan1 = pytree.composites.Sequence(name="Scan 1", memory=True)
    goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)
    scan1.add_child(goto_1m_pos)
    scan_for_bin = BtNode_ScanAndSave("Scan for bin", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN, filter_far=False, transform_to_map=True)
    scan1.add_child(scan_for_bin)
    root.add_child(scan1)

    scan2 = pytree.composites.Sequence(name="Scan 2", memory=True)
    goto_left_pos = BtNode_Goto("Goto left pos", None, service_name=SRV_GOTO, target=pose_scan_left)
    scan2.add_child(goto_left_pos)
    scan_for_bin = BtNode_ScanAndSave("Scan for bin", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN, filter_far=False, transform_to_map=True)
    scan2.add_child(scan_for_bin)
    root.add_child(scan2)

    scan3 = pytree.composites.Sequence(name="Scan 3", memory=True)
    goto_right_pos = BtNode_Goto("Goto right pos", None, service_name=SRV_GOTO, target=pose_scan_right)
    scan3.add_child(goto_right_pos)
    scan_for_bin = BtNode_ScanAndSave("Scan for bin", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN, filter_far=False, transform_to_map=True)
    scan3.add_child(scan_for_bin)
    root.add_child(scan3)

    # save_bin = BtNode_RelToAbs("Save absolute bin position", KEY_POINT_BIN_REL, KEY_POINT_BIN_ABS, True, service_name=SRV_REL_TO_ABS)

    return root


def createScanAndTurn(with_arm=False):
    root = pytree.composites.Selector(name="Scan and Turn", memory=True)

    scan = BtNode_ScanAndSave("Scan & check if found", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    root.add_child(scan)

    if with_arm:
        scan_with_arm = scanWithArm()
        root.add_child(scan_with_arm)

    if not WITHOUT_NAV_CONSTANTS and RUNNING_NAV:
        turn = BtNode_Goto("turn 90 degrees", None, service_name=SRV_GOTO, target=pose_turn90)
        turn_and_fail = pytree.decorators.SuccessIsFailure(name="success is fail", child=turn)
        root.add_child(turn_and_fail)

    return root


def create_drop_node():
    root = pytree.composites.Sequence(name="Drop in Trash Can", memory=True)

    if DROP_EXISTS:
        # TODO: use absolute point of trash can
        if RUNNING_NAV:
            goto_bin = BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP)
            root.add_child(goto_bin)

        # find_bin = BtNode_ScanAndSave("Find trash can", None, KEY_POINT_BIN_REL, "temp", service_name=SRV_OBJ_DETECTION, object=PROMPT_TRASH_BIN)
        # root.add_child(find_bin)

        drop = BtNode_Drop("Drop in bin", KEY_POINT_BIN_REL, SRV_DROP)
        root.add_child(drop)
    
    else:
        root.add_child(BtNode_Announce("Announce drop not working", None, service_name=SRV_ANNOUNCE, message="Drop service not working"))

    return root

def pickAndDrop():
    # deprecated
    root = pytree.composites.Sequence(name="pickup trash and drop", memory=True)
    if RUNNING_NAV:
        goto_trash = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)
        root.add_child(goto_trash)
    find_trash = BtNode_FindObj("find trash", KEY_TYPE_TRASH, "/", "trash", SRV_OBJ_DETECTION)
    grasp = BtNode_Grasp("Grasp trash", "/trash", service_name=SRV_GRASP)
    drop = create_drop_node()

    root.add_children([find_trash, grasp, drop])

    return root

def createScanAndGoto():
    root = pytree.composites.Sequence(name="Scan and Goto Trash", memory=True)
    scan_once = createScanAndTurn()
    search = pytree.decorators.Retry(name="Keep Scanning and Turning", child=createScanAndTurn(True), num_failures=4)
    root.add_child(search)
    if RUNNING_NAV:
        goto = BtNode_GotoGrasp("Got to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP)
        root.add_child(goto)
    return root

def createScanAndGotoWithoutTurn():
    root = pytree.composites.Sequence(name="Scan and goto trash", memory=True)

    scan = pytree.composites.Selector(name="Scan", memory=True)
    scan_orbbec = BtNode_ScanAndSave("Scan & check if found", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    scan.add_child(scan_orbbec)
    scan.add_child(scanWithArm())

    root.add_child(scan)

    root.add_child(BtNode_GotoGrasp("Go to trash point", KEY_POINT_TRASH, service_name=SRV_GOTO_GRASP))

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
    if RUNNING_NAV:
        root.add_child(BtNode_GotoGrasp("Got to Trashcan", KEY_POINT_BIN_ABS, service_name=SRV_GOTO_GRASP))
    root.add_child(createScanAndGoto())
    root.add_child(createGraspWithRetry())
    root.add_child(create_drop_node())
    root = pytree.decorators.Retry("Retry until success", root, 3)
    return root

def createGraspAndDrop():
    root = pytree.composites.Sequence(name="Scan and Pickup", memory=True)
    root.add_child(BtNode_MoveArmSet("Move arm back", service_name=SRV_MOVE_ARM))
    root.add_child(BtNode_Goto("Go outside", None, service_name=SRV_GOTO, target=pose_1m))
    root.add_child(createScanAndGotoWithoutTurn())
    root.add_child(createGraspWithRetry())
    root.add_child(BtNode_MoveArmSet("Move arm back", service_name=SRV_MOVE_ARM))
    root.add_child(BtNode_Goto("Go outside", None, service_name=SRV_GOTO, target=pose_1m))
    root.add_child(createSearchForBin())
    root.add_child(create_drop_node())
    root = pytree.decorators.Retry("Retry until success", root, 3)
    return root

def searchAndPickupOutside():
    root = pytree.composites.Sequence(name="Scan at 1m and pickup", memory=True)

    if not WITHOUT_NAV_CONSTANTS and RUNNING_NAV:
        goto_1m_pos = BtNode_Goto("Goto 1m pos", None, service_name=SRV_GOTO, target=pose_1m)
        root.add_child(goto_1m_pos)
    
    scan = BtNode_ScanAndSave("Scan for trash", None, KEY_POINT_TRASH, KEY_TYPE_TRASH, service_name=SRV_OBJ_DETECTION, object=PROMPT_ALL)
    root.add_child(scan)
    pick_and_drop = pickAndDrop()
    root.add_child(pick_and_drop)
    
    return pytree.decorators.Repeat(name="Seach and Pickup at Outside", child=root, num_success=9)


def createPickUpTrashTree() -> pytree.composites.Sequence:
    root = pytree.composites.Sequence(name="Pick Up Trash Root", memory=True)

    reset_arm = BtNode_MoveArmSet("Reset Arm", service_name=SRV_MOVE_ARM)
    root.add_child(reset_arm)
    reser_arm2 = BtNode_MoveArmSet("Reset Arm 2", service_name=SRV_MOVE_ARM, arm_joint_pose=INITIAL_POSE2)
    root.add_child(reser_arm2)

    announce_start = BtNode_Announce("Announce Ready", None, service_name=SRV_ANNOUNCE, message="Ready")
    root.add_child(announce_start)
    wait_for_start = BtNode_WaitForStart("Wait for Start", service_name=SRV_WAIT_FOR_START)
    root.add_child(wait_for_start)

    pickup = pytree.decorators.Repeat("repeat grasp", createGraspAndDrop(), 9)
    root.add_child(pytree.decorators.FailureIsSuccess("failure is success", pickup))

    # search_for_bin = createSearchForBin()

    # pickup_body = pytree.composites.Selector(name="Pickup until none left", memory=True)

    # # search_at_bin = searchAndPickupAtBin()

    # search_outside = pytree.decorators.FailureIsSuccess(name="succcess wrapper", child=searchAndPickupOutside())

    # pickup_body.add_children([createScanAndGraspWithRetry(), search_outside])
    root.add_child(BtNode_MoveArmSet("Move arm back", service_name=SRV_MOVE_ARM))
    root.add_child(BtNode_Goto("Go to finish point", None, service_name=SRV_GOTO, target=pose_finish))

    end_node = pytree.behaviours.Running(name="The end...")
    root.add_child(end_node)

    # root.add_children([announce_start, wait_for_start, search_for_bin, pickup_body, end_node])
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