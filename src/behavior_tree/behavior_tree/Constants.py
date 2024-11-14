import math
# service names (matching the mocked services, please prioritize changing the service names in your own node INSTEAD of changing them here)
SRV_ANNOUNCE = "announce"
SRV_DROP = "drop"
SRV_GOTO = "go_to"
SRV_GOTO_GRASP = "go_to_grasp"
SRV_GRASP = "start_grasp"
# SRV_GRASP = "grasp"
SRV_OBJ_DETECTION = "object_detection"
# SRV_REL_TO_ABS = "rel_to_abs"
SRV_WAIT_FOR_START = "wait_for_start"
SRV_MOVE_ARM = "arm_joint_service"

PRINT_BLACKBOARD = False
PRINT_DEBUG = True

DROP_EXISTS = False
WITHOUT_NAV_CONSTANTS = True

# joint poses in degrees
SCAN_POSES_D = [
    [0.0, 50.7, 0.0, 12.6, 0.0, -87.8, 0.0],
    [0.0, 65.6, 0.0, 38.4, 0.0, -87.9, 0.0],
    # [0., 0.6, 0., -0.3, 0., -44.4, 0.],
    [0.0, 62.8, 0.4, 48.2, -4.4, -85.3, 0.0]
    # [1.2 , -15.8 , 0.0 , -1.5,1.0,-32.4,0.0]
    # [0., 0.6, 0, 20.2, 0., -24.3, 1.4],
    # [0., 13.3, 0, 44.2, 0., -11.2, 1.4],
    # [-0.8, -9.6, 3, 59.4, -4.7, 34.8, 4.5]
]
SCAN_POSES = [
    [x / 180 * math.pi for x in p] for p in SCAN_POSES_D
]

