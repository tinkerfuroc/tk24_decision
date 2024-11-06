from enum import Enum
from time import sleep

import rclpy
from rclpy.node import Node

from tinker_decision_msgs.srv import Announce, Drop, Goto, GotoGrasp, Grasp, ObjectDetection, RelToAbs, WaitForStart
from tinker_vision_msgs.msg import Object
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, PoseStamped


# modify to decide which services to mock
class Services(Enum):
    ANNOUNCE = True
    DROP = True
    GOTO = True
    GOTO_GRASP = True
    GRASP = True
    OBJ_DETECTION = True
    REL_TO_ABS = True
    WAIT_FOR_START = True


class MockServices(Node):
    def __init__(self) -> None:
        super().__init__("mock_services")

        if Services.ANNOUNCE:
            self.announce = self.create_service(Announce, "announce", self.announce_callback)
        if Services.DROP:
            self.drop = self.create_service(Drop, "drop", self.drop_callback)
        if Services.GOTO:
            self.drop = self.create_service(Goto, "goto", self.goto_callback)
        if Services.GOTO_GRASP:
            self.drop = self.create_service(GotoGrasp, "goto_grasp", self.goto_grasp_callback)
        if Services.GRASP:
            self.drop = self.create_service(Grasp, "grasp", self.grasp_callback)
        if Services.OBJ_DETECTION:
            self.drop = self.create_service(ObjectDetection, "object_detection", self.obj_detection_callback)
        if Services.REL_TO_ABS:
            self.rel_to_abs = self.create_service(RelToAbs, "rel_to_abs", self.rel_to_abs_callback)
        if Services.WAIT_FOR_START:
            self.drop = self.create_service(WaitForStart, "wait_for_start", self.wait_for_start_callback)
    
    def announce_callback(self, request, response):
        self.get_logger().info(f'Incoming announcement request for message: {request.announcement_msg}')
        if request.announcement_msg:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "Error, no announcement message sent"

        return response

    def drop_callback(self, request, response):
        self.get_logger().info(f'Incoming drop request for bin point: {request.bin_point}')
        if request.bin_point is not None:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "bin_point not received (is None)"
        
        return response

    def goto_callback(self, request, response):
        self.get_logger().info(f'Incoming goto request for target pose: {request.target}')
        if request.target is not None:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "target not received (is None)"
        
        return response

    def goto_grasp_callback(self, request, response):
        self.get_logger().info(f'Incoming goto_grasp request for target point: {request.target}')
        if request.target is not None:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "target not received (is None)"
        
        return response

    def grasp_callback(self, request, response):
        self.get_logger().info(f'Incoming grasp request')

        response.success = True

        return response

    def obj_detection_callback(self, request, response):
        self.get_logger().info(f'Incoming object detection request for prompt {request.prompt}')

        if 'scan' in request.flags:
            response.status = 0
            response.header = Header()
            response.header.frame_id = "0"
            response.header.stamp.sec = 0
            response.header.stamp.nanosec = 1

            response.objects = []
            object = Object()
            response.objects.append(object)
        elif 'find_for_grasp' in request.flags:
            # TODO: add return stuff
            response.status = 0
            response.rgb_image = Image()
            response.depth_image = Image()
            response.segments = []
            response.segments.append(Image())

        else:
            self.get_logger().info(f"Error: no valid flags passed! (flags: {request.flags})")
            response.status = 1
            response.error_msg = "No valid flags passed"

        return response

    def rel_to_abs_callback(self, request, response):
        if "point" in request.flags:
            if request.point_rel is None:
                self.get_logger().info(f"Error: 'point' passed as flag but no 'point_rel' given!")
                response.status = 2
                response.error_msg = "No valid relative point passed"
            else:
                response.point_abs = PointStamped()
        elif "pose" in request.flags:
            if request.pose_rel is None:
                self.get_logger().info(f"Error: 'poposeint' passed as flag but no 'pose_rel' given!")
                response.status = 2
                response.error_msg = "No valid relative pose passed"
            else:
                response.pose_abs = PoseStamped()
        else:
            self.get_logger().info(f"Error: no valid flags passed! (flags: {request.flags})")
            response.status = 1
            response.error_msg = "No valid flags passed"
        
        return response

    def wait_for_start_callback(self, request, response):
        self.get_logger().info(f'Incoming wait for start request')
        response.status = 0
        return response


def main():
    rclpy.init(args=None)

    mock_services = MockServices()

    rclpy.spin(mock_services)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
