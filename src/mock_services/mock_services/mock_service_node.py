from enum import Enum

import rclpy
from rclpy.node import Node

from tinker_decision_msgs.srv import Announce, Drop, Goto, GotoGrasp, Grasp, ObjectDetection, WaitForStart
from std_msgs.msg import Header


# modify to decide which service to mock
class Services(Enum):
    ANNOUNCE = True
    DROP = True
    GOTO = True
    GOTO_GRASP = True
    GRASP = True
    OBJ_DETECTION = True
    WAIT_FOR_START = True


class MockServices(Node):
    def __init__(self) -> None:
        super().__init__("mock services")

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
        if Services.WAIT_FOR_START:
            self.drop = self.create_service(WaitForStart, "wait_for_start", self.wait_for_start_callback)
    
    def announce_callback(self, request, response):
        self.get_logger().info(f'Incoming announcement request for message: {request.announcement_msg}')
        response.status = 0

        return response

    def drop_callback(self, request, response):
        if request.bin_point is not None:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "bin_point not received (is None)"
        
        return response

    def goto_callback(self, request, response):
        if request.target is not None:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "target not received (is None)"
        
        return response

    def goto_grasp_callback(self, request, response):
        if request.target is not None:
            response.status = 0
        else:
            response.status = 1
            response.error_msg = "target not received (is None)"
        
        return response

    def grasp_callback(self, request, response):
        self.get_logger().info(f'Incoming grasp request for prompt {request.prompt}')

        response.status = 0
        return response

    def obj_detection_callback(self, request, response):
        self.get_logger().info(f'Incoming object detection request for prompt {request.prompt}')

        if 'scan' in self.request.flags:
            response.status = 0
            response.header = Header()
            response.header.frame_id = ""
            response.header.stamp.sec = 0
            response.header.stamp.nanosec = 1

            # TODO: add a return object
            response.header.objects = []
        if 'find_for_grasp' in self.request.flags:
            # TODO: add return stuff
            pass


    def wait_for_start_callback(self, request, response):
        response.status = 0
        return response


def main():
    rclpy.init(args=None)

    mock_services = MockServices()

    rclpy.spin(mock_services)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
