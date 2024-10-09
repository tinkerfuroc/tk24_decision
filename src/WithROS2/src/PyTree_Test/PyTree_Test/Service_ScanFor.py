import rclpy
from rclpy.node import Node

from mock_msgs.srv import ScanFor

import time

class ScanForService(Node):
    def __init__(self):
        super().__init__('vision_scan_for')
        self.srv = self.create_service(ScanFor, 'scan_for', self.scan_for_callback)

    def scan_for_callback(self, request, response):
        self.get_logger().info(f'Incoming request\nscanning for: {request.name}')
        time.sleep(2)
        response.status = 0

        return response

def main(args=None):
    rclpy.init(args=args)

    scanfor_service = ScanForService()

    rclpy.spin(scanfor_service)

    rclpy.shutdown()

if __name__=='__main__':
    main()