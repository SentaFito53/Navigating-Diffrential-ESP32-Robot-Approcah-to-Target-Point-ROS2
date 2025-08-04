import rclpy
from rclpy.node import Node
from aktual.srv import GetTargetPos
from geometry_msgs.msg import Point

class GetTargetPosService(Node):
    def __init__(self):
        super().__init__('get_target_pos_service')
        self.srv = self.create_service(GetTargetPos, 'get_target_pos', self.get_target_pos_callback)
        self.sub = self.create_subscription(Point, '/target_pos', self.target_pos_callback, 10)
        self.latest_position = Point()

    def target_pos_callback(self, msg):
        self.latest_position = msg

    def get_target_pos_callback(self, request, response):
        response.position = self.latest_position
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GetTargetPosService()
    rclpy.spin(node)
    rclpy.shutdown()
