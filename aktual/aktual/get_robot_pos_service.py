import rclpy
from rclpy.node import Node
from aktual.srv import GetRobotPos
from geometry_msgs.msg import Point

class GetRobotPosService(Node):
    def __init__(self):
        super().__init__('get_robot_pos_service')
        self.srv = self.create_service(GetRobotPos, 'get_robot_pos', self.get_robot_pos_callback)
        self.sub = self.create_subscription(Point, '/robot_pos', self.robot_pos_callback, 10)
        self.latest_position = Point()

    def robot_pos_callback(self, msg):
        self.latest_position = msg

    def get_robot_pos_callback(self, request, response):
        response.position = self.latest_position
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GetRobotPosService()
    rclpy.spin(node)
    rclpy.shutdown()
