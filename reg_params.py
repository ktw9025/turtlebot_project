import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class RegParams(Node):
    def __init__(self):
        super().__init__('reg_params')
        #qos_profile = QoSProfile(depth=10)

        #pick 파라미터
        self.declare_parameter('pick_place', 'place3') # yet or already
        # self.declare_parameter('pick_goal0', 'yet') # yet or already
        # self.declare_parameter('pick_goal1', 'yet')
        # self.declare_parameter('pick_goal2', 'yet')
        #put 파라미터
        self.declare_parameter('put_place', 'place3') # yet or already
        # self.declare_parameter('put_goal0', 'yet') # yet or already
        # self.declare_parameter('put_goal1', 'yet')
        # self.declare_parameter('put_goal2', 'yet')
        #pick - put 전환 파라미터
        self.declare_parameter('delivery', 'stop') #stop or go

        

'''
String value is: yet
'''
def main():
    rclpy.init()
    node = RegParams()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
