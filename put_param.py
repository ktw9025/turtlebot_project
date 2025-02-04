import rclpy, sys, os
from rclpy.node import Node


class Put_param(Node):

    def __init__(self):
        super().__init__('put_param')
        self.call_param    = self.create_timer(1, self.check_param)
        self.param = ''
        self.param2 = ''

    def check_param(self):
        temp = os.popen("ros2 param get /reg_params put_place").read().strip()
        temp_param = temp[17:].strip()  # 필요한 부분을 잘라내고 공백 제거
        self.param = temp_param
        temp2 = os.popen("ros2 param get /reg_params delivery").read().strip()
        temp_param2 = temp2[17:].strip()  # 필요한 부분을 잘라내고 공백 제거
        self.param2 = temp_param2


def main(args=None):
    rclpy.init(args=args)

    node = Put_param()
    try:
        while rclpy.ok:
            rclpy.spin_once(node, timeout_sec= 1)
            param = node.param  # param 값이 업데이트된 후 읽기
            param2 = node.param2  # param 값이 업데이트된 후 읽기
            print(f"Updated param: {param}")  # 현재 param 값 출력
            print(f"Updated param: {param2}")  # 현재 param 값 출력
            if param2 == 'go':
                if param == 'place0':
                    os.system("ros2 run project_3 put_nav0 ")
                    os.system("ros2 param set /reg_params put_place place3 ")
                    os.system("ros2 param set /reg_params delivery stop ")
                    

                if param == 'place1':
                    os.system("ros2 run project_3 put_nav1 ")
                    os.system("ros2 param set /reg_params put_place place3 ")
                    os.system("ros2 param set /reg_params delivery stop ")
                    

                if param == 'place2':
                    os.system("ros2 run project_3 put_nav2 ")
                    os.system("ros2 param set /reg_params put_place place3 ")
                    os.system("ros2 param set /reg_params delivery stop ")
                    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
