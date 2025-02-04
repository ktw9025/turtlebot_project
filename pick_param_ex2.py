import rclpy, sys, os
from rclpy.node import Node


class Check_param(Node):

    def __init__(self):
        super().__init__('check_param')
        self.call_param    = self.create_timer(1, self.check_param)
        self.param1 = ''
        self.param2 = ''
        self.param3 = ''
        self.param4 = ''

    def check_param(self):
        temp1 = os.popen("ros2 param get /reg_params pick_goal0").read().strip()
        temp_param1 = temp1[17:].strip()  # 필요한 부분을 잘라내고 공백 제거
        self.param1 = temp_param1
        temp2 = os.popen("ros2 param get /reg_params pick_goal1").read().strip()
        temp_param2 = temp2[17:].strip()  # 필요한 부분을 잘라내고 공백 제거
        self.param2 = temp_param2
        temp3 = os.popen("ros2 param get /reg_params pick_goal2").read().strip()
        temp_param3 = temp3[17:].strip()  # 필요한 부분을 잘라내고 공백 제거
        self.param3 = temp_param3
        temp4 = os.popen("ros2 param get /reg_params delivery").read().strip()
        temp_param4 = temp4[17:].strip()  # 필요한 부분을 잘라내고 공백 제거
        self.param4 = temp_param4


def main(args=None):
    rclpy.init(args=args)

    node = Check_param()
    try:
        while rclpy.ok:
            rclpy.spin_once(node, timeout_sec= 1)
            param1 = node.param1  # param 값이 업데이트된 후 읽기
            param2 = node.param2  # param 값이 업데이트된 후 읽기
            param3 = node.param3  # param 값이 업데이트된 후 읽기
            param4 = node.param4  # param 값이 업데이트된 후 읽기
            print(f"Updated param: {param1}")  # 현재 param 값 출력
            print(f"Updated param: {param2}")  # 현재 param 값 출력
            print(f"Updated param: {param3}")  # 현재 param 값 출력
            print(f"Updated param: {param4}")  # 현재 param 값 출력
            if param4 == 'stop':

                if param1 == 'already':
                    os.system("ros2 run project_3 pick_track_marker 0 ")
                    os.system("ros2 param set /reg_params pick_goal0 yet ")
                    os.system("ros2 param set /reg_params delivery go ")
                   

                if param2 == 'already':
                    os.system("ros2 run project_3 pick_track_marker 1 ")
                    os.system("ros2 param set /reg_params pick_goal0 yet ")
                    os.system("ros2 param set /reg_params delivery go ")
                   

                if param3 == 'already':
                    os.system("ros2 run project_3 pick_track_marker 2 ")
                    os.system("ros2 param set /reg_params pick_goal0 yet ")
                    os.system("ros2 param set /reg_params delivery go ")
                   
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
