import rclpy, sys, os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi
from tf_transformations import euler_from_quaternion # 쿼터니언 -> 오일러 각 변환
from ar_track.move_tb3 import MoveTB3 # TurtleBot3 이동을 제어하는 사용자 정의 클래스

# 타겟 마커 ID를 명령줄 인수로 전달받음
TARGET_ID = int(sys.argv[1])# argv[1]: 목표 마커 ID

# TurtleBot3의 최대 속도 설정
MAX_LIN_SPEED = 0.22  # 최대 선형 속도 (m/s)
MAX_ANG_SPEED = 2.84  # 최대 각속도 (rad/s)

# 기본 이동 속도 (최대 속도의 7.5%)
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

# 90도 (라디안 값)
R = pi / 2 # radians(90) # 1.5708



# 허용 오차 범위
# 마커의 x 좌표(pose.position.x)가 -0.0025에서 0.0025 사이에 있으면 목표 위치에 정확히 정렬되었다고 판단
# 이는 로봇이 마커에 대해 얼마나 정밀하게 정렬되어야 하는지를 결정하는 허용 오차 값
X_ALIGNMENT_TOLERANCE = 0.0025

# 속도 설정 (각속도)
# ANG_SPEED의 비율로 설정된 각속도 값
# ANG_SPEED의 7.5% 정도로 천천히 회전하여 목표 위치를 정밀하게 조정하려는 의도
# 속도를 낮추면 정밀도가 높아지고, 오차가 줄어듦
FINE_ANGULAR_SPEED = 0.075 * ANG_SPEED  

# 리프팅 기준 거리
# 로봇이 마커에 충분히 가까이 접근해야 리프팅을 할 수 있음. 이 값은 리프팅을 위한 목표 거리. 마커와 로봇 사이의 최소 거리를 정의
LIFTING_DISTANCE = 0.38 # 0.185


# 회전 및 거리 보정 비율

# angle(회전 각도)에 대해 90%만 회전하도록 설정
# 이유: 로봇이 회전을 할 때, 실제 환경에서 관성이나 바퀴 슬립 등의 물리적 요인으로 인해 초과 회전이 발생할 수 있어서 이를 방지하기 위해 약간 부족하게 회전하도록 보정한 값
FIRST_ROTATION_SCALING_POSITIVE = 0.9 # 0.9

# 음수 방향으로 회전할 때 97%만 회전하도록 설정
# 이유: 양수 방향과 음수 방향의 회전 특성이 다를 수 있으므로, 음수 방향의 초과 회전을 줄이기 위해 약간 다른 값을 사용한 것
FIRST_ROTATION_SCALING_NEGATIVE = 1 # 0.97

# 2차 회전 시 목표 각도 R의 87.5%만 회전하도록 설정
# 이유: 첫 번째 회전 후 정렬이 대체로 이루어진 상태에서, 마무리 회전을 더 정밀하게 수행하기 위한 값. 이 과정에서 지나친 회전을 방지하기 위해 보정된 값
SECOND_ROTATION_SCALING = 1 # 0.875

# 거리 계산에 대해 112.5%를 곱해 직진 거리를 보정
# 이유: 로봇이 직진할 때, 센서 데이터나 마커 위치 정보가 완벽히 정확하지 않을 수 있음. 이를 보완하기 위해 조금 더 이동하도록 설정한 값
STRAIGHT_DISTANCE_SCALING = 1.5 # 1 #1.125


class TrackMarker(Node):
    """   
                                                    ////////////| ar_marker |////////////
            y                      z                --------+---------+---------+--------
            ^  x                   ^                        |     R-0/|\R-0    R|
            | /                    |                        |       /0|0\       |
     marker |/                     | robot                  |      /  |  \      |
            +------> z    x <------+                        |     /   |   \     |
                                  /                         |  dist   |  dist   |
                                 /                          |   /     |     \   |
                                y                           |  /      |      \  |
                                                            | /       |       \0|
                                                            |/R-0    R|R    R-0\|
    pose.x = position.z                             (0 < O) x---------+---------x (0 > O)
    pose.y = position.x              [0]roll    (pos.x > O) ^                   ^ (pos.x < O)
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose  = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
            
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_lift = self.create_publisher(String, '/grip_msg', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)
        
        self.pose = Pose()
        self.tw   = Twist()
        self.tb3  = MoveTB3()
        self.lift_msg = String()
        
        self.theta   = 0.0
        self.dir     = 0
        self.th_ref  = 0.0
        self.z_ref   = 0.0
        self.cnt_sec = 0
        
        self.target_found = False
        
        
    def get_marker_pose_(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        if len(msg.marker_ids) == 0:    # no marker found
            self.target_found = False
        
        else: # if len(msg.marker_ids) != 0: # marker found at least 1EA
        
            for i in range(len(msg.marker_ids)):
            
                if msg.marker_ids[i] == TARGET_ID:  # target marker found
                    if self.target_found == False:
                        self.target_found = True                        
                    self.pose  = msg.poses[i]
                    self.theta = self.get_theta(self.pose)
                else:
                    self.target_found = False            
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta
    
    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1    
        
    def pub_lift_msg(self, lift_msg):
        msg = String()
        msg.data = lift_msg
        self.pub_lift.publish(msg)
    
    def stop_move(self):
        self.tw.linear.z = self.tw.angular.z = 0.0
        self.pub_tw.publish(self.tw)  
        
def main(args=None):

    rclpy.init(args=args)
    node = TrackMarker()
    
    node.tw.angular.z = (0.5 * ANG_SPEED)

    dist_ref = 0.10  # 15 cm
    margin = 0.08 # 0.025   # 2.5 cm
    
    try:    
        while rclpy.ok():
            if node.theta != 0.0:   break   # this means target marker found
            node.pub_tw.publish(node.tw)
            rclpy.spin_once(node, timeout_sec=0.1)
        
        node.stop_move()
        print("\n----- 1_target marker found!\n") ###########################
        
        # -0.0175 ~ 0.0175 허용편차
        while node.pose.position.x < -0.0175 or node.pose.position.x >  0.0175:
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.pose.position.x < -0.0155:
                # -0.0155 보다 작으면 시계 반대 방향으로 아래와 같은 속도로 회전
                node.tw.angular.z =  0.175 * ANG_SPEED
            else:# node.pose.position.x >  0.025:
                # 아니면 시계방향으로 아래와 같은 속도로 회전
                node.tw.angular.z = -0.125 * ANG_SPEED
            node.pub_tw.publish(node.tw)
            rclpy.spin_once(node, timeout_sec=0.1)

        
        node.stop_move()        
        print(node.pose.position.x)#~
        print("\n----- 2_arrived reference position!\n") ####################
        
        node.th_ref = node.theta
        node.z_ref = node.pose.position.z

        # 방향 설정
        if node.th_ref >= 0:
            node.dir = 1  # 오른쪽
        else:
            node.dir = -1  # 왼쪽

        # 각도 계산
        if node.dir == 1:  # 오른쪽
            angle = R - node.th_ref  # 목표 각도 계산
        else:  # 왼쪽
            angle = abs(R - abs(node.th_ref))  # 항상 절대값으로 계산
            if node.dir < 0:  # 왼쪽
                angle *= -1  # 음수로 설정

        # 회전 로직
        if node.th_ref > radians(10):  # 오른쪽에 있고 큰 각도
            node.tb3.rotate(angle * FIRST_ROTATION_SCALING_POSITIVE)
        elif node.th_ref < radians(-10):  # 왼쪽에 있고 큰 각도
            node.tb3.rotate(angle * FIRST_ROTATION_SCALING_NEGATIVE)
        else:  # 작은 각도에서는 회전하지 않음
            pass


        print("angle ~ ")
        print(angle)
        print("\n----- 3_1st rotation finished!\n") #########################
        
        dist1 = abs(node.z_ref * sin(node.th_ref) * STRAIGHT_DISTANCE_SCALING)
        print("dist1 ~ ")#~
        print(dist1)#~
        print("node.z_ref ~ ")#~
        print(node.z_ref)#~
        node.tb3.straight(dist1)
        print("\n----- 4_move to front of marker end!\n") ###################
        
        if   node.th_ref >  radians(10):
            node.tb3.rotate(-R * SECOND_ROTATION_SCALING)
        elif node.th_ref < -radians(10):
            node.tb3.rotate( R * SECOND_ROTATION_SCALING)
        else:
            pass        
        print("\n----- 5_2nd rotation finished!\n") #########################
        
        while node.pose.position.x < -X_ALIGNMENT_TOLERANCE or node.pose.position.x >  X_ALIGNMENT_TOLERANCE:
            if   node.pose.position.x < -X_ALIGNMENT_TOLERANCE:
                node.tw.angular.z =  FINE_ANGULAR_SPEED
            elif node.pose.position.x >  X_ALIGNMENT_TOLERANCE:
                node.tw.angular.z = -FINE_ANGULAR_SPEED
            else:
                node.tw.angular.z =  0.0
                
            node.pub_tw.publish(node.tw)                
            rclpy.spin_once(node, timeout_sec=0.02)
            
        

        print("가능?1")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            print("찾으면서")
            while node.pose.position.x < -X_ALIGNMENT_TOLERANCE or node.pose.position.x >  X_ALIGNMENT_TOLERANCE:
                if   node.pose.position.x < -X_ALIGNMENT_TOLERANCE:
                    node.tw.angular.z =  FINE_ANGULAR_SPEED
                elif node.pose.position.x >  X_ALIGNMENT_TOLERANCE:
                    node.tw.angular.z = -FINE_ANGULAR_SPEED
                else:
                    node.tw.angular.z =  0.0
                    
                node.pub_tw.publish(node.tw)                
                rclpy.spin_once(node, timeout_sec=0.02)

            if node.pose.position.z > dist_ref + margin:
                node.tw.linear.x = 0.01
                print("Moving forward")
            elif node.pose.position.z < dist_ref - margin:
                node.tw.linear.x = -0.01
                print("Moving backward")
            else:
                node.tw.linear.x = 0.0
                print("Stopping")
                node.stop_move()
                break
            node.pub_tw.publish(node.tw) 
        print("가능?2")

        
        print("\n----- 6_arrived lifting position!\n") ####################
        
        node.pub_lift_msg("grip_on")
        duration = node.cnt_sec + 5
        
        while node.cnt_sec < duration: 
            print(duration - node.cnt_sec)               
            rclpy.spin_once(node, timeout_sec=1.0)
        print("\n----- 7_finished loading!\n") ############################     
        
        ds = 0.2
        node.tb3.straight(-ds)

        
        print("\n----- 8 moving other point!\n") ######################

        
        sys.exit(1)
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()
