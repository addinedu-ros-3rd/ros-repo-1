import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

from interfaces_pkg.msg import TaskRequest, AstarMsg
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist

from ament_index_python.packages import get_package_share_directory


import os
from math import sqrt, atan2, pi
from tf_transformations import euler_from_quaternion

bt_file = os.path.join(get_package_share_directory('robot_pkg'), 'behavior_tree', 'navigate_to_pose_and_pause_near_obstacle_proj.xml')


global amcl
amcl = PoseWithCovarianceStamped()

class AMCLSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_subscriber')

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, amcl_pose_qos) # 아직 어떤 로봇 불러올지 안맞춤

    def amcl_callback(self, msg):
        global amcl
        amcl = msg

class TaskSubscriber(Node):
    def __init__(self):
        super().__init__('send_task_subscriber')
        
        self.subscription = self.create_subscription(
            TaskRequest,
            'task',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        print(msg)
        

class GoPoseNode(Node):
    def __init__(self):
        super().__init__(node_name='go_pose_navigator')

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.amcl = PoseWithCovarianceStamped()
        self.navigator = BasicNavigator()

        self.vel_linear = 0.1
        self.vel_angle = 0.2

        self.done_publisher = self.create_publisher(String, '/done_task', 10)       
        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10) 
        self.astar_path_sub = self.create_subscription(AstarMsg,
                                                       'astar_paths',
                                                       self.astarCallback,
                                                       10)

    def astarCallback(self, astar_paths):
        global amcl
        # self.navigator.waitUntilNav2Active()

        print(astar_paths.length)
        
        if astar_paths.length < 1:
            print("Invalid Paths")
            msg = String()
            msg.data = "REJECT"
            self.done_publisher.publish(msg)
            return


        i = 0
        while True:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = astar_paths.poses[i].position.x
            goal_pose.pose.position.y = astar_paths.poses[i].position.y
            goal_pose.pose.orientation.z = astar_paths.poses[i].orientation.z
            goal_pose.pose.orientation.w = astar_paths.poses[i].orientation.w
            # paths.append(goal_pose)
            self.navigator.goToPose(goal_pose, behavior_tree=bt_file)


            j = 0
            waiting_cnt = 0
            while not self.navigator.isTaskComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                j = j + 1
                feedback = self.navigator.getFeedback()
                if feedback and j % 5 == 0:
                    if int(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) == 0:
                        if waiting_cnt >= 3:
                            break
                        waiting_cnt += 1
                    else:
                        waiting_cnt = 0

                    print(
                        'Estimated time of arrival: '
                        + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                            / 1e9
                        )
                        + ' seconds.'
                    )

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                        self.navigator.cancelTask()


            twist_msg = Twist()
            
            # 각도 조정
            diff_theta = self.calc_diff_theta(goal_pose.pose.position.x, goal_pose.pose.position.y)
            before_diff_theta = diff_theta

            while diff_theta > 0.01:
                diff_theta = self.calc_diff_theta(goal_pose.pose.position.x, goal_pose.pose.position.y)

                print("cmd_vel로 조정중...", end='')
                print(" 목표지점과 각도 : ", diff_theta * 180 / pi, 'degree')

                if before_diff_theta < diff_theta:
                    # continue_flag = True
                    break

                if diff_theta > (pi / 2) or diff_theta < -(pi / 2):
                    twist_msg.angular.z = -self.vel_angle
                else :
                    twist_msg.angular.z = self.vel_angle
                self.cmd_vel_pub.publish(twist_msg)


                before_diff_theta = diff_theta

            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)

            # 거리 조정
            diff_distance = self.calc_diff_distance(amcl.pose.pose.position.x, goal_pose.pose.position.x, amcl.pose.pose.position.y, goal_pose.pose.position.y)
            before_diff_distance = diff_distance

            while diff_distance > 0.01:
                diff_distance = self.calc_diff_distance(amcl.pose.pose.position.x, goal_pose.pose.position.x, amcl.pose.pose.position.y, goal_pose.pose.position.y)

                print("cmd_vel로 조정중...", end='')
                print(" 목표지점과 거리 : ", diff_distance, 'm')

                if before_diff_distance < diff_distance:
                    # continue_flag = True
                    break

                twist_msg.linear.x = self.vel_linear
                self.cmd_vel_pub.publish(twist_msg)


                before_diff_distance = diff_distance

            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)

            diff_distance = self.calc_diff_distance(amcl.pose.pose.position.x, goal_pose.pose.position.x, amcl.pose.pose.position.y, goal_pose.pose.position.y)

            # Do something depending on the return code
            result = self.navigator.getResult()
            if diff_distance <= 0.03:
                self.navigator.cancelTask()
                if i == astar_paths.length - 1:
                    print('Goal succeeded!')
                    msg = String()
                    msg.data = "OK"
                    self.done_publisher.publish(msg)
                    break
                else:
                    i = i + 1
            elif result == TaskResult.SUCCEEDED:
                if i == astar_paths.length - 1:
                    print('Goal succeeded!')
                    msg = String()
                    msg.data = "OK"
                    self.done_publisher.publish(msg)
                    break
                else:
                    i = i + 1
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
              
    @staticmethod
    def calc_diff_distance(sx, gx, sy, gy):
        return sqrt(pow(gx - sx, 2) + pow(gy - sy, 2))      

    @staticmethod
    def calc_diff_theta(gx, gy):
        global amcl

        alpha = euler_from_quaternion([0, 0, amcl.pose.pose.orientation.z, amcl.pose.pose.orientation.w])[2]
        beta = atan2((gy - amcl.pose.pose.position.y), (gx - amcl.pose.pose.position.x))

        return beta - alpha

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    send_task_subscriber = TaskSubscriber()
    executor.add_node(send_task_subscriber)
    
    # navigation 수행 노드
    go_pose_node = GoPoseNode()
    executor.add_node(go_pose_node)

    # amcl subscriber
    amcl_subscriber = AMCLSubscriber()
    executor.add_node(amcl_subscriber)
    
    executor.spin()
    
    
if __name__ == '__main__':
    main()