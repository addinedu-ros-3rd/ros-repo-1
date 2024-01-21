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
from math import sqrt

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

        self.vel_linear = 0.2

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

            diff_distance = self.calc_diff_distance(amcl.pose.pose.position.x, goal_pose.pose.position.x, amcl.pose.pose.position.y, goal_pose.pose.position.y)
            twist_msg = Twist()
            before_diff_distance = diff_distance

            while diff_distance > 0.05:
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
            if result == TaskResult.SUCCEEDED or diff_distance <= 0.01:
                if i == astar_paths.length - 1:
                    # 업무 완료
                    msg = String()
                    msg.data = "OK"
                    self.done_publisher.publish(msg)
                    print('Goal succeeded!')
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