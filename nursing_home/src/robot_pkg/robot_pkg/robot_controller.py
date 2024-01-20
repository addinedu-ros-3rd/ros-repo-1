import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

from interfaces_pkg.msg import TaskRequest, AstarMsg
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from ament_index_python.packages import get_package_share_directory
import os

bt_file = os.path.join(get_package_share_directory('robot_pkg'), 'behavior_tree', 'navigate_to_pose_and_pause_near_obstacle_proj.xml')


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

        self.done_publisher = self.create_publisher(String, '/done_task', 10)        
        self.astar_path_sub = self.create_subscription(AstarMsg,
                                                       'astar_paths',
                                                       self.astarCallback,
                                                       10)

    def astarCallback(self, astar_paths):
        # self.navigator.waitUntilNav2Active()

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

            # Do something depending on the return code
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
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
                    

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    send_task_subscriber = TaskSubscriber()
    executor.add_node(send_task_subscriber)
    
    # navigation 수행 노드
    go_pose_node = GoPoseNode()
    executor.add_node(go_pose_node)
    
    executor.spin()
    
    
if __name__ == '__main__':
    main()