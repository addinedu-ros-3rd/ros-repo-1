import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from interfaces_pkg.msg import Task
from std_msgs.msg import String


class SendTaskSubscriber(Node):
    def __init__(self):
        super().__init__('send_task_subscriber')
        
        self.subscription = self.create_subscription(
            Task,
            'send_task',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        print("listening..." + msg)
        # todo: basic navigator로 현재 로봇 실행


class DoneTaskPublisher(Node):
    def __init__(self):
        super().__init__('done_task_publisher')
        self.publisher = self.create_publisher(String, '/done_task', 10)
        msg = String()
        msg.data = "OK"
        self.publisher.publish(msg)
        

class GoPoseNode(Node):
    def __init__(self):
        super().__init__(node_name='go_pose_navigator')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.amcl = PoseWithCovarianceStamped()
        self.navigator = BasicNavigator()
        
        self.astar_path_sub = self.create_subscription(AstarMsg,
                                                       'astar_paths',
                                                       self.astarCallback,
                                                       10)

    def astarCallback(self, astar_paths):
        self.navigator.waitUntilNav2Active()

        
        paths = []
        for i in range(astar_paths.length):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = astar_paths.positions[i].x
            goal_pose.pose.position.y = astar_paths.positions[i].y
            # 
            # goal_pose.pose.orientation.z = 0.10424110491274381
            # goal_pose.pose.orientation.w = 0.9945520559762422
            paths.append(goal_pose)



        self.navigator.goThroughPoses(paths)


        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    send_task_subscriber = SendTaskSubscriber()
    executor.add_node(send_task_subscriber)
    
    # navigation 수행 노드
    go_pose_node = GoPoseNode()
    executor.add_node(go_pose_node)

    done_task_publisher = DoneTaskPublisher()
    executor.add_node(done_task_publisher)
    
    executor.spin()
    
    
if __name__ == '__main__':
    main()