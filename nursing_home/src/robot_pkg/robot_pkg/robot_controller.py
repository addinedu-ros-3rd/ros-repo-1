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


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    send_task_subscriber = SendTaskSubscriber()
    executor.add_node(send_task_subscriber)
    
    done_task_publisher = DoneTaskPublisher()
    executor.add_node(done_task_publisher)
    
    executor.spin()
    
    
if __name__ == '__main__':
    main()