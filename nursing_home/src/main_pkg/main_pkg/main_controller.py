import rclpy

from utils.custom_logger import Logger
from module.task_planning import TaskPlanning

from rclpy.node import Node
from std_msgs.msg import String


log = Logger(__name__)


class TaskSubscriber(Node):

    def __init__(self):
        log.info("TaskSubscriber started.")
        self.task = None
        
        super().__init__('task_subscriber')
        self.subscription = self.create_subscription(
            String,
            'task_request',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.task = msg.data
        log.info("listening..." + self.task)


def main():
    rclpy.init()
    
    task_subscriber = TaskSubscriber()
    rclpy.spin_once(task_subscriber, timeout_sec=3)
    
    task_planner = TaskPlanning()
    
    # subscribe한 task가 있으면 task_planner에 넘겨주고, 없으면 파라미터 없이 실행
    if task_subscriber.task != None:
        task_planner.main(task_subscriber.task)
    else:
        task_planner.main()


if __name__ == '__main__':
    main()