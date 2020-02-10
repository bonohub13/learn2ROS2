from time import sleep

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from fibonacci_action.action import Fibonacci

class FibonacciServer(Node):
    def __init__(self):
        super().__init__('fib_server')
        self.action_server = ActionServer(
                self, 
                Fibonacci,
                'fibonacci',
                self.executeCB)

    def executeCB(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            sleep(1)
        
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        return result

def main(args=None):
    rclpy.init(args=args)
    fib_server = FibonacciServer()
    try:
        rclpy.spin(fib_server)
    except KeyboardInterrupt:
        print('quitting process...')

if __name__ == '__main__':
    main()
