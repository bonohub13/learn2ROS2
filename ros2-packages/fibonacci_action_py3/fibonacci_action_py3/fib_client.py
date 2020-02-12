import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from fibonacci_action.action import Fibonacci

class FibonacciClient(Node):
    def __init__(self):
        super().__init__('fib_client')
        self.fib_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order): #sending goal to server
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.fib_client.wait_for_server()
        self.send_goal_future = self.fib_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedbackCB)
        self.send_goal_future.add_done_callback(self.goal_responseCB)

    def goal_responseCB(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected...')
            return

        self.get_logger().info('Goal accepted!')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_resultCB)

    def get_resultCB(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedbackCB(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Recieved feedback: {0}'.format(feedback.sequence))

def main(args=None):
    rclpy.init(args=args)
    
    fib_client = FibonacciClient()
    fib_client.send_goal(10)
    
    rclpy.spin(fib_client)

if __name__ == '__main__':
    main()
