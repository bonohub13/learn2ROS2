#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fibonacci_action/action/fibonacci.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class FibonacciServer : public rclcpp::Node
{
public:
	explicit FibonacciServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
private:
	rclcpp_action::Server<fibonacci_action::action::Fibonacci>::SharedPtr fib_server;
	rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID &uuid, 
			std::shared_ptr<const Fibonacci::Goal> goal);
	rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<GoalHandleFibonacci> goal_handle);
};
