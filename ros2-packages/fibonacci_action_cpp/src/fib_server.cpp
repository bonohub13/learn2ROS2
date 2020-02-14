#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fibonacci_action/action/fibonacci.hpp"

using namespace std::placeholders;

class FibonacciServer : public rclcpp::Node
{
public:
	using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<fibonacci_action::action::Fibonacci>;

public:
	explicit FibonacciServer(const rclcpp::NodeOptions &options);

private:
	rclcpp_action::Server<fibonacci_action::action::Fibonacci>::SharedPtr action_server;
	// ???
	rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID &uuid,
			std::shared_ptr<const fibonacci_action::action::Fibonacci::Goal> goal);
	rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<GoalHandleFibonacci> goal_handle);

private:
	void executeCB(
			const std::shared_ptr<GoalHandleFibonacci> goal_handle);
	void handle_accepted(
			const std::shared_ptr<GoalHandleFibonacci> goal_handle);
};
FibonacciServer::FibonacciServer(
		const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("fib_server", options)
{
	//why does "action_server" need "this->"?
	this->action_server = rclcpp_action::create_server<fibonacci_action::action::Fibonacci>(
			/*::::: self :::::*/
			this->get_node_base_interface(),
			this->get_node_clock_interface(),
			this->get_node_logging_interface(),
			this->get_node_waitables_interface(),
			/*::::::::::::::::*/
			"fibonacci",
			/*::::: self.executeCB :::::*/
			std::bind(&FibonacciServer::handle_goal, this, _1, _2),
			std::bind(&FibonacciServer::handle_cancel, this, _1),
			std::bind(&FibonacciServer::handle_accepted, this, _1));
			/*::::::::::::::::::::::::::*/
}
rclcpp_action::GoalResponse FibonacciServer::handle_goal(
		const rclcpp_action::GoalUUID &uuid,
		std::shared_ptr<const fibonacci_action::action::Fibonacci::Goal> goal)
{
	RCLCPP_INFO(this->get_logger(), "Recieved goal request\n  order: %d", goal->order);
	(void)uuid;
	if (goal->order > 9000) //reject goals with orders over 9000
	{
		return rclcpp_action::GoalResponse::REJECT;
	}
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FibonacciServer::handle_cancel(const std::shared_ptr<FibonacciServer::GoalHandleFibonacci> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal...");
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}
void FibonacciServer::executeCB(std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing goal...");
	rclcpp::Rate rate(1);
	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<fibonacci_action::action::Fibonacci::Feedback>();
	auto &sequence = feedback->sequence;
	sequence.push_back(0);
	sequence.push_back(1);
	auto result = std::make_shared<fibonacci_action::action::Fibonacci::Result>();

	for (int i=1; (i < goal->order) && rclcpp::ok(); ++i)
	{
		if (goal_handle->is_canceling()) //if goal_handle quits buffering goals
		{
			result->sequence = sequence;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Goal canceled!");
				return;
		}
		
		sequence.push_back(sequence[i] + sequence[i-1]);
		goal_handle->publish_feedback(feedback);
		RCLCPP_INFO(this->get_logger(), "Publishing feedback...");
		rate.sleep();
	}

	if (rclcpp::ok()) //run if the result reached the goal
	{
		result->sequence = sequence;
		goal_handle->succeed(result);
		RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
	}
}
void FibonacciServer::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) //to keep the Excuter running while the executeCB method is running
{
	std::thread(std::bind(&FibonacciServer::executeCB, this, _1),
			goal_handle).detach();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto fib_server = std::make_shared<FibonacciServer>();

	rclcpp::spin(fib_server);
	rclcpp::shutdown();

	return 0;
}
