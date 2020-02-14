#include <inttypes.h>
#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fibonacci_action/action/fibonacci.hpp"

using namespace std::placeholders;

class FibonacciClient : public rclcpp::Node
{
public:
	using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<fibonacci_action::action::Fibonacci>;

public:
	explicit FibonacciClient(const rclcpp::NodeOptions &options);
	bool is_goal_done() const;
	void send_goal();

private:
	rclcpp_action::Client<fibonacci_action::action::Fibonacci>::SharedPtr action_client;
	rclcpp::TimerBase::SharedPtr timer;

private:
	bool goal_done;
	void goal_responseCB(std::shared_future<GoalHandleFibonacci::SharedPtr> future);
	void feedbackCB(
			GoalHandleFibonacci::SharedPtr,
			const std::shared_ptr<const fibonacci_action::action::Fibonacci::Feedback> feedback);
	void resultCB(const GoalHandleFibonacci::WrappedResult &result);
};
FibonacciClient::FibonacciClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) :
	Node("fib_client", options), goal_done(false)
{
	this->action_client = rclcpp_action::create_client<fibonacci_action::action::Fibonacci>(
			this->get_node_base_interface(),
			this->get_node_graph_interface(),
			this->get_node_logging_interface(),
			this->get_node_waitables_interface(),
			"fibonacci");
	this->timer = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&FibonacciClient::send_goal, this));
}
bool FibonacciClient::is_goal_done() const
{
	return this->goal_done;
}
void FibonacciClient::send_goal()
{
	this->timer->cancel();
	this->goal_done = false;
	
	if (!this->action_client)
		RCLCPP_ERROR(this->get_logger(), "Action client not initialized!");
	if (!this->action_client->wait_for_action_server(std::chrono::milliseconds(100)))
	{
		RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting...");
		this->goal_done = true;
		return;
	}

	auto goal_msg = fibonacci_action::action::Fibonacci::Goal();
	goal_msg.order = 10;

	RCLCPP_INFO(this->get_logger(), "Sending goal...");

	auto send_goal_options = rclcpp_action::Client<fibonacci_action::action::Fibonacci>::SendGoalOptions();
	send_goal_options.goal_response_callback = std::bind(&FibonacciClient::goal_responseCB, this, _1);
	send_goal_options.feedback_callback= std::bind(&FibonacciClient::feedbackCB, this, _1, _2);
	send_goal_options.result_callback = std::bind(&FibonacciClient::resultCB, this, _1);
	auto goal_handle_future = this->action_client->async_send_goal(goal_msg, send_goal_options);
}
void FibonacciClient::goal_responseCB(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
{
	auto goal_handle = future.get();
	if (!goal_handle)
		RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
	else
		RCLCPP_INFO(this->get_logger(), "Goal accepted by server\n  waiting for results...");
}
void FibonacciClient::feedbackCB(
		GoalHandleFibonacci::SharedPtr,
		const std::shared_ptr<const fibonacci_action::action::Fibonacci::Feedback> feedback)
{
	RCLCPP_INFO(this->get_logger(), "Next number in sequence recieved: %" PRId64,
			feedback->sequence.back());
}
void FibonacciClient::resultCB(const GoalHandleFibonacci::WrappedResult &result)
{
	this->goal_done = true;
	
	switch (result.code)
	{
		case rclcpp_action::ResultCode::SUCCEEDED:
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
			return;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(this->get_logger(), "Goal was canceled...");
			return;
		default:
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			return;
	}

	RCLCPP_INFO(this->get_logger(), "Result recieved!");
	for (auto number : result.result->sequence)
		RCLCPP_INFO(this->get_logger(), "result: %" PRId64, number);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto fib_client = std::make_shared<FibonacciClient>();

	while (!fib_client->is_goal_done())
	{
		rclcpp::spin_some(fib_client);
	}

	rclcpp::shutdown();

	return 0;
}
