#include <functional>
#include <memory>
#include <thread>
#include <algorithm>
#include <random>
#include <atomic>
#include <mutex>

#include "robot_perception_interfaces/srv/set_confidence_threshold.hpp"
#include "robot_perception_interfaces/action/start_detection.hpp"
#include "robot_perception_interfaces/msg/detection.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class PerceptionManager : public rclcpp::Node{
public:
    using StartDetection =
        robot_perception_interfaces::action::StartDetection;

    using GoalHandleStartDetection =
        rclcpp_action::ServerGoalHandle<StartDetection>;

    PerceptionManager() : Node("perception_manager"){
        this->declare_parameter<double>("confidence_threshold", 0.0);

        confidence_threshold_.store(this->get_parameter
            ("confidence_threshold").as_double());

        // Create the "/detections" publisher
        detection_publisher_ = this->create_publisher
            <robot_perception_interfaces::msg::Detection>
            ("/detections", 10);

        // Create the "/start_detection" action server
        start_detection_action_server_ = rclcpp_action::create_server
            <StartDetection>(this, "/start_detection",
            std::bind(&PerceptionManager::handle_goal, this, 
                std::placeholders::_1, std::placeholders::_2),
            std::bind(&PerceptionManager::handle_cancel, this, 
                std::placeholders::_1),
            std::bind(&PerceptionManager::handle_accepted, this, 
                std::placeholders::_1));

        // Create the internal cancellation client
        // This allows the server to place the active goal in the CANCELING state
        cancel_goal_client_ =this->create_client
            <action_msgs::srv::CancelGoal>("/start_detection/_action/cancel_goal");

        // Create the"/set_confidence" service server
        set_confidence_service_ = this->create_service<
            robot_perception_interfaces::srv::SetConfidenceThreshold>
            ("/set_confidence",
            std::bind(&PerceptionManager::handle_request, this,
            std::placeholders::_1, std::placeholders::_2));


        RCLCPP_INFO(this->get_logger(), 
            "Perception manager node has started with threshold %.2f", 
            confidence_threshold_.load());
    }

private:
    // Starts action execution in a separate thread so the ROS executor
    // remains available to receive services, cancellations, and new goals
    void start_execution(
        const std::shared_ptr<GoalHandleStartDetection> goal_handle)
    {
        std::thread{std::bind(&PerceptionManager::execute, this,
            std::placeholders::_1), goal_handle}.detach();
    }

    // Sends a cancellation request for a specific active goal
    void request_internal_cancellation(
        const std::shared_ptr<GoalHandleStartDetection> goal_handle)
    {
        auto request =
            std::make_shared<action_msgs::srv::CancelGoal::Request>();

        // Copy the active goal's unique ID so only this goal is canceled
        request->goal_info.goal_id.uuid = goal_handle->get_goal_id();

        // A zero timestamp means the request targets the specified goal ID
        request->goal_info.stamp.sec = 0;
        request->goal_info.stamp.nanosec = 0;

        // Send the request asynchronously so the node remains responsive
        cancel_goal_client_->async_send_request(request,
            [this](rclcpp::Client<action_msgs::srv::CancelGoal>::
            SharedFuture future)
        {
            const auto response = future.get();

            // A non-empty list confirms that the goal entered canceling state
            if (!response->goals_canceling.empty()){
                RCLCPP_INFO(this->get_logger(),
                "Preemption cancellation request accepted");
            }
            else{
                RCLCPP_WARN(this->get_logger(),
                    "Preemption cancellation request was not accepted");
            }
        });
    }

    // Executes the accepted detection goal
    void execute(const std::shared_ptr<GoalHandleStartDetection> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Executing detection goal");

        // Publish Detection messages on "/detections" at ~10 Hz
        rclcpp::Rate detection_rate(10.0);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<StartDetection::Feedback>();
        auto result = std::make_shared<StartDetection::Result>();

        int detections_count = 0;

        std::random_device random_device;
        std::mt19937 random_generator(random_device());
        std::uniform_real_distribution<float> coordinate_distribution
            (0.0F, 1.0F);

        while (rclcpp::ok()) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()){
                // Fill the final result for the goal being canceled
                result->total_detections = detections_count;
                result->success = false;

                // Publish a genuine CANCELED result to the action client
                goal_handle->canceled(result);

                RCLCPP_INFO(this->get_logger(),
                    "Detection goal canceled after %d detections",
                    detections_count);

                std::shared_ptr<GoalHandleStartDetection>
                next_goal;

                {
                    // Safely update the active and pending goal pointers
                    std::lock_guard<std::mutex> lock(goal_mutex_);

                    if (active_goal_ == goal_handle){
                        // The canceled goal is no longer active
                        active_goal_.reset();

                        if (pending_goal_){
                            // Promote the waiting goal to become the new active goal
                            next_goal = pending_goal_;
                            pending_goal_.reset();

                            active_goal_ = next_goal;
                        }
                    }
                }

            if (next_goal){
                RCLCPP_INFO(this->get_logger(),
                    "Starting the pending detection goal");

                // Start the replacement goal immediately after the old one
                // has published its CANCELED result
                start_execution(next_goal);
            }

            return;
        }

            robot_perception_interfaces::msg::Detection detection;

            detection.class_name = goal->target_class;

            // Recreate the distribution using the latest threshold value
            // (in case the service updates it) 
            const double current_threshold = confidence_threshold_.load();
            std::uniform_real_distribution<float> confidence_distribution
                (static_cast<float>(current_threshold), 1.0F);
            detection.confidence = confidence_distribution(random_generator);

            float x1 = coordinate_distribution(random_generator);
            float x2 = coordinate_distribution(random_generator);
            float y1 = coordinate_distribution(random_generator);
            float y2 = coordinate_distribution(random_generator);

            detection.x_min = std::min(x1, x2);
            detection.x_max = std::max(x1, x2);
            detection.y_min = std::min(y1, y2);
            detection.y_max = std::max(y1, y2);

            detection.stamp = this->now();

            detection_publisher_->publish(detection);
            // Update detection count
            detections_count++;

            // Feedback must be sent every 0.2 seconds
            // Since detections are published every 0.1 second,
            // publish feedback every second detection
            if (detections_count % 2 == 0){
                feedback->current_fps = 10.0F;
                feedback->detections_so_far = detections_count;

                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), 
                    "Feedback: %d detections so far.", detections_count);
            }

            detection_rate.sleep();
        }
    }

    // Called when a client sends a new detection goal
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const StartDetection::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request.");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Called when a client asks to cancel an active goal
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleStartDetection> gh)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)gh;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Called after ROS 2 accepts the goal
    void handle_accepted(const std::shared_ptr
        <GoalHandleStartDetection> goal_handle)
    {
        std::shared_ptr<GoalHandleStartDetection>goal_to_cancel;

        bool start_immediately = false;

        {
            // Protect goal pointers because this callback and execute()
            // may access them from different threads
            std::lock_guard<std::mutex> lock(goal_mutex_);

            // No goal is running, so this goal can start immediately
            if (!active_goal_){
                active_goal_ = goal_handle;
                start_immediately = true;
            }
            else{
                // Store the new goal until the current goal finishes canceling
                pending_goal_ = goal_handle;
                // Keep a local copy of the current goal to cancel after
                // releasing the mutex
                goal_to_cancel = active_goal_;
            }
        }

        if (start_immediately){
            RCLCPP_INFO(this->get_logger(),
                "Starting newly accepted detection goal");

            start_execution(goal_handle);
        }
        else{
            RCLCPP_INFO(this->get_logger(),
                "New goal received: canceling the current goal");

            // Request a real ROS 2 cancellation for the active goal
            request_internal_cancellation(goal_to_cancel);
        }
    }

    // Service callback
    void handle_request(
        const std::shared_ptr<robot_perception_interfaces::srv::SetConfidenceThreshold::Request> request,
        std::shared_ptr<robot_perception_interfaces::srv::SetConfidenceThreshold::Response> response)
    {
        // Reject confidence values outside the vaid range
        if (request->threshold < 0.0F || request->threshold > 1.0F){
            response->success = false;
            response->message = "Threshold must be between 0.0 and 1.0";
            return;
        }

        // Store the valid threshold in the node
        confidence_threshold_.store(static_cast<double>(request->threshold));

        // Keep the ROS 2 parameter synchronized with the new value
        this->set_parameter(rclcpp::Parameter(
            "confidence_threshold", 
            static_cast<double>(request->threshold)));

        // Send a successful response to the service client
        response->success = true;
        response->message = "Confidence threshold updated";

        RCLCPP_INFO(this->get_logger(),"Confidence threshold changed to %.2f",
            confidence_threshold_.load());
    }

    rclcpp::Publisher<robot_perception_interfaces::msg::Detection>::SharedPtr 
        detection_publisher_;

    rclcpp_action::Server<StartDetection>::SharedPtr 
        start_detection_action_server_;
    
    rclcpp::Service<robot_perception_interfaces::srv::SetConfidenceThreshold>::SharedPtr 
        set_confidence_service_;

    // Protects shared goal pointers accessed by multiple threads
    std::mutex goal_mutex_;
    // Stores the goal that is currently being executed
    std::shared_ptr<GoalHandleStartDetection> active_goal_;
    // Stores a newly accepted goal while the current goal is being canceled
    std::shared_ptr<GoalHandleStartDetection> pending_goal_;

    // Internal client used to send a real cancellation request
    // to the action server's cancel service
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr
        cancel_goal_client_;

    // The service callback and the action thread may access confidence_threshold
    // at the same time.
    std::atomic<double> confidence_threshold_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}