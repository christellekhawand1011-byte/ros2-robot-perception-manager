#include <functional>
#include <memory>
#include <thread>

#include "robot_perception_interfaces/srv/set_confidence_threshold.hpp"
#include "rclcpp/rclcpp.hpp"


class PerceptionManager : public rclcpp::Node{
public:
    PerceptionManager() : Node("perception_manager"){
        this->declare_parameter<double>("confidence_threshold", 0.0);

        confidence_threshold_ = this->get_parameter
            ("confidence_threshold").as_double();

        set_confidence_service_ = this->create_service<
            robot_perception_interfaces::srv::SetConfidenceThreshold>
            ("/set_confidence",
            std::bind(&PerceptionManager::handle_request, this,
            std::placeholders::_1, std::placeholders::_2));


        RCLCPP_INFO(this->get_logger(), 
            "Perception manager node has started with threshold %.2f", 
            confidence_threshold_);
    }

private:
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
        confidence_threshold_ = static_cast<double>(request->threshold);

        // Keep the ROS 2 parameter synchronized with the new value
        this->set_parameter(rclcpp::Parameter(
            "confidence_threshold", confidence_threshold_));

        // Send a successful response to the service client
        response->success = true;
        response->message = "Confidence threshold updated";

        RCLCPP_INFO(this->get_logger(),"Confidence threshold changed to %.2f",
            confidence_threshold_);
    }

    rclcpp::Service<robot_perception_interfaces::srv::SetConfidenceThreshold>::SharedPtr 
        set_confidence_service_;
    double confidence_threshold_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}