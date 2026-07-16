#include <memory>

#include "rclcpp/rclcpp.hpp"

class PerceptionManager : public rclcpp::Node{
public:
    PerceptionManager() : Node("perception_manager"){
        RCLCPP_INFO(this->get_logger(), "Perception manager node has started.");
    }

private:

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}