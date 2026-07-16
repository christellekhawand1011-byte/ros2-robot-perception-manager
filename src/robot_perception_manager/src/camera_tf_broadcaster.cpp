#include <memory>

#include "rclcpp/rclcpp.hpp"

class CameraTFBroadcaster : public rclcpp::Node{
public:
    CameraTFBroadcaster() : Node("camera_tf_broadcaster"){
        RCLCPP_INFO(this->get_logger(), "Camera TF broadcaster node has started.");
    }

private:

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}