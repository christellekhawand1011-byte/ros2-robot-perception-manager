#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include "rclcpp/rclcpp.hpp"

class CameraTFBroadcaster : public rclcpp::Node{
public:
    CameraTFBroadcaster() : Node("camera_tf_broadcaster"){
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->declare_parameter<double>("tx", 0.0);
        this->declare_parameter<double>("ty", 0.0);
        this->declare_parameter<double>("tz", 0.0);

        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("yaw", 0.0);

        double tx = this->get_parameter("tx").as_double();
        double ty = this->get_parameter("ty").as_double();
        double tz = this->get_parameter("tz").as_double();

        double roll = this->get_parameter("roll").as_double();
        double pitch = this->get_parameter("pitch").as_double();
        double yaw = this->get_parameter("yaw").as_double();

        // Publish static transforms once at startup
        make_transforms(tx, ty, tz, roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Camera TF broadcaster node has started.");
    }

private:
    void make_transforms(double tx, double ty, double tz,
                        double roll, double pitch, double yaw){
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "camera_optical_frame";

    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.translation.z = tz;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}