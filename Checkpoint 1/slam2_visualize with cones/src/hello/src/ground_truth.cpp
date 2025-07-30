#include "rclcpp/rclcpp.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PoseVisualizer : public rclcpp::Node {
public:
    PoseVisualizer() : Node("ground_truth") {
        sub_ = this->create_subscription<eufs_msgs::msg::CarState>(
            "/ground_truth/state", 10,
            std::bind(&PoseVisualizer::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker", 10);
    }

private:
    void callback(const eufs_msgs::msg::CarState::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Or "odom", depending on your TF setup
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pose_arrow";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position and orientation
        marker.pose.position = msg->pose.pose.position;
        marker.pose.orientation = msg->pose.pose.orientation;

        // Arrow dimensions
        marker.scale.x = 1.0;  // Length
        marker.scale.y = 0.1;  // Shaft diameter
        marker.scale.z = 0.1;  // Head diameter

        // Color: Green
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;


        pub_->publish(marker);
    }

    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseVisualizer>());
    rclcpp::shutdown();
    return 0;
}
