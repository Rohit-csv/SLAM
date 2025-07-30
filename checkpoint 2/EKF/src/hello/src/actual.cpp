#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class SimpleMarkerNode : public rclcpp::Node {
public:
    SimpleMarkerNode() : Node("simple_marker_node") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SimpleMarkerNode::publish_marker, this));

        RCLCPP_INFO(this->get_logger(), "📌 Simple Marker Node Started");
    }

private:
    void publish_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "simple_arrow";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Arrow position at origin
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.20;

        // Orientation (no rotation)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 1.00;
        marker.pose.orientation.w = 2.0;

        // Arrow dimensions
        marker.scale.x = 1.0;  // Length
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Color (blue)
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        // Display forever
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_pub_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
