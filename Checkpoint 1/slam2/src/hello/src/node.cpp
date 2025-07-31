#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class ArrowMarkerSubscriber : public rclcpp::Node {
public:
    ArrowMarkerSubscriber() : Node("arrow_marker_subscriber") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10);

        velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/synced_velocity", 10,
            std::bind(&ArrowMarkerSubscriber::velocity_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArrowMarkerSubscriber::publish_marker, this));

        RCLCPP_INFO(this->get_logger(), "Arrow Marker Subscriber Node Started.");
    }

private:
    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        vx_ = msg->twist.linear.x;
        vy_ = msg->twist.linear.y;  // not directly used in motion model
        yaw_ = msg->twist.angular.z * M_PI / 180.0;  // converting from deg to rad
        yaw_rate_ = msg->twist.angular.x * M_PI / 180.0;  // deg/s to rad/s
    }

    void publish_marker() {
        double dt = 0.05; // time step in seconds (same as timer period)

        // Motion model:
        // vx is forward speed in local frame
        // yaw is orientation angle
        // yaw_rate is angular velocity

        double delta_x = (std::cos(yaw_) * vx_ - std::sin(yaw_) * vy_) * dt;
        double delta_y = (std::sin(yaw_) * vx_ + std::cos(yaw_) * vy_) * dt;

        double delta_yaw = yaw_rate_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        yaw_ += delta_yaw;

        // Normalize yaw to [-π, π]
        while (yaw_ > M_PI) yaw_ -= 2 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2 * M_PI;

   
       // Create a marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pose_arrow";
        marker.id = 10;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x_;
        marker.pose.position.y = y_;
        marker.pose.position.z = 0.5;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(),
            "Marker Pose -> x: %.2f, y: %.2f, yaw: %.2f°", x_, y_, yaw_ * 180.0 / M_PI);
    }

    double vx_ = 0.0;
    double vy_ = 0.0;
    double yaw_rate_ = 0.0;
    double yaw_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArrowMarkerSubscriber>());
    rclcpp::shutdown();
    return 0;
}
