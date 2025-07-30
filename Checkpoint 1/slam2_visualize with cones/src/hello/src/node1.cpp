#include "rclcpp/rclcpp.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <cmath>  // For M_PI

class VelocityEstimator : public rclcpp::Node
{
public:
    VelocityEstimator()
        : Node("velocity_estimator")
    {
        wheel_speed_sub_ = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
            "/ros_can/wheel_speeds", 10,
            std::bind(&VelocityEstimator::callback, this, std::placeholders::_1));

        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/computed_velocity", 10);
    }

private:
    void callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
    {
        // Constants
        const double r_wheel = 0.25;  // meters
        const double i_gear = 1.0;   // gear ratio

        // Read wheel speeds in RPM
        double rb_rpm = msg->speeds.rb_speed;
        double lb_rpm = msg->speeds.lb_speed;
        double avg_rpm = (rb_rpm + lb_rpm) / 2.0;

        // Convert to m/s
        double rps = avg_rpm / 60.0;  // revolutions per second
        double vx = (2.0 * M_PI * r_wheel * rps) / i_gear;

        // Steering and lateral velocity
        double steering_rad = msg->speeds.steering;
        double steering_deg = steering_rad * (180.0 / M_PI);
        double vy = vx * std::tan(steering_rad);

        // Prepare message
        auto vel_msg = geometry_msgs::msg::TwistStamped();
        vel_msg.header.stamp = this->now();
        vel_msg.header.frame_id = "base_link";
        vel_msg.twist.linear.x = vx;
        vel_msg.twist.linear.y = vy;

        velocity_pub_->publish(vel_msg);

        RCLCPP_INFO(this->get_logger(), "vx: %.2f m/s, vy: %.2f m/s, steering: %.4f rad (%.2f deg)",
                    vx, vy, steering_rad, steering_deg);
    }

    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_speed_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityEstimator>());
    rclcpp::shutdown();
    return 0;
}
