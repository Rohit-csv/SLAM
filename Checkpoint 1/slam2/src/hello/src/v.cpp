#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class VelocityListener : public rclcpp::Node {
public:
    VelocityListener() : Node("velocity_listener_node") {
        // Subscriber: Yaw (deg)
        yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/imu/yaw_deg", 10,
            std::bind(&VelocityListener::yawCallback, this, std::placeholders::_1));

        // Subscriber: Yaw Rate (deg/s)
        yaw_rate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/imu/yaw_rate_deg", 10,
            std::bind(&VelocityListener::yawRateCallback, this, std::placeholders::_1));

        // Subscriber: Linear velocities vx, vy
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/computed_velocity", 10,
            std::bind(&VelocityListener::velCallback, this, std::placeholders::_1));

        // Publisher: Synced velocity
        synced_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/synced_velocity", 10);
    }

private:
    void yawCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        yaw_ = msg->data;
        publishSyncedData();
    }

    void yawRateCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        yaw_rate_ = msg->data;
        publishSyncedData();
    }

    void velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        vx_ = msg->twist.linear.x;
        vy_ = msg->twist.linear.y;
        last_vel_stamp_ = msg->header.stamp;
        publishSyncedData();
    }

    void publishSyncedData() {
        // Check if we have valid timestamp
        if (last_vel_stamp_.nanoseconds() == 0) return;

        RCLCPP_INFO(this->get_logger(),
            "Synced Data -> Yaw: %.2f° | YawRate: %.2f°/s | vx: %.3f, vy: %.3f",
            yaw_, yaw_rate_, vx_, vy_);

        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = last_vel_stamp_;
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = vx_;
        msg.twist.linear.y = vy_;
        msg.twist.angular.z = yaw_;        // Use for yaw
        msg.twist.angular.x = yaw_rate_;   // Reuse x for yaw rate (document this clearly!)

        synced_pub_->publish(msg);
    }

    double yaw_ = 0.0;
    double yaw_rate_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    rclcpp::Time last_vel_stamp_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_rate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr synced_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityListener>());
    rclcpp::shutdown();
    return 0;
}
