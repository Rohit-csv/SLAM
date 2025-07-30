#include "rclcpp/rclcpp.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

class WheelSpeedToVelocity : public rclcpp::Node {
public:
    WheelSpeedToVelocity() : Node("calculation") {
        subscription_ = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
            "/ros_can/wheel_speeds", 10,
            std::bind(&WheelSpeedToVelocity::wheel_speed_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&WheelSpeedToVelocity::imuCallback, this, std::placeholders::_1));
    }

private:
    void wheel_speed_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg) {
        double lf = msg->speeds.lf_speed;
        double rf = msg->speeds.rf_speed;
        double lb = msg->speeds.lb_speed;
        double rb = msg->speeds.rb_speed;
        double steering_angle = msg->speeds.steering;

        double vx = (lf + rf + lb + rb) / 4.0;
        double vy = vx * tan(steering_angle);

        geometry_msgs::msg::TwistStamped velocity_msg;
        velocity_msg.header.stamp = msg->header.stamp;
        velocity_msg.header.frame_id = "base_link";
        velocity_msg.twist.linear.x = vx / 500.0;
        velocity_msg.twist.linear.y = vy / 500.0;
        velocity_msg.twist.angular.z = steering_angle;

        // Publish here if you have a publisher_
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std_msgs::msg::Float64 ang_vel_msg;
        ang_vel_msg.data = msg->angular_velocity.z;
        // Optional: Publish or process angular velocity
    }

    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelSpeedToVelocity>());
    rclcpp::shutdown();
    return 0;
}
