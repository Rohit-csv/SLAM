#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class YawExtractor : public rclcpp::Node
{
public:
    YawExtractor()
    : Node("yaw_extractor_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&YawExtractor::imuCallback, this, std::placeholders::_1));

        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/imu/yaw_deg", 10);
        yaw_rate_pub_ = this->create_publisher<std_msgs::msg::Float64>("/imu/yaw_rate_deg", 10);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // --- Extract yaw from quaternion ---
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        double roll, pitch, yaw_rad;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw_rad);
        double yaw_deg = yaw_rad * 180.0 / M_PI;

        // Normalize to [0, 360)
        if (yaw_deg < 0) yaw_deg += 360.0;

        // --- Extract yaw rate from angular velocity ---
        double yaw_rate_rad = msg->angular_velocity.z;
        double yaw_rate_deg = yaw_rate_rad * 180.0 / M_PI;

        // --- Log the values ---
        RCLCPP_INFO(this->get_logger(),
            "Yaw: %.2f° | Yaw Rate: %.2f°/s", yaw_deg, yaw_rate_deg);

        // --- Publish yaw ---
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = yaw_deg;
        yaw_pub_->publish(yaw_msg);

        // --- Publish yaw rate ---
        std_msgs::msg::Float64 yaw_rate_msg;
        yaw_rate_msg.data = yaw_rate_deg;
        yaw_rate_pub_->publish(yaw_rate_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawExtractor>());
    rclcpp::shutdown();
    return 0;
}
