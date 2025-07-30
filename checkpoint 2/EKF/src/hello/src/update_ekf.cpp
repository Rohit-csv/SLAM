// File: src/update_ekf.cpp

#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

// Calculate the Jacobian F_k and its transpose F_k^T for the motion model
void calculateJacobianAndTranspose(
    double theta, double v, double dt,
    Eigen::Matrix3d &F_k, Eigen::Matrix3d &F_k_T)
{
    F_k.setIdentity();
    F_k(0, 2) = -dt * v * std::sin(theta);
    F_k(1, 2) = dt * v * std::cos(theta);

    F_k_T = F_k.transpose();
}

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class EKFLocalizer : public rclcpp::Node
{
public:
    EKFLocalizer()
        : Node("update_ekf")
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/robot_pose_2d", 10,
            std::bind(&EKFLocalizer::poseCallback, this, std::placeholders::_1));

        cones_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "pose_cones_array", 10,
            std::bind(&EKFLocalizer::conesArrayCallback, this, std::placeholders::_1));

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "ekf_marker_final/marker", 10);

        // vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        //     "/computed_velocity", 10,
        //     std::bind(&EKFLocalizer::velCallback, this, std::placeholders::_1));

        local_cones_polar_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "local_cones_polar", 10,
            std::bind(&EKFLocalizer::local_cones_polar_callback, this, std::placeholders::_1));

        subscriptionz_ = this->create_subscription<std_msgs::msg::Float64>(
            "/imu/yaw_rate_deg", 10,
            std::bind(&EKFLocalizer::yawRateCallback, this, std::placeholders::_1));

        velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/synced_velocity", 10,
            std::bind(&EKFLocalizer::velocity_callback, this, std::placeholders::_1));

        // **FIXED: Initialize state vector properly**
        x_ = Eigen::Vector3d::Zero(); // Initialize to [0, 0, 0]

        // Initialize covariance matrices
        P_ << 10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 1.0;
        Q_ << 0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.05;
        R_ << 0.5, 0,
            0, 0.5;

        // **FIXED: Create timer to periodically publish marker**
        marker_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Publish every 100ms
            std::bind(&EKFLocalizer::publishMarker, this));

        RCLCPP_INFO(this->get_logger(), "EKF Localizer initialized with state: [0, 0, 0]");
    }

private:
    // Storage for cone positions
    std::vector<Eigen::Vector2d> cone_positions_;
    Matrix3d P_, Q_;
    Eigen::Matrix3d F_k, F_k_T;
    Matrix2d R_;
    std::vector<Eigen::Vector2d> cone_polar_positions_;
    Eigen::Vector3d x_; // State vector: [x, y, theta]
    bool initialized_ = false;

    // Velocity and timing variables
    double vx_ = 0.0;
    double vy_ = 0.0;
    double v_ = 0.0;
    double dt_ = 0.05;
    double omega_ = 0;
    double yaw_rate_ = 0.0;
    double yaw_ = 0.0;

    // ROS2 subscribers and publihers
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cones_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr local_cones_polar_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriptionz_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;

    // **FIXED: Add timer for marker publishing**
    rclcpp::TimerBase::SharedPtr marker_timer_;

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        vx_ = msg->twist.linear.x;
        vy_ = msg->twist.linear.y;
        yaw_ = msg->twist.angular.z * M_PI / 180.0;      // deg to rad
        yaw_rate_ = msg->twist.angular.x * M_PI / 180.0; // deg/s to rad/s
    }

    // Callback to store global cone positions in cone_positions_ (vector<Eigen::Vector2d>)
    void conesArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Clear previous cone positions
        cone_positions_.clear();

        // Store each cone's global position (skip the first pose if needed)
        for (size_t i = 1; i < msg->poses.size(); ++i)
        {
            const auto &p = msg->poses[i];
            cone_positions_.emplace_back(p.position.x, p.position.y);
        }

        // RCLCPP_INFO(this->get_logger(), "Updated global cone positions: %zu cones", cone_positions_.size());

        // Log the cone positions array
        logConePositions();
    }
    void yawRateCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        omega_ = msg->data;
    }

    void logConePositions() const
    {
        std::ostringstream oss;
        oss << "Cone positions: ";
        for (const auto &pt : cone_positions_)
        {
            oss << "[" << pt.x() << ", " << pt.y() << "] ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        

        double delta_x = std::cos(x_(2)) * vx_ * dt_;
        double delta_y = std::sin(x_(2)) * vx_ * dt_;
        double delta_yaw = yaw_rate_ * dt_;

        x_(0) += delta_x;
        x_(1) += delta_y;
        x_(2) += delta_yaw;

        // Normalize yaw to [-π, π]
        while (x_(2) > M_PI)
            x_(2)  -= 2 * M_PI;
        while (x_(2)  < -M_PI)
           x_(2)  += 2 * M_PI;
        predictCovariance(x_(2));

        // RCLCPP_INFO(this->get_logger(),
        //             "Received pose: x=%.3f, y=%.3f, theta=%.3f", x_(0), x_(1), x_(2));
    }

    void velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        vx_ = msg->twist.linear.x;
        vy_ = msg->twist.linear.y;

        // Calculate the magnitude of the velocity vector
        v_ = std::sqrt(vx_ * vx_ + vy_ * vy_);
    }

    void predictCovariance(double theta)
    {
        Eigen::Matrix3d F_k, F_k_T;
        calculateJacobianAndTranspose(theta, v_, dt_, F_k, F_k_T);
        P_ = F_k * P_ * F_k_T + Q_;

        // RCLCPP_DEBUG(this->get_logger(), "Predicted covariance updated");
    }

    // Callback to store local cones polar coordinates (r, theta) in cone_polar_positions_ (vector<Eigen::Vector2d>)
    void local_cones_polar_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received local cones polar data with %zu poses", msg->poses.size());

        // Clear previous polar cone positions
        cone_polar_positions_.clear();

        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            double r = msg->poses[i].position.x;
            double theta = msg->poses[i].position.y;

            // Store r and theta in Eigen::Vector2d
            cone_polar_positions_.emplace_back(r, theta);

            RCLCPP_INFO(this->get_logger(), "Cone %zu: r = %.3f, theta = %.3f radians", i, r, theta);
        }

        // **FIXED: Actually call the measurement update function**
        performMeasurementUpdates();
    }

    /**
     * EKF Measurement Update Function
     */
    void measurementUpdate(const Eigen::Vector2d &z,
                           const Eigen::Vector2d &cone_pos)
    {
        // 1. Predict measurement h(x): range r, bearing φ
        double dx = cone_pos.x() - x_(0); // x difference
        double dy = cone_pos.y() - x_(1); // y difference
        double q = dx * dx + dy * dy;     // squared distance

        // **FIXED: Add safety check for division by zero**
        if (q < 1e-6)
        {
            // RCLCPP_WARN(this->get_logger(), "Cone too close to robot, skipping measurement");
            return;
        }

        double r_pred = std::sqrt(q);                 // predicted range
        double phi_pred = std::atan2(dy, dx) - x_(2); // predicted bearing

        // 2. Measurement residual y = z – h(x), normalize bearing
        Eigen::Vector2d y;
        y << z(0) - r_pred, // range residual
            std::atan2(std::sin(z(1) - phi_pred),
                       std::cos(z(1) - phi_pred)); // normalized bearing residual

        // 3. Compute Jacobian H (2×3)
        Eigen::Matrix<double, 2, 3> H;
        H << -dx / r_pred, -dy / r_pred, 0, // ∂h/∂x, ∂h/∂y, ∂h/∂θ for range
            dy / q, -dx / q, -1;            // ∂h/∂x, ∂h/∂y, ∂h/∂θ for bearing

        // 4. Innovation covariance S and Kalman gain K
        Eigen::Matrix2d S = H * P_ * H.transpose() + R_;

        // **FIXED: Add safety check for matrix inversion**
        if (S.determinant() < 1e-6)
        {
            // RCLCPP_WARN(this->get_logger(), "Innovation covariance singular, skipping measurement");
            return;
        }

        Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();

        // 5. State update
        x_ += K * y;

        // 6. Covariance update (Joseph form for numerical stability)
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P_ = (I - K * H) * P_;

        // RCLCPP_INFO(this->get_logger(),
        //             "Updated state: x=%.3f, y=%.3f, theta=%.3f",
        //             x_(0), x_(1), x_(2));
    }

    /**
     * Perform measurement updates for all available cone measurements
     */
    void performMeasurementUpdates()
    {
        if (cone_polar_positions_.empty() || cone_positions_.empty())
            return;

        size_t N = 100;
        size_t start_idx = (cone_polar_positions_.size() > N) ? cone_polar_positions_.size() - N : 0;

        // Track which global cones have already been matched
        std::set<size_t> used_global_indices;

        for (size_t i = start_idx; i < cone_polar_positions_.size(); ++i)
        {
            const Eigen::Vector2d &z = cone_polar_positions_[i];

            // Convert local polar (r, theta) to global (x, y)
            double x_local = z(0) * std::cos(z(1));
            double y_local = z(0) * std::sin(z(1));
            double x_global = x_(0) + std::cos(x_(2)) * x_local - std::sin(x_(2)) * y_local;
            double y_global = x_(1) + std::sin(x_(2)) * x_local + std::cos(x_(2)) * y_local;

            // Find nearest unused global cone
            double min_dist = std::numeric_limits<double>::max();
            size_t best_idx = 0;
            bool found = false;
            for (size_t j = 0; j < cone_positions_.size(); ++j)
            {
                if (used_global_indices.count(j))
                    continue;
                double dx = cone_positions_[j].x() - x_global;
                double dy = cone_positions_[j].y() - y_global;
                double dist = dx * dx + dy * dy;
                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_idx = j;
                    found = true;
                }
            }
            // Optionally, check if min_dist is below a threshold (e.g., 4.0 m^2)
            if (found && min_dist < 2.0)
            {
                used_global_indices.insert(best_idx);
                measurementUpdate(z, cone_positions_[best_idx]);
                RCLCPP_INFO(this->get_logger(),
                            "Associated local cone (r=%.2f, th=%.2f) to global cone [%.2f, %.2f] (dist=%.2f)",
                            z(0), z(1), cone_positions_[best_idx].x(), cone_positions_[best_idx].y(), std::sqrt(min_dist));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "No suitable global cone found for local cone (r=%.2f, th=%.2f)", z(0), z(1));
            }
        }
    };

    // **FIXED: This function now gets called by the timer**
    void publishMarker()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = get_clock()->now();
        m.ns = "ekf";
        m.id = 588;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;

        // Use EKF state for marker position
        m.pose.position.x = x_(0); // x position from EKF
        m.pose.position.y = x_(1); // y position from EKF
        m.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2)); // theta from EKF
        m.pose.orientation = tf2::toMsg(q);

        m.scale.x = 0.5;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;

        marker_pub_->publish(m);

        // // **FIXED: Add logging to confirm marker is being published**
        // RCLCPP_INFO(this->get_logger(),
        //             "Published marker at: x=%.3f, y=%.3f, theta=%.3f",
        //             x_(0), x_(1), x_(2));
    }
};

int main(int argc, char *argv[])

{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFLocalizer>());
    rclcpp::shutdown();
    return 0;
}
