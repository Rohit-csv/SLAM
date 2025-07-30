// File: src/update_ekf.cpp

#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>


using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Angle–normalization helper
static double normalizeAngle(double a)
{
    const double PI = std::acos(-1.0);
    a = std::fmod(a, 2 * PI);
    if (a >= PI)
        a -= 2 * PI;
    if (a < -PI)
        a += 2 * PI;
    return a;
}

// Jacobian data struct
struct JacobMatrices
{
    Eigen::Vector2d zp;
    Eigen::Matrix<double, 2, 3> Hv;
    Eigen::Matrix<double, 2, 2> Sf;
};

// Compute Jacobians
static JacobMatrices compute_jacobians(
    const std::vector<double> &mu,
    const Eigen::Vector3d &xf,
    const Eigen::Matrix2d &Pf,
    const Eigen::Matrix2d &Q)
{
    JacobMatrices jm;
    double dx = xf.x() - mu[0];
    double dy = xf.y() - mu[1];
    double d2 = dx * dx + dy * dy;
    if (d2 < 1e-6)
    {
        jm.zp = Eigen::Vector2d::Zero();
        jm.Hv = Eigen::Matrix<double, 2, 3>::Zero();
        jm.Sf = Q;
        return jm;
    }
    double d = std::sqrt(d2);
    jm.zp << d, normalizeAngle(std::atan2(dy, dx) - mu[2]);
    jm.Hv << -dx / d, -dy / d, 0.0,
        dy / d2, -dx / d2, -1.0;
    jm.Sf = jm.Hv.block<2, 2>(0, 0) * Pf * jm.Hv.block<2, 2>(0, 0).transpose() + Q;
    return jm;
}

class EKFLocalizer : public rclcpp::Node
{
public:
    EKFLocalizer()
        : Node("update_ekf")
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/robot_pose_2d", 10,
            std::bind(&EKFLocalizer::poseCallback, this, std::placeholders::_1));

        // NEW: bind the array‐filling callback
        cones_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "pose_cones_array", 10,
            std::bind(&EKFLocalizer::conesArrayCallback, this, std::placeholders::_1));

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "ekf_marker_final/marker", 10);

        Q_ << 0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.05;
        R_ << 0.5, 0,
            0, 0.5;

        x_ << 0, 0, 0;
        P_ << 10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 1.0;
    }

private:
    // Storage for cone positions
    std::vector<Eigen::Vector2d> cone_positions_;

    // Callback to fill the array
    void conesArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        cone_positions_.clear();
        for (size_t i = 1; i < msg->poses.size(); ++i)
        {
            const auto &p = msg->poses[i];
            cone_positions_.emplace_back(p.position.x, p.position.y);
        }
        // 1-line logger
        RCLCPP_INFO(get_logger(), "Cones: %s",
                    [&]()
                    {
                        std::ostringstream oss;
                        for (auto &pt : cone_positions_)
                        {
                            oss << "[" << pt.x() << "," << pt.y() << "] ";
                        }
                        return oss.str();
                    }()
                        .c_str());
    }







    geometry_msgs::msg::Pose2D::SharedPtr last_pose_;
    Vector3d x_;
    Matrix3d P_, Q_;
    Matrix2d R_;

    void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        last_pose_ = msg;
        predict();
    }

    void conesCallback(const geometry_msgs::msg::PoseArray::SharedPtr cones)
    {
        if (!last_pose_ || cones->poses.size() < 2)
            return;
        update(cones->poses);
        publishMarker();
        // RCLCPP_INFO(get_logger(), "EKF x,y,θ = %.2f, %.2f, %.2f", x_(0), x_(1), x_(2));
    }

    void predict()
    {
        double dt = 0.1;
        double v = last_pose_->theta;
        double w = last_pose_->y;
        double th = x_(2);

        Vector3d xp;
        xp << x_(0) + v * dt * std::cos(th),
            x_(1) + v * dt * std::sin(th),
            normalizeAngle(th + w * dt);

        Matrix3d F = Matrix3d::Identity();
        F(0, 2) = -v * dt * std::sin(th);
        F(1, 2) = v * dt * std::cos(th);

        P_ = F * P_ * F.transpose() + Q_;
        x_ = xp;
    }

    void update(const std::vector<geometry_msgs::msg::Pose> &cones)
    {
        Eigen::Matrix2d Pf = P_.block<2, 2>(0, 0);
        for (size_t i = 1; i < cones.size(); ++i)
        {
            Eigen::Vector3d xf(cones[i].position.x,
                               cones[i].position.y, 0.0);
            auto jm = compute_jacobians(
                {x_(0), x_(1), x_(2)}, xf, Pf, R_);
            if (jm.Hv.isZero(0))
                continue;

            double dx = xf.x() - x_(0), dy = xf.y() - x_(1);
            Eigen::Vector2d z{
                std::hypot(dx, dy),
                normalizeAngle(std::atan2(dy, dx) - x_(2))};
            Eigen::Vector2d y{
                z(0) - jm.zp(0),
                normalizeAngle(z(1) - jm.zp(1))};

            Eigen::Matrix2d S = jm.Sf;
            Eigen::Matrix<double, 3, 2> K = P_ * jm.Hv.transpose() * S.inverse();
            x_ += K * y;
            P_ = (Matrix3d::Identity() - K * jm.Hv) * P_;
            Pf = P_.block<2, 2>(0, 0);
        }
    }

    void publishMarker()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = get_clock()->now();
        m.ns = "ekf";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x_(0);
        m.pose.position.y = x_(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2));
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.5;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.color.r = 1;
        m.color.g = 1;
        m.color.b = 0;
        m.color.a = 1;
        marker_pub_->publish(m);
    }

    double vx_ = 0.0;
    double vy_ = 0.0;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cones_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFLocalizer>());
    rclcpp::shutdown();
    return 0;
}
