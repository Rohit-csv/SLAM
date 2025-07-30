// File: src/global_racetrack_mapper.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include <array>
#include <vector>

class GlobalRacetrackMapper : public rclcpp::Node
{
public:
    GlobalRacetrackMapper() : Node("global_racetrack_mapper"), pose_received_(false)
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/robot_pose_2d", 10,
            std::bind(&GlobalRacetrackMapper::pose_callback, this, std::placeholders::_1));

        cones_sub_ = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
            "ground_truth/cones", rclcpp::SensorDataQoS(),
            std::bind(&GlobalRacetrackMapper::cones_callback, this, std::placeholders::_1));

        global_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "global_racetrack_map", 10);

        pose_cones_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "pose_cones_array", 10);

        local_cones_polar_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("local_cones_polar", 10);

        RCLCPP_INFO(this->get_logger(), "Global Racetrack Mapper initialized");
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        latest_pose_ = *msg;
        pose_received_ = true;
    }

    void cones_callback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
    {
        if (!pose_received_)
        {
            RCLCPP_WARN(this->get_logger(), "No pose data - cannot build global map");
            return;
        }
        publish_local_cones_polar(msg); // <-- Add this line

        // Add sure cones to global map
        add_cones_to_global_map(msg->blue_cones, global_blue_cones_);
        add_cones_to_global_map(msg->yellow_cones, global_yellow_cones_);

        // Visualize complete global track
        visualize_global_track();

        // Publish robot pose and all cone coordinates as a PoseArray
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = this->now();
        pose_array_msg.header.frame_id = "map";

        // Add robot pose as the first element
        geometry_msgs::msg::Pose robot_pose;
        robot_pose.position.x = latest_pose_.x;
        robot_pose.position.y = latest_pose_.y;
        robot_pose.position.z = 0.0;
        double cy = cos(latest_pose_.theta * 0.5);
        double sy = sin(latest_pose_.theta * 0.5);
        robot_pose.orientation.x = 0.0;
        robot_pose.orientation.y = 0.0;
        robot_pose.orientation.z = sy;
        robot_pose.orientation.w = cy;
        pose_array_msg.poses.push_back(robot_pose);

        // Add all blue cones
        for (const auto &cone : global_blue_cones_)
        {
            geometry_msgs::msg::Pose cone_pose;
            cone_pose.position.x = cone[0];
            cone_pose.position.y = cone[1];
            cone_pose.position.z = cone[2];
            cone_pose.orientation.w = 1.0;
            pose_array_msg.poses.push_back(cone_pose);
        }
        // Add all yellow cones
        for (const auto &cone : global_yellow_cones_)
        {
            geometry_msgs::msg::Pose cone_pose;
            cone_pose.position.x = cone[0];
            cone_pose.position.y = cone[1];
            cone_pose.position.z = cone[2];
            cone_pose.orientation.w = 1.0;
            pose_array_msg.poses.push_back(cone_pose);
        }

        pose_cones_pub_->publish(pose_array_msg);

        RCLCPP_INFO(this->get_logger(), "Global map: %zu blue + %zu yellow cones, published %zu poses",
                    global_blue_cones_.size(), global_yellow_cones_.size(), pose_array_msg.poses.size());
    }

    void add_cones_to_global_map(
        const std::vector<eufs_msgs::msg::ConeWithCovariance> &local_cones,
        std::vector<std::array<double, 3>> &global_cones)
    {
        for (const auto &cone : local_cones)
        {
            // Transform from local to global frame
            double global_x = latest_pose_.x +
                              (cone.point.x * cos(latest_pose_.theta) -
                               cone.point.y * sin(latest_pose_.theta));
            double global_y = latest_pose_.y +
                              (cone.point.x * sin(latest_pose_.theta) +
                               cone.point.y * cos(latest_pose_.theta));

            std::array<double, 3> global_cone = {global_x, global_y, cone.point.z};

            // Avoid duplicates
            if (!is_duplicate_cone(global_cone, global_cones))
            {
                global_cones.push_back(global_cone);
            }
        }
    }

    bool is_duplicate_cone(const std::array<double, 3> &new_cone,
                           const std::vector<std::array<double, 3>> &existing_cones)
    {
        for (const auto &existing : existing_cones)
        {
            double dist = sqrt(pow(new_cone[0] - existing[0], 2) +
                               pow(new_cone[1] - existing[1], 2));
            if (dist < 3)
                return true; // 3m duplicate threshold
        }
        return false;
    }

    void publish_local_cones_polar(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr &msg)
    {
        geometry_msgs::msg::PoseArray polar_array_msg;
        polar_array_msg.header.stamp = this->now();
        polar_array_msg.header.frame_id = "base_link"; // local frame

        // Blue cones
        for (const auto &cone : msg->blue_cones)
        {
            geometry_msgs::msg::Pose pose;
            double r = sqrt(cone.point.x * cone.point.x + cone.point.y * cone.point.y);
            double theta = atan2(cone.point.y, cone.point.x);
            pose.position.x = r;
            pose.position.y = theta;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            polar_array_msg.poses.push_back(pose);
        }
        // Yellow cones
        for (const auto &cone : msg->yellow_cones)
        {
            geometry_msgs::msg::Pose pose;
            double r = sqrt(cone.point.x * cone.point.x + cone.point.y * cone.point.y);
            double theta = atan2(cone.point.y, cone.point.x);
            pose.position.x = r;
            pose.position.y = theta; //radien
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            polar_array_msg.poses.push_back(pose);
        }

        local_cones_polar_pub_->publish(polar_array_msg);
    }

    void visualize_global_track()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        // Blue cones (left boundary)
        for (const auto &cone : global_blue_cones_)
        {
            auto marker = create_cone_marker(cone, id++, {0.0, 0.0, 1.0}, "blue_boundary");
            marker_array.markers.push_back(marker);
        }

        // Yellow cones (right boundary)
        for (const auto &cone : global_yellow_cones_)
        {
            auto marker = create_cone_marker(cone, id++, {1.0, 1.0, 0.0}, "yellow_boundary");
            marker_array.markers.push_back(marker);
        }

        global_map_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_cone_marker(
        const std::array<double, 3> &position, int id,
        const std::array<float, 3> &color, const std::string &ns)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = position[0];
        marker.pose.position.y = position[1];
        marker.pose.position.z = position[2];
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.5;

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;

        return marker;
    }

    // Member variables
    std::vector<std::array<double, 3>> global_blue_cones_;
    std::vector<std::array<double, 3>> global_yellow_cones_;
    geometry_msgs::msg::Pose2D latest_pose_;
    bool pose_received_;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cones_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_cones_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr local_cones_polar_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalRacetrackMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
