#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower")
    {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&PathFollower::path_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PathFollower::odom_callback, this, _1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PathFollower::scan_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathFollower::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Path Follower Node Started");
    }

private:

    /* ===================== CALLBACKS ===================== */

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        waypoints_.clear();
        for (auto & pose : msg->poses)
        {
            waypoints_.push_back(
                {pose.pose.position.x,
                 pose.pose.position.y});
        }

        current_index_ = 0;
        RCLCPP_INFO(this->get_logger(),
                    "Received path with %ld waypoints",
                    waypoints_.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        robot_theta_ = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy*qy + qz*qz));
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        min_front_ = 10.0;
        min_left_  = 10.0;
        min_right_ = 10.0;

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            double angle = msg->angle_min + i * msg->angle_increment;
            double r = msg->ranges[i];

            if (!std::isfinite(r))
                continue;

            // Wider front cone ±50° (~0.87 rad)
            if (angle > -0.87 && angle < 0.87)
                min_front_ = std::min(min_front_, r);

            else if (angle >= 0.87 && angle < 1.57)
                min_left_ = std::min(min_left_, r);

            else if (angle <= -0.87 && angle > -1.57)
                min_right_ = std::min(min_right_, r);
        }
    }

    /* ===================== CONTROL LOOP ===================== */

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (current_index_ >= waypoints_.size())
        {
            cmd_pub_->publish(cmd);
            return;
        }

        double safety_threshold = 1.0;
        double clear_threshold  = 1.6;

        /* -------- ENTER AVOIDANCE -------- */
        if (!avoiding_ && min_front_ < safety_threshold)
        {
            avoiding_ = true;
            avoid_steps_ = 0;
            avoid_start_theta_ = robot_theta_;

            if (min_left_ > min_right_)
                avoid_direction_ = 1;
            else
                avoid_direction_ = -1;

            RCLCPP_INFO(this->get_logger(), "Entering avoidance");
        }

        /* -------- AVOIDANCE MODE -------- */
        if (avoiding_)
        {
            avoid_steps_++;

            cmd.angular.z = 0.6 * avoid_direction_;
            cmd.linear.x  = 0.08;

            // Calculate how much we've rotated
            double turned = robot_theta_ - avoid_start_theta_;
            while (turned > M_PI)  turned -= 2.0*M_PI;
            while (turned < -M_PI) turned += 2.0*M_PI;
            turned = std::fabs(turned);

            // Exit only after meaningful rotation + clearance
            if (min_front_ > clear_threshold &&
                turned > 0.8 &&           // ~45 degrees
                avoid_steps_ > 15)
            {
                avoiding_ = false;
                RCLCPP_INFO(this->get_logger(), "Exiting avoidance");
            }

            cmd_pub_->publish(cmd);
            return;
        }

        /* -------- PATH FOLLOWING -------- */

        double target_x = waypoints_[current_index_].first;
        double target_y = waypoints_[current_index_].second;

        double dx = target_x - robot_x_;
        double dy = target_y - robot_y_;

        double distance = std::sqrt(dx*dx + dy*dy);
        double target_theta = std::atan2(dy, dx);
        double angle_error = target_theta - robot_theta_;

        while (angle_error > M_PI)  angle_error -= 2.0*M_PI;
        while (angle_error < -M_PI) angle_error += 2.0*M_PI;

        if (std::fabs(angle_error) > 0.15)
        {
            cmd.angular.z = std::clamp(0.8 * angle_error, -0.6, 0.6);
        }
        else if (distance > 0.25)
        {
            cmd.linear.x = 0.15;
        }
        else
        {
            current_index_++;
            RCLCPP_INFO(this->get_logger(),
                        "Reached waypoint %ld",
                        current_index_);
        }

        cmd_pub_->publish(cmd);
    }

    /* ===================== MEMBERS ===================== */

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::pair<double,double>> waypoints_;
    size_t current_index_ = 0;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;

    double min_front_ = 10.0;
    double min_left_  = 10.0;
    double min_right_ = 10.0;

    bool avoiding_ = false;
    int avoid_direction_ = 0;
    int avoid_steps_ = 0;
    double avoid_start_theta_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}
