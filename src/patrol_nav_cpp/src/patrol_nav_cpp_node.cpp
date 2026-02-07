#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>
#include <chrono> // For timer
#include <mutex>  // For pose data protection

// Standard ROS 2 messages
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Custom Interface (For testing Cross-file bugs later)
#include "patrol_interfaces/msg/patrol_status.hpp"

using namespace std::chrono_literals;

class PatrolNavCppNode : public rclcpp::Node
{
  public:
    PatrolNavCppNode()
    : Node("patrol_nav_cpp_node")
    {
      // QoS for Sensor (Best Effort is standard for Lidar)
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      
      // 1. Subscriber: LaserScan
      scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        std::bind(&PatrolNavCppNode::scan_callback, this, std::placeholders::_1)
      );

      // 1.1 Subscriber: AMCL Pose (Robot localization)
      // We keep the latest (x, y) so the timer can publish it in PatrolStatus.
      amcl_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&PatrolNavCppNode::amcl_pose_callback, this, std::placeholders::_1)
      );

      // 2. Publisher: Velocity Command
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      // 3. Publisher: Robot Status (For Python Logger)
      status_pub_ = this->create_publisher<patrol_interfaces::msg::PatrolStatus>("/patrol/status", 10);

      // 4. Timer: Publish status every 1 second
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&PatrolNavCppNode::timer_callback, this));
      
      RCLCPP_INFO(this->get_logger(), "Patrol Nav Node has started.");
    }

  private:
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_x_ = _msg->pose.pose.position.x;
        latest_pose_y_ = _msg->pose.pose.position.y;
        pose_received_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
      // Logic: Process only the front cone (+/- 30 degrees)
      
      const float angle_min_scan = _msg->angle_min;
      const float angle_inc = _msg->angle_increment;
      const size_t count = _msg->ranges.size();

      // Define Region of Interest (ROI)
      const float kDeg2Rad = 0.017453292519943295f;
      const float min_angle_roi = -30.0f * kDeg2Rad;
      const float max_angle_roi = 30.0f * kDeg2Rad;

      float min_distance = std::numeric_limits<float>::infinity();

      // Loop through all points
      for (size_t i = 0; i < count; ++i) {
        const float r = _msg->ranges[i];
        
        // Calculate current angle
        const float angle_raw = angle_min_scan + (i * angle_inc);

        // Normalize angle to -PI to +PI (Safety check)
        const float angle = static_cast<float>(std::remainder(angle_raw, 2.0f*M_PI));

        // Check if inside ROI
        if (angle < min_angle_roi || angle > max_angle_roi) {
          continue;
        }

        // Filter invalid data
        if (!std::isfinite(r) || r < _msg->range_min || r > _msg->range_max) {
          continue;
        }

        // Find closest obstacle in ROI
        if (r < min_distance) {
          min_distance = r;
        }
      }

      // --- Navigation Decision ---
      geometry_msgs::msg::Twist cmd_msg;
      
      if (min_distance < 0.4f) { // Safety distance 0.5m
        // Stop and Turn
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.5; // Rotate left
        current_status_ = "obstacle_avoidance";
      } else {
        // Move Forward
        cmd_msg.linear.x = 0.2; // 0.2 m/s
        cmd_msg.angular.z = 0.0;
        current_status_ = "patrolling";
      }

      cmd_vel_pub_->publish(cmd_msg);
    }

    void timer_callback()
    {
        auto message = patrol_interfaces::msg::PatrolStatus();
        message.id = 1; // Robot ID

        // Robot position from AMCL
        // If AMCL is not ready yet, it will stay at (0.0, 0.0).
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          if (pose_received_) {
            message.location.x = latest_pose_x_;
            message.location.y = latest_pose_y_;
          } else {
            message.location.x = 0.0;
            message.location.y = 0.0;
          }
        }
        message.status = current_status_;

        status_pub_->publish(message);
    }

    // Member Variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<patrol_interfaces::msg::PatrolStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Latest localization data from AMCL
    std::mutex pose_mutex_;
    bool pose_received_ = false;
    double latest_pose_x_ = 0.0;
    double latest_pose_y_ = 0.0;
    
    std::string current_status_ = "idle";
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNavCppNode>());
  rclcpp::shutdown();
  return 0;
}