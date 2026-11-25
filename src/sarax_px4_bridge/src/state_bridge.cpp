// state_bridge: PX4 /fmu/out/vehicle_odometry -> /sarax/odom + TF world -> base_link.
//
// Same contract as the Python commander used to provide:
//   nav_msgs/Odometry  pose of base_link in `world` (ENU), twist in the child frame (FLU)
//   TF                 world -> sarax_plus_0/base_link
// Stamps use the node clock (sim time in SITL), not PX4's microsecond timestamp, so
// TF lookups at camera-cloud stamps line up with the rest of the ROS side.
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sarax_px4_bridge/frames.hpp"

class StateBridge : public rclcpp::Node
{
public:
  StateBridge()
  : Node("state_bridge")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "sarax_plus_0/base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/sarax/odom");
    const auto px4_topic = declare_parameter<std::string>(
      "px4_odometry_topic", "/fmu/out/vehicle_odometry");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // PX4 publishes /fmu/out/* best effort: a reliable subscription would see nothing.
    sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4_topic, rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {onOdometry(*msg);});
    RCLCPP_INFO(
      get_logger(), "state_bridge: %s -> %s, TF %s -> %s%s", px4_topic.c_str(), odom_topic.c_str(),
      world_frame_.c_str(), base_frame_.c_str(), publish_tf_ ? "" : " (TF disabled)");
  }

private:
  void onOdometry(const px4_msgs::msg::VehicleOdometry & m)
  {
    using namespace sarax::frames;  // NOLINT
    if (m.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PX4 odometry pose_frame=%u is not NED, dropped",
        m.pose_frame);
      return;
    }
    const Eigen::Vector3d p_ned(m.position[0], m.position[1], m.position[2]);
    if (!p_ned.allFinite() || !std::isfinite(m.q[0])) {
      return;
    }
    const Eigen::Vector3d p = ned_to_enu(p_ned);
    const Eigen::Quaterniond q = px4_quat_to_enu_flu(m.q[0], m.q[1], m.q[2], m.q[3]);

    // Twist in the child frame (FLU), the pinocchio free-flyer convention the planner expects.
    Eigen::Vector3d v_body = Eigen::Vector3d::Zero();
    const Eigen::Vector3d v_raw(m.velocity[0], m.velocity[1], m.velocity[2]);
    if (v_raw.allFinite()) {
      if (m.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
        v_body = q.toRotationMatrix().transpose() * ned_to_enu(v_raw);
      } else if (m.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
        v_body = frd_to_flu(v_raw);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "unsupported velocity_frame=%u, velocity set to 0",
          m.velocity_frame);
      }
    }
    Eigen::Vector3d w_body = Eigen::Vector3d::Zero();
    const Eigen::Vector3d w_raw(m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]);
    if (w_raw.allFinite()) {
      w_body = frd_to_flu(w_raw);
    }

    const rclcpp::Time stamp = now();
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = world_frame_;
    od.child_frame_id = base_frame_;
    od.pose.pose.position.x = p.x();
    od.pose.pose.position.y = p.y();
    od.pose.pose.position.z = p.z();
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    od.twist.twist.linear.x = v_body.x();
    od.twist.twist.linear.y = v_body.y();
    od.twist.twist.linear.z = v_body.z();
    od.twist.twist.angular.x = w_body.x();
    od.twist.twist.angular.y = w_body.y();
    od.twist.twist.angular.z = w_body.z();
    odom_pub_->publish(od);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header = od.header;
      ts.child_frame_id = base_frame_;
      ts.transform.translation.x = p.x();
      ts.transform.translation.y = p.y();
      ts.transform.translation.z = p.z();
      ts.transform.rotation = od.pose.pose.orientation;
      tf_->sendTransform(ts);
    }
  }

  std::string world_frame_, base_frame_;
  bool publish_tf_{true};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateBridge>());
  rclcpp::shutdown();
  return 0;
}
