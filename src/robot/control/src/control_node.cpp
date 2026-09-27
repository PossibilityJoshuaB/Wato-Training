#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  lookahead_distance_ = 1.0; 
  goal_tolerance_ = 0.1;     
  linear_speed_ = 0.7;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 
      10, 
      [this](const nav_msgs::msg::Path::SharedPtr msg)
      { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 
      10, 
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 
    10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), 
      [this]()
      { controlLoop(); });
}

void ControlNode::controlLoop()
{
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_)
  {
    return;
  }
  
  const auto goal = current_path_->poses.back();
  double xDiff = goal.pose.position.x - robot_odom_->pose.pose.position.x;
  double yDiff = goal.pose.position.y - robot_odom_->pose.pose.position.y;
  double goalDistance = sqrt((xDiff*xDiff + yDiff*yDiff));

  if (goalDistance < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point)
  {
    return; // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
  for (auto pose : current_path_->poses) {
    geometry_msgs::msg::Point pathPoseAsPoint;
    pathPoseAsPoint.set__x(pose.pose.position.x);
    pathPoseAsPoint.set__y(pose.pose.position.y);

    geometry_msgs::msg::Point robotPoseAsPoint;
    robotPoseAsPoint.set__x(robot_odom_->pose.pose.position.x);
    robotPoseAsPoint.set__y(robot_odom_->pose.pose.position.y);

    if (computeDistance(pathPoseAsPoint, robotPoseAsPoint) > lookahead_distance_) {
      return pose;
    } 
  }
  return std::nullopt; //default
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
  // TODO: Implement logic to compute velocity commands
  double xDiff = target.pose.position.x - robot_odom_->pose.pose.position.x;
  double yDiff = target.pose.position.y - robot_odom_->pose.pose.position.y;
  
  double targetAngle = std::atan2(yDiff, xDiff);
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);
  double angleToRobot = targetAngle - yaw;
  
  double curvature = 2 * std::sin(angleToRobot) / lookahead_distance_;
  double angularVel = curvature * linear_speed_;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = angularVel;
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  double ax = a.x;
  double ay = a.y;
  double bx = b.x;
  double by = b.y;

  double dist = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
  return dist;
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat)
{
  double w = quat.w;
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double yaw = std::atan2(2*(w*z + x*y), 1 - (2*(y*y + z*z)));

  return yaw;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
