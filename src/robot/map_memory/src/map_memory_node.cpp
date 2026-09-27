#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::costmapFetch, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::odomUpdate, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  update_map_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MapMemoryNode::updateMap, this)
  );

  global_grid_map_.info.resolution = 0.1;
  global_grid_map_.info.width = 400;
  global_grid_map_.info.height = 400;
  global_grid_map_.info.origin.position.x = -20;
  global_grid_map_.info.origin.position.y = -20;
  global_grid_map_.info.origin.orientation.w = 1.0;
  global_grid_map_.data.assign(400*400, -1);

}

// void MapMemoryNode::calculateDistanceTravelled(const nav_msgs::msg::Odometry::SharedPtr msg) {
//   if(!initialValueSet) {
//     initialValueX = msg->pose.pose.position.x;
//     initialValueY = msg->pose.pose.position.y;
//     initialValueSet = true;
//   }
//   float currentValueX = msg->pose.pose.position.x;
//   float currentValueY = msg->pose.pose.position.y;

//   float distance = sqrt(pow((currentValueX - initialValueX), 2) + pow((currentValueY - initialValueY), 2));
// }
void MapMemoryNode::odomUpdate(const nav_msgs::msg::Odometry::SharedPtr odoMsg) {
  if(!hasInitialValue) {
    lastUpdateX_ = odoMsg->pose.pose.position.x;
    lastUpdateY_ = odoMsg->pose.pose.position.y;
    hasInitialValue = true;
  }
  currentX_ = odoMsg->pose.pose.position.x;
  currentY_ = odoMsg->pose.pose.position.y;

  double w = odoMsg->pose.pose.orientation.w;
  double x = odoMsg->pose.pose.orientation.x;
  double y = odoMsg->pose.pose.orientation.y;
  double z = odoMsg->pose.pose.orientation.z;
  currentYaw_ = std::atan2(2*(w*z + x*y), 1 - (2*(y*y + z*z)));
}
void MapMemoryNode::costmapFetch(const nav_msgs::msg::OccupancyGrid::SharedPtr costmapMsg) {
  new_costmap_.assign(costmapMsg->data.begin(), costmapMsg->data.end());
  local_width_ = costmapMsg->info.width;
  local_height_ = costmapMsg->info.height;
  local_resolution_ = costmapMsg->info.resolution;
  local_origin_x_ = costmapMsg->info.origin.position.x;
  local_origin_y_ = costmapMsg->info.origin.position.y;
}

void MapMemoryNode::updateMap() {
  double distance = sqrt(pow((currentX_ - lastUpdateX_), 2) + pow((currentY_ - lastUpdateY_), 2));
  
  if(distance > 1.5) {
    // int local_width = l;
    // int local_height ;
    // double local_res;

    for (int local_y = 0; local_y < local_height_; ++local_y) {
      for (int local_x = 0; local_x < local_width_; ++local_x) {
        int8_t value = new_costmap_[local_y * local_width_ + local_x];

        if (value < 0) continue; 

        double lx = (local_resolution_ * local_x) + local_origin_x_;
        double ly = (local_resolution_ * local_y) + local_origin_y_;

        double rotated_x = lx*cos(currentYaw_) - ly * sin(currentYaw_);
        double rotated_y = lx*sin(currentYaw_) + ly * cos(currentYaw_);

        double world_x = rotated_x + currentX_;
        double world_y = rotated_y + currentY_;

        int global_x = floor((world_x-global_grid_map_.info.origin.position.x) / global_grid_map_.info.resolution);
        int global_y = floor((world_y-global_grid_map_.info.origin.position.y) / global_grid_map_.info.resolution);
      

        if (global_x < 0 || global_x >= 400 || global_y < 0 || global_y >= 400) continue;
        global_grid_map_.data[global_y * 400 + global_x] = value;
      }
    }
    global_grid_map_.header.stamp = this->get_clock()->now();
    global_grid_map_.header.frame_id = "sim_world";
    map_pub_->publish(global_grid_map_);
      // for (int j = 0; j < 400; j++) {
      //   global_grid_map_.data.push_back(new_costmap_[j]);
      // }
      lastUpdateX_ = currentX_;
      lastUpdateY_ = currentY_;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
