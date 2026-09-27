#include <chrono>
#include <memory>
#include <math.h>

#include "costmap_node.hpp"


// CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
// {
//   // Initialize the constructs and their parameters
//   string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
//   timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
// }

// // Define the timer to publish a message every 500ms
// void CostmapNode::publishMessage()
// {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      rclcpp::SensorDataQoS(),
      std::bind(&CostmapNode::receiveMessage, this, std::placeholders::_1));
  occupancy_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::receiveMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int size = msg->ranges.size();
  std::vector<std::vector<int>> costmap(400, std::vector<int>(400, 0));

  float min = msg->range_min;
  float max = msg->range_max;
  
  float occupiedEq = max - min;

  float resolution = 0.1;
  float origin_x = -20;
  float origin_y = -20;

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    double angle = msg->angle_min + i * msg->angle_increment;
    double range = msg->ranges[i];
    if (range < msg->range_max && range > msg->range_min)
    {
      float x = range * cosf(angle);
      float y = range * sinf(angle);

      int grid_coord_x = floor((x - origin_x) / resolution);
      int grid_coord_y = floor((y - origin_y) / resolution);

      if ((grid_coord_x > 400 || grid_coord_x < 0) || (grid_coord_y > 400 || grid_coord_y < 0)) continue;
      costmap[grid_coord_x][grid_coord_y] = (range / occupiedEq) * 100;
      
      float adj_corner_dist = sqrt(0.1*0.1 + 0.1*0.1);
      float adj_cell_dist = 0.1; 
      float dist;

      for(int8_t i = -1; i <= 1; ++i) {
        for(int8_t j = -1; j <= 1; ++j){

          if((i == 0) && (j==0)) continue;
          int nx = grid_coord_x + i;
          int ny = grid_coord_y + j;
          if (nx < 0 || nx >= 400 || ny < 0 || ny >= 400) continue;

          ((abs(i) == 1) && (abs(j) == 1)) ? dist = adj_corner_dist :  dist = adj_cell_dist; 
          
          float inflated_cost = 75 * (1 - dist);
          
          if (inflated_cost > costmap[grid_coord_x+i][grid_coord_y+j]) {
            costmap[grid_coord_x+i][grid_coord_y+j] = inflated_cost;
          }
        }
      }

    }
  }
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.header.frame_id = msg->header.frame_id;
  
  grid_msg.info.resolution = resolution;
  grid_msg.info.width = 400;
  grid_msg.info.height = 400;
  grid_msg.info.origin.position.x = origin_x;
  grid_msg.info.origin.position.y = origin_y;
  grid_msg.info.origin.orientation.w = 1.0;

  for(int i=0; i < 400; i++) {
    for (int j=0; j < 400; j++) {
      grid_msg.data.push_back(static_cast<int8_t>(costmap[j][i]));
    }
  }

  occupancy_grid_pub->publish(grid_msg);

  // RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", msg->ranges.size());
  // RCLCPP_INFO(this->get_logger(), "Min angle: ", msg->angle_min);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}