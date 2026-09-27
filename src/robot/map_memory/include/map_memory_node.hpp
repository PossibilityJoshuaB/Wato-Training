#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"


class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void updateMap();
    void odomUpdate(const nav_msgs::msg::Odometry::SharedPtr odoMsg);
    void costmapFetch(const nav_msgs::msg::OccupancyGrid::SharedPtr costmapMsg);

  private:
    robot::MapMemoryCore map_memory_;

    bool hasInitialValue = false;
    double currentX_ = 0.0;
    double currentY_ = 0.0;
    double currentYaw_ = 0.0;
    double lastUpdateX_ = 0.0;
    double lastUpdateY_ = 0.0;

    int local_width_ = 0;
    int local_height_ = 0;
    int local_resolution_ = 0;
    int local_origin_x_ = 0;
    int local_origin_y_ = 0;
    

    nav_msgs::msg::OccupancyGrid global_grid_map_;

    std::vector<int8_t> new_costmap_;
    


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr update_map_timer_;
    
};

#endif 
