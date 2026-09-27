#include "planner_node.hpp"
#include <queue>

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score; // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

PlannerNode::PlannerNode() : Node("planner_node"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL)
{

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point",
      10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered",
      10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/path",
      10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));
}

CellIndex worldToGrid(double worldx, double worldy, double originx, double originy, double resolution)
{
  int gridx = floor((worldx - originx) / resolution);
  int gridy = floor((worldy - originy) / resolution);

  return CellIndex(gridx, gridy);
}

double heuristic(const CellIndex &a, const CellIndex &b)
{
  double ax = a.x;
  double ay = a.y;
  double bx = b.x;
  double by = b.y;

  double dist = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
  return dist;
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;

  RCLCPP_INFO(
      this->get_logger(),
      "MAP RECEIVED: width=%u height=%u cells=%zu",
      current_map_.info.width,
      current_map_.info.height,
      current_map_.data.size());

  if (state_ == State::WAITING_TO_REACH_GOAL)
  {
    planPath();
  }
}


void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;

  RCLCPP_INFO(
      this->get_logger(),
      "GOAL RECEIVED: x=%.2f y=%.2f frame=%s",
      msg->point.x,
      msg->point.y,
      msg->header.frame_id.c_str());

  state_ = State::WAITING_TO_REACH_GOAL;

  planPath();
}


void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

geometry_msgs::msg::Point gridToWorld(
    const CellIndex &cell,
    double originx,
    double originy,
    double resolution)
{
  geometry_msgs::msg::Point point;

  point.x = originx + (cell.x + 0.5) * resolution;
  point.y = originy + (cell.y + 0.5) * resolution;
  point.z = 0.0;

  return point;
}

void PlannerNode::planPath()
{
  if (!goal_received_)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Cannot plan: no goal received.");
    return;
  }

  if (current_map_.data.empty())
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Cannot plan: no map received.");
    return;
  }

  // A*...


  // A* Implementation (pseudo-code)
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  // Compute path using A* on current_map_
  // Fill path.poses with the resulting waypoints.
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  bool found_goal = false;

  CellIndex start = worldToGrid(
      robot_pose_.position.x,
      robot_pose_.position.y,
      current_map_.info.origin.position.x,
      current_map_.info.origin.position.y,
      current_map_.info.resolution);

  CellIndex goal = worldToGrid(
      goal_.point.x,
      goal_.point.y,
      current_map_.info.origin.position.x,
      current_map_.info.origin.position.y,
      current_map_.info.resolution);

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  while (!open_set.empty())
  {
    AStarNode current = open_set.top();
    open_set.pop();

    // 2c. Skip if this is a stale entry already fully processed
    //     (check closed_set here)
    if (closed_set.find(current.index) != closed_set.end())
    {
      continue;
    }

    // 2d. Mark this cell as closed
    closed_set.emplace(current.index);
    // 2e. Check if this is the goal — if so, set found_goal = true and break
    if (current.index == goal)
    {
      found_goal = true;
      break;
    }
    // 2f. Expand neighbors (up to 8 directions)
    //     for each neighbor:
    //       - bounds check against current_map_.info.width / height
    //       - obstacle check against current_map_.data (careful with the flat-index formula)
    //       - skip if already in closed_set
    //       - compute tentative_g
    //       - compare against g_score[neighbor], update + push if better
    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, // straight
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1} // diagonal
    };
    for (const auto &dir : directions)
    {
      CellIndex neighbor(current.index.x + dir.first, current.index.y + dir.second);

      // (a) Bounds check — what condition tells you neighbor is outside the map?
      if (neighbor.x > current_map_.info.width || neighbor.y > current_map_.info.height || neighbor.x < 0 || neighbor.y < 0)
      {
        continue;
      }

      // (b) Closed-set check — skip if already fully processed
      if (closed_set.find(neighbor) != closed_set.end())
      {
        continue;
      }

      // (c) Obstacle check — read current_map_.data using the flat-index formula
      int flat_index = neighbor.y * current_map_.info.width + neighbor.x;
      int8_t costmap_value = current_map_.data[flat_index];
      if (costmap_value > 65)
      {
        continue;
      }

      // (d) Step cost — is this a diagonal or straight move?
      double step_cost = abs(dir.first + dir.second) != 1 ? 14 : 10;

      // (e) Compute tentative_g
      double tentative_g = g_score[current.index] + step_cost;

      // (f) Compare and update
      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second)
      {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current.index;
        open_set.emplace(neighbor, tentative_g + heuristic(neighbor, goal) * 10);
      }
    }
  }

  // ---- Step 3: handle "no path found" ----
  if (!found_goal)
  {
    RCLCPP_WARN(this->get_logger(), "No path found!");
    path_pub_->publish(path); // still publish (empty poses) — or should you skip publishing entirely?
    return;
  }

  std::vector<CellIndex> grid_path;
  CellIndex current = goal;

  // Start at the goal and walk backwards
  grid_path.push_back(current);

  while (current != start)
  {
    auto it = came_from.find(current);

    if (it == came_from.end())
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to reconstruct path!");
      return;
    }

    current = it->second;

    grid_path.push_back(current);
  }

  // We built it goal -> start, so reverse it
  std::reverse(grid_path.begin(), grid_path.end());

  for (const auto &cell : grid_path)
  {
    geometry_msgs::msg::Point world_point = gridToWorld(cell,
                                                        current_map_.info.origin.position.x,
                                                        current_map_.info.origin.position.y,
                                                        current_map_.info.resolution);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose.position = world_point;
    pose_stamped.pose.orientation.w = 1.0;
    path.poses.push_back(pose_stamped);
  }

  path_pub_->publish(path);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
