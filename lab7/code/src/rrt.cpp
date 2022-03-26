// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "lab7/rrt.h"
#include "lab7/geom_helpers.h"
#include "stdio.h"

#define PI 3.1415927

// Destructor of the RRT class
RRT::~RRT()
{
  // Do something in here, free up used memory, print message, etc.
  ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh)
    : nh_(nh), gen((std::random_device())()), tfListener(tfBuffer)
{

  // TODO: Load parameters from yaml file, you could add your own parameters to
  // the rrt_params.yaml file
  // nh_.getParam("pose_topic", pose_topic);
  // nh_.getParam("scan_topic", scan_topic);
  // nh_.getParam("drive_topic", drive_topic);
  // nh_.getParam("env_viz", env_viz);
  // nh_.getParam("dynamic_viz", dynamic_viz);
  // nh_.getParam("static_viz", static_viz);
  // nh_.getParam("tree_lines", tree_lines);
  // nh_.getParam("map_viz_topic", map_viz_topic);
  // nh_.getParam("map_topic", map_topic);
  nh_.getParam("gain", gain);

  pose_topic = "/odom";
  scan_topic = "/scan";
  map_topic = "/map";

  nh_.getParam("fov", fov);
  nh_.getParam("min_goal_distance", min_goal_distance);
  nh_.getParam("steer_length", steer_length);
  nh_.getParam("lookahead_distance", lookahead_distance);
  nh_.getParam("max_iteration", max_iteration);
  nh_.getParam("goal_distance", goal_distance);

  // flags
  nh_.getParam("publish_grid", publish_grid);

  // ROS publishers
  // TODO: create publishers for the the drive topic, and other topics you might
  // need

  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
  mapvisual_pub_ = nh_.advertise<visualization_msgs::Marker>("/env_viz", 1000);
  points_pub_ = nh_.advertise<visualization_msgs::Marker>("/dynamic_viz", 1000);
  waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/static_viz", 1000);
  edges_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_lines", 1000);

  // ROS subscribers
  // TODO: create subscribers as you need
  pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
  scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
  auto map_message_ptr =
      ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh_);
  nav_msgs::OccupancyGrid map_message = *map_message_ptr;

  // TODO: create a occupancy grid
  // save empty grid for easy reset during scan callback (grid size [500,200])
  // grid params
  height = map_message.info.height;
  width = map_message.info.width;
  resolution = map_message.info.resolution;
  origin_x = map_message.info.origin.position.x;
  origin_y = map_message.info.origin.position.y;
  top_left_x = origin_x + resolution * width;
  top_left_y = origin_y + resolution * height;

  // char tmp[256];
  // getcwd(tmp, 256);
  // std::cout << tmp << std::endl;
  goals = get_goals("/home/reuben/reuben_ws/src/lab7/downsample_5.csv");
  // std::cout << goals[0][0] << " " << goals[0][1] << std::endl;

  occupancy_grid_static = unflatten(map_message.data, height, width);

  occupancy_grid = occupancy_grid_static;

  ROS_INFO("Created new RRT Object.");
}

// REFERENCE: https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/#reading-from-csv
// and: https://stackoverflow.com/questions/53142798/reading-csv-file-to-vector-of-doubles
std::vector<std::vector<double>> RRT::get_goals(std::string path)
{
  std::vector<std::vector<double>> goals;
  std::ifstream data(path, std::ifstream::in);
  // std::cout << data.is_open() << std::endl;
  if (data.is_open())
  {
    std::string line;
    std::vector<double> goal;
    while (std::getline(data, line)) // break lines by \n
    {
      std::stringstream ss(line); // create stringstream for current line
      std::string str_point;
      while (getline(ss, str_point, ',')) // break each line by comma
        goal.push_back(std::stold(str_point));
      goals.push_back(goal);
    }
  }

  return goals;
}

// get individual goalpoint
std::vector<double> RRT::get_goalpoint(bool plot)
{
  std::vector<double> trans_x, trans_y, dist, goal_point;
  std::size_t num_goals = goals.size();
  // transform all waypoints to local frame
  for (std::size_t i = 0; i < num_goals; i++)
  {
    double x, y, tran_x, tran_y, distance;
    x = goals[i][0];
    y = goals[i][1];
    geometry_msgs::PointStamped point = getTransformedPoint(x, y, localTransformStamped); // transform to local frame
    tran_x = point.point.x;
    tran_y = point.point.y;
    trans_x.push_back(tran_x);
    trans_y.push_back(tran_y);
    distance = std::sqrt(std::pow(tran_x, 2) + std::pow(tran_y, 2));
    dist.push_back(distance);
  }

  bool found = false;
  unsigned int counter = 0;
  unsigned int max_searches = goals.size();
  while (!found && counter < max_searches)
  {
    std::vector<double>::iterator res = std::min_element(dist.begin(), dist.end());
    int index = std::distance(dist.begin(), res);
    if (trans_x[index] > goal_distance)
    {
      found = true;
      double point_x, point_y;
      point_x = goals[index][0];
      point_y = goals[index][1];
      goal_point.push_back(point_x);
      goal_point.push_back(point_y);
    }
    else
    {
      dist[index] = 999999; // set to high value so we skip past it next time we find the min
    }
    counter++;
  }

  // if (plot)
  //   publishPoint(goal_point[0], goal_point[1], 0, 255, 0);

  return goal_point;
}

std::vector<int> RRT::flatten(const std::vector<std::vector<int>> &matrix)
{
  std::size_t total_size = 0;
  for (const auto &sub : matrix)
    total_size += sub.size();
  std::vector<int> res;
  res.reserve(total_size);
  for (const auto &sub : matrix)
    res.insert(res.end(), sub.begin(), sub.end());

  return res;
}

// convert 1D array into 2D matrix
std::vector<std::vector<int>> RRT::unflatten(const std::vector<int8_t> &array,
                                             int height, int width)
{
  std::vector<std::vector<int>> res(height, std::vector<int>(width, FREE));
  for (int k = 0; k < array.size(); k++)
  {
    int i = k / width;
    int j = k % width;
    res[i][j] = array[k];
    if ((array[k] != FREE) && (array[k] != OCCUPIED))
    {
      res[i][j] = OCCUPIED; // assume occupied for weird values
    }
  }
  // std::vector<int> grid_coords = get_grid_coords(0, 0);
  // int x = grid_coords[0];
  // int y = grid_coords[1];
  // std::cout<< "x, y = " << x << ", " << y << std::endl;
  // for (int i = 0; i < 100; i++) {
  //     res[x][y+i] = OCCUPIED;
  // }

  return res;
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  // The scan callback, update your occupancy grid here
  // Args:
  //    scan_msg (*LaserScan): pointer to the incoming scan message
  // TODO: update your occupancy grid
  occupancy_grid = occupancy_grid_static;
  double rear_to_lidar = 0.29275; // not sure how to use for now
}

void RRT::publishOccupancy(const std::vector<std::vector<int>> &occupancyGrid)
{
  nav_msgs::OccupancyGrid grid_msg;
  std::vector<int> flattened;
  grid_msg.header.stamp = ros::Time::now();
  grid_msg.header.frame_id = "map";

  // fill in map data
  grid_msg.info.height = height; // measurements in terms of cells
  grid_msg.info.width = width;
  grid_msg.info.resolution = resolution;
  // std::cout << "Pose position x" << last_pose.pose.pose.position.x <<
  // std::endl; grid_msg.info.origin.position.x =
  // last_pose.pose.pose.position.x; grid_msg.info.origin.position.y =
  // last_pose.pose.pose.position.y;
  grid_msg.info.origin.position.x = origin_x;
  grid_msg.info.origin.position.y = origin_y;
  // ?: Not sure if angle required
  grid_msg.info.origin.orientation.w = 1;
  grid_msg.info.origin.orientation.x = 0;
  grid_msg.info.origin.orientation.y = 0;
  grid_msg.info.origin.orientation.z = 0;

  flattened = flatten(occupancy_grid);
  grid_msg.data = std::vector<int8_t>(
      flattened.begin(), flattened.end()); // cast to match message data type

  // map_viz_pub_.publish(grid_msg);
}

// void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
  // The pose callback when subscribed to particle filter's inferred pose
  // The RRT main loop happens here
  // Args:
  //    pose_msg (*PoseStamped): pointer to the incoming pose message
  // Returns:
  //
  // std::cout << "Inside pf callback" << std::endl;

  x_current = pose_msg->pose.pose.position.x;
  y_current = pose_msg->pose.pose.position.y;

  // set the global goal point depending on the car's location
  if (x_current <= 8.00 && y_current <= 1.5)
  { // on the right side of the loop
    x_goal = x_current + 2.30;
    y_goal = -0.75;
    x_limit_top = x_current + 2.50;
    x_limit_bot = x_current;
    y_limit_left = 0.37;
    y_limit_right = -0.66;
  }
  else if (x_current > 8.00 && y_current <= 6.15)
  { // on the top side of the loop
    x_goal = 9.775;
    y_goal = y_current + 2.30;
    x_limit_top = 10.03;
    x_limit_bot = 8.12;
    y_limit_left = y_current + 2.50;
    y_limit_right = y_current;
  }
  else if (x_current >= -12 && y_current > 6.15)
  { // on the left side of the loop
    x_goal = x_current - 2.30;
    y_goal = 8.65;
    x_limit_top = x_current;
    x_limit_bot = x_current - 2.50;
    y_limit_left = 9.15;
    y_limit_right = 8.15;
  }
  else if (x_current < -12 && y_current > 1.5)
  { // on the bottom side of the loop
    x_goal = -13.79;
    y_goal = y_current - 2.30;
    x_limit_top = -13.32;
    x_limit_bot = -14.26;
    y_limit_left = y_current;
    y_limit_right = y_current - 2.50;
  }

  try
  {
    // acquire transform to convert local frame to global frame
    transformStamped =
        tfBuffer.lookupTransform(global_frame, local_frame, ros::Time(0));
    // also acquire to convert to local frame
    localTransformStamped =
        tfBuffer.lookupTransform(local_frame, global_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  last_pose = *pose_msg; // multiple methods require access to pose
  pose_set = true;
  geometry_msgs::Quaternion q = last_pose.pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  heading_current = tf2::impl::getYaw(quat); // heading_current = yaw
  // tree as std::vector
  std::vector<Node> tree;
  // store the final path
  std::vector<Node> paths;
  // setup root node
  Node start;
  start.x = pose_msg->pose.pose.position.x;
  start.y = pose_msg->pose.pose.position.y;
  start.is_root = true;
  tree.push_back(start);

  // std::cout << "Before clearing points" << std::endl;
  marker.points.clear();
  // std::cout << "After clearing points" << std::endl;
  // points to be plot through marker
  geometry_msgs::Point points;

  // get the waypoint
  // std::vector<double> goal = get_goalpoint();
  // x_goal = goal[0];
  // y_goal = goal[1];
  // std::cout << "Goalpoint is: " << goal[0] << ", " << goal[1] << std::endl;

  // TODO: fill in the RRT main loop
  for (unsigned int i = 0; i < max_iteration; i++)
  {
    // std::cout << "Calling sample" << std::endl;
    std::vector<double> sampled_point = sample();
    unsigned int nearest_point = nearest(tree, sampled_point);
    Node new_node = steer(tree[nearest_point], sampled_point);
    new_node.parent = nearest_point;
    if (!check_collision(tree[nearest_point], new_node))
    {
      // std::cout << "Goalpoint is: " << x_goal << ", " << y_goal << std::endl;
      // std::cout << "Car is at: " << start.x << ", " << start.y << std::endl;
      tree.push_back(new_node);
      points.x = new_node.x;
      points.y = new_node.y;
      points.z = 0.0;
      marker.points.push_back(points);
      // std::cout << "Tree now has size: " << tree.size() << std::endl;
      // if (is_goal(new_node, goal[0], goal[1]))
      if (is_goal(new_node, x_goal, y_goal))
      {
        paths = find_path(tree, new_node);
        break;
      }
    }
  }

  // Perform pure pursuit
  double x_target, y_target;
  std::size_t path_len = paths.size();
  // std::cout << "Path size: " << path_len << std::endl;

  for (std::size_t i = 0; i < path_len; i++)
  {
    // std::cout << "Iter..." << i << std::endl;
    double distance = euclidean_distance(paths[path_len - 1 - i].x, paths[path_len - 1 - i].y,
                                         pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    if (distance >= lookahead_distance)
    {
      // std::cout << "Updating target " << i << std::endl;
      x_target = paths[path_len - 1 - i].x;
      y_target = paths[path_len - 1 - i].y;
      break;
    }
  }
  // PP control
  double target_distance = euclidean_distance(x_target, y_target, pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
  double lookahead_angle = std::atan2(y_target - pose_msg->pose.pose.position.y, x_target - pose_msg->pose.pose.position.x);
  double dy = target_distance * std::sin(lookahead_angle - heading_current);
  double angle = gain * dy / (std::pow(target_distance, 2));
  steer_pure_pursuit(angle);

  // blue for sampled points
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  mapvisual_pub_.publish(marker);

  // red for the global goal point
  marker_2.points.clear();
  points.x = x_goal;
  points.y = y_goal;
  marker_2.points.push_back(points);
  marker_2.header.frame_id = "map";
  marker_2.header.stamp = ros::Time();
  marker_2.type = visualization_msgs::Marker::POINTS;
  marker_2.action = visualization_msgs::Marker::ADD;
  marker_2.scale.x = 0.3;
  marker_2.scale.y = 0.3;
  marker_2.color.a = 1.0;
  marker_2.color.r = 1.0;
  marker_2.color.g = 0.0;
  marker_2.color.b = 0.0;
  points_pub_.publish(marker_2);

  // green for the target waypoint
  marker_3.points.clear();
  points.x = x_target;
  points.y = y_target;
  marker_3.points.push_back(points);
  marker_3.header.frame_id = "map";
  marker_3.header.stamp = ros::Time();
  marker_3.type = visualization_msgs::Marker::POINTS;
  marker_3.action = visualization_msgs::Marker::ADD;
  marker_3.scale.x = 0.3;
  marker_3.scale.y = 0.3;
  marker_3.color.a = 1.0;
  marker_3.color.r = 0.0;
  marker_3.color.g = 1.0;
  marker_3.color.b = 0.0;
  waypoint_pub_.publish(marker_3);

  // green connecting lines
  marker_4.header.frame_id = "map";
  marker_4.header.stamp = ros::Time();
  marker_4.type = visualization_msgs::Marker::LINE_STRIP;
  marker_4.action = visualization_msgs::Marker::ADD;
  marker_4.scale.x = 0.05;
  marker_4.scale.y = 0.1;
  marker_4.color.a = 1.0;
  marker_4.color.r = 0.0;
  marker_4.color.g = 1.0;
  marker_4.color.b = 0.0;
  edges_pub_.publish(marker_4);
  // if (path_len == 0)
  // {
  //   // do nothing
  // }
  // else
  // {
  //   for (std::size_t i = 0; i < path_len; i++)
  //   {
  //     std::cout << "Iter..." << i << std::endl;
  //     double distance = euclidean_distance(paths[path_len - 1 - i].x, paths[path_len - 1 - i].y,
  //                                          pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
  //     if (distance >= lookahead_distance)
  //     {
  //       x_target = paths[path_len - 1 - i].x;
  //       y_target = paths[path_len - 1 - i].y;
  //       break;
  //     }
  //   }
  //   // PP control
  //   double target_distance = euclidean_distance(x_target, y_target, pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
  //   double lookahead_angle = std::atan2(y_target - pose_msg->pose.pose.position.y, x_target - pose_msg->pose.pose.position.x);
  //   double dy = target_distance * std::sin(lookahead_angle - heading_current);
  //   double angle = 2 * dy / (std::pow(target_distance, 2));
  //   steer_pure_pursuit(angle);
  // }
  // if (path_len != 0)
  // {
  // double target_distance = euclidean_distance(x_target, y_target, pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
  // double lookahead_angle = std::atan2(y_target - pose_msg->pose.pose.position.y, x_target - pose_msg->pose.pose.position.x);
  // double dy = target_distance * std::sin(lookahead_angle - heading_current);
  // double angle = 2 * dy / (std::pow(target_distance, 2));
  // steer_pure_pursuit(angle);
  // }
}

void RRT::steer_pure_pursuit(const double &angle)
{
  ackermann_msgs::AckermannDriveStamped drive_msg = ackermann_msgs::AckermannDriveStamped();
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = "laser";
  drive_msg.drive.steering_angle = angle;
  if (std::abs(angle) > 20.0 / 180.0 * PI) {
      drive_msg.drive.speed = 0.5;
  } else if (std::abs(angle) > 10.0 / 180.0 * PI) {
      drive_msg.drive.speed = 1.0;
  } else {
      drive_msg.drive.speed = 1.5;
  }
  // std::cout << "Publishing drive message with angle: " << angle << std::endl;
  drive_pub_.publish(drive_msg);
}

std::vector<double> RRT::sample() // WORKS
{
  // This method returns a sampled point from the free space
  // You should restrict so that it only samples a small region
  // of interest around the car's current position
  // Args:
  // Returns:
  //     sampled_point (std::vector<double>): the sampled point in free space

  // sample points locally in front of the car
  double sx, sy, gx, gy;
  // x_limit_top = 2.5;
  // x_limit_bot = 0;
  // y_limit_left = 0.8;
  // y_limit_right = -0.75;
  std::vector<double> sampled_point;
  std::uniform_real_distribution<> dis_x(x_limit_bot, x_limit_top);
  std::uniform_real_distribution<> dis_y(y_limit_left, y_limit_right);

  sx = dis_x(gen); // sample locally
  sy = dis_y(gen);

  // geometry_msgs::PointStamped sampledPoint = getTransformedPoint(sx, sy, transformStamped);
  // gx = sampledPoint.point.x;
  // gy = sampledPoint.point.y;

  // std::cout << "Sampling..." << gx << ", " << gy << std::endl;
  // publishPoint(gx, gy, 255, 0, 0); // publish red point for visualisation

  sampled_point.push_back(sx);
  sampled_point.push_back(sy);

  // sampled_point.push_back(gx);
  // sampled_point.push_back(gy);

  return sampled_point;
}

void RRT::publishPoint(const double &x, const double &y, const int &r, const int &g, const int &b)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose pos;
  pos.position.x = x;
  pos.position.y = y;
  pos.position.z = 0;
  pos.orientation.x = 0;
  pos.orientation.y = 0;
  pos.orientation.z = 0;
  pos.orientation.w = 1;
  marker.pose = pos;

  geometry_msgs::Vector3 scale;
  scale.x = 0.4;
  scale.y = 0.4;
  scale.z = 0.4;
  marker.scale = scale;

  std_msgs::ColorRGBA colour;
  colour.r = r;
  colour.g = g;
  colour.b = b;
  colour.a = 1;
  marker.color = colour;

  marker.lifetime.sec = 0;
  marker.frame_locked = true;
  // point_pub_.publish(marker);
}

double RRT::euclidean_distance(const double &x1, const double &y1, const double &x2, const double &y2)
{
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point)
{
  // This method returns the nearest node on the tree to the sampled point
  // Args:
  //     tree (std::vector<Node>): the current RRT tree
  //     sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //     nearest_node (int): index of nearest node on the tree

  int nearest_node = 0;
  // TODO: fill in this method
  double sx, sy;
  sx = sampled_point[0];
  sy = sampled_point[1];

  double min_distance = euclidean_distance(tree[0].x, tree[0].y, sx, sy);
  for (std::size_t i = 0; i < tree.size(); i++)
  {
    // std::cout << "Finding nearest point iter: " << i << std::endl;
    if (euclidean_distance(tree[i].x, tree[i].y, sx, sy) < min_distance)
    {
      min_distance = euclidean_distance(tree[i].x, tree[i].y, sx, sy);
      nearest_node = i;
    }
  }

  return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point)
{
  // The function steer:(x,y)->z returns a point such that z is “closer”
  // to y than x is. The point z returned by the function steer will be
  // such that z minimizes ||z−y|| while at the same time maintaining
  //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

  // basically, expand the tree towards the sample point (within a max dist)

  // Args:
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //    new_node (Node): new node created from steering

  Node new_node;
  // TODO: fill in this method
  double euclidean = euclidean_distance(nearest_node.x, nearest_node.y, sampled_point[0], sampled_point[1]);
  // think off as slider between the nearest node and the sampled point
  new_node.x = nearest_node.x + steer_length / euclidean * (sampled_point[0] - nearest_node.x);
  new_node.y = nearest_node.y + steer_length / euclidean * (sampled_point[1] - nearest_node.y);

  return new_node;
}

std::vector<int> RRT::get_grid_coords(double global_x, double global_y)
{
  // This method returns the grid coordinates when given the global coordinates

  // Args:
  //    global_x, global_y: x and y coordinates in global frame accordingly
  // Returns:
  //    vector of grid coordinates (grid_x, grid_y)
  int grid_y = std::floor((global_x - origin_x) /
                          resolution); // the grid visualization is flipped
  int grid_x = std::floor((global_y - origin_y) / resolution);
  return {grid_x, grid_y};
}

bool RRT::check_occupied(int grid_x, int grid_y)
{
  // This method checks if a grid cell is occupied when given the grid
  // coordinates
  bool is_occupied = false;
  int buffer = 6; // number of grid cells as buffers on all sides of obstacle
  for (int i = -buffer; i <= buffer; i++) {
    for (int j = -buffer; j <= buffer; j++) {
      if (occupancy_grid[grid_x + i][grid_y + j] == OCCUPIED) {
        is_occupied = true;
        break;
      }
    }
  }
  return is_occupied;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node)
{
  // This method returns a boolean indicating if the path between the
  // nearest node and the new node created from steering is collision free
  // Args:
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    new_node (Node): new node created from steering
  // Returns:
  //    collision (bool): true if in collision, false otherwise

  bool collision = false;
  // TODO: fill in this method
  const int NUM_CHECKPOINTS = 500;
  double checkpoint_x, checkpoint_y;
  std::vector<int> grid_coords(2, 0);
  for (int i = 0; i <= NUM_CHECKPOINTS; i++)
  {
    checkpoint_x = nearest_node.x +
                   (double)i / NUM_CHECKPOINTS * (new_node.x - nearest_node.x);
    checkpoint_y = nearest_node.y +
                   (double)i / NUM_CHECKPOINTS * (new_node.y - nearest_node.y);
    grid_coords = get_grid_coords(checkpoint_x, checkpoint_y);
    if (check_occupied(grid_coords[0], grid_coords[1]))
    {
      collision = true;
      // std::cout << "occupied cell (" << checkpoint_x << ", " << checkpoint_y
      //           << ")" << std::endl;
      // std::cout << "grid coord (" << grid_coords[0] << ", " << grid_coords[1]
      //           << ")" << std::endl;
      break;
    }
  }

  return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y)
{
  // This method checks if the latest node added to the tree is close
  // enough (defined by goal_threshold) to the goal so we can terminate
  // the search and find a path
  // Args:
  //   latest_added_node (Node): latest addition to the tree
  //   goal_x (double): x coordinate of the current goal
  //   goal_y (double): y coordinate of the current goal
  // Returns:
  //   close_enough (bool): true if node close enough to the goal

  bool close_enough = false;
  // TODO: fill in this method
  double distance = std::pow(std::pow(latest_added_node.x - goal_x, 2) +
                                 std::pow(latest_added_node.y - goal_y, 2),
                             0.5);
  if (distance < min_goal_distance)
  {
    // std::cout << "Good enough to goal" << std::endl;
    close_enough = true;
  }

  return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree,
                                 Node &latest_added_node)
{
  // This method traverses the tree from the node that has been determined
  // as goal
  // Args:
  //   latest_added_node (Node): latest addition to the tree that has been
  //      determined to be close enough to the goal
  // Returns:
  //   path (std::vector<Node>): the vector that represents the order of
  //      of the nodes traversed as the found path

  geometry_msgs::Point points;
  marker_4.points.clear();
  points.x = x_goal;
  points.y = y_goal;
  points.z = 0.0;
  marker_4.points.push_back(points);
  points.x = latest_added_node.x;
  points.y = latest_added_node.y;
  points.z = 0.0;
  marker_4.points.push_back(points);

  int i = 0;
  std::vector<Node> found_path;
  // TODO: fill in this method
  found_path.push_back(latest_added_node);
  Node next_node = tree[latest_added_node.parent];
  while (!next_node.is_root)
  {
    // std::cout << "Finding path counter: " << i << std::endl;
    i++;
    found_path.push_back(next_node);
    next_node = tree[next_node.parent];
    points.x = next_node.x;
    points.y = next_node.y;
    points.z = 0.0;
    marker_4.points.push_back(points);
  }

  found_path.push_back(tree[0]); // always ends at root
  points.x = tree[0].x;
  points.y = tree[0].y;
  points.z = 0.0;
  marker_4.points.push_back(points);

  return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node)
{
  // This method returns the cost associated with a node
  // Args:
  //    tree (std::vector<Node>): the current tree
  //    node (Node): the node the cost is calculated for
  // Returns:
  //    cost (double): the cost value associated with the node

  double cost = 0;
  // TODO: fill in this method

  return cost;
}

double RRT::line_cost(Node &n1, Node &n2)
{
  // This method returns the cost of the straight line path between two nodes
  // Args:
  //    n1 (Node): the Node at one end of the path
  //    n2 (Node): the Node at the other end of the path
  // Returns:
  //    cost (double): the cost value associated with the path

  double cost = 0;
  // TODO: fill in this method

  return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{
  // This method returns the set of Nodes in the neighborhood of a
  // node.
  // Args:
  //   tree (std::vector<Node>): the current tree
  //   node (Node): the node to find the neighborhood for
  // Returns:
  //   neighborhood (std::vector<int>): the index of the nodes in the
  //   neighborhood

  std::vector<int> neighborhood;
  // TODO:: fill in this method
  return neighborhood;
}

double toDegrees(double r) { return r * 180 / PI; }

double toRadians(double d) { return d * PI / 180; }
