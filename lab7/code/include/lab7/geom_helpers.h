#include <vector>
#include <math.h>
#include <algorithm>
#include <tuple>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<std::tuple<int, int>> bresenham(int x0, int y0, int x1, int y1);
std::tuple<int, int> toLocalIndex(const double &distance, const double &angle, const double &resolution = 0.05, const int &width = 500);

geometry_msgs::PointStamped getTransformedPoint(const double &pos_x, const double &pos_y,
                                                const geometry_msgs::TransformStamped &transformStamped);