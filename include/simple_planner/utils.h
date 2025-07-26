#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <functional> // for std::hash
#include <cmath>

#include "simple_planner/grid_map.h"
#include "simple_planner/dmap.h"


using Eigen::Vector2f;

bool checkCollision(const GridMap& grid_map, Vector2f& world_pos);

void displayMarker(ros::Publisher& marker_pub, const Vector2f& pos, int id, float r, float g, float b);

void publishPath(const ros::Publisher& path_pub,
                 const std::vector<Eigen::Vector2f>& path_points,
                 const std::string& frame_id = "map");


std::vector<Vector2i> extractObstacles(const GridMap& grid_map);


struct Node {
    int x, y;
    bool operator==(const Node &other) const { return x == other.x && y == other.y; }
};

// Needed for using Node as key in unordered_map
struct NodeHash {
    size_t operator()(const Node &n) const {
        return std::hash<int>()(n.x) ^ std::hash<int>()(n.y << 1);
    }
};

inline float heuristic(Node a, Node b) {
    // return abs(a.x - b.x) + abs(a.y - b.y);  // Manhattan distance
    return std::sqrt((a.x - b.x) * (a.x - b.x) + // Euclidean distance
                     (a.y - b.y) * (a.y - b.y));
}


std::vector<Node> get_neighbors(Node n, const std::vector<std::vector<int>> &grid);
std::vector<Node> reconstruct_path(std::unordered_map<Node, Node, NodeHash> &cameFrom, Node current);
std::vector<Node> A_Star(Node start, Node goal, const std::vector<std::vector<int8_t>> &grid, const Grid_<float>& cost_map);