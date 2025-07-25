#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

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
    Vector2i pos;
    float g = std::numeric_limits<float>::infinity();  // cost-so-far
    float f = std::numeric_limits<float>::infinity();  // g + h
    Node* parent = nullptr;

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

struct Vector2iCompare {
    bool operator()(const Vector2i& a, const Vector2i& b) const {
        if (a.x() == b.x())
            return a.y() < b.y();
        return a.x() < b.x();
    }
};

bool inBounds (int& r, int& c, const GridMap& grid_map);

bool isFree (int& r, int& c, const GridMap& grid_map);

float heuristic (const Vector2i& a, const Vector2i& b);

std::vector<Vector2f> astarWithCostMap(const GridMap& grid_map,
                                       const Grid_<float>& cost_map,
                                       const Vector2f& world_start,
                                       const Vector2f& world_goal);