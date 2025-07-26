#include "simple_planner/utils.h"

#include <queue>
#include <map>
#include <iostream>
#include <unordered_map>
#include <climits>



bool checkCollision(const GridMap& grid_map, Vector2f& world_pos){
    Vector2f grid_f = grid_map.world2grid(world_pos);
    int row = static_cast<int>(grid_f.y());
    int col = static_cast<int>(grid_f.x());

    if (row >= 0 && row < grid_map.rows && col >= 0 && col < grid_map.cols) {
        uint8_t occupancy = grid_map(row, col);

        if (occupancy > 200) {
            // free
            return false;
            
        } else {
            // obstacle
            ROS_WARN("world position: (%.2f, %.2f), grid position: (%d, %d) is in collision.",
            world_pos.x(), world_pos.y(), row, col);   
        } 
    } else {
            ROS_WARN("world position: (%.2f, %.2f) is out of bounds.",
    world_pos.x(), world_pos.y());
    }

    return true;
}


void displayMarker(ros::Publisher& marker_pub, const Vector2f& pos, int id,
                   float r, float g, float b) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
    ROS_INFO("Marker ID %d published at (%.2f, %.2f)", id, pos.x(), pos.y());
}


void publishPath(const ros::Publisher& path_pub,
                 const std::vector<Eigen::Vector2f>& path_points,
                 const std::string& frame_id ) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;

    for (const auto& point : path_points) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now(); 
        pose.header.frame_id = frame_id;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
}


std::vector<Vector2i> extractObstacles(const GridMap& grid_map) {
    std::vector<Vector2i> obstacles;

    for (int r = 0; r < grid_map.rows; ++r) {
        for (int c = 0; c < grid_map.cols; ++c) {
            uint8_t value = grid_map(r, c);  
            if (value < 200) {  
                obstacles.emplace_back(c, r);  
            }
        }
    }
    return obstacles;
}


std::vector<Node> get_neighbors(Node n, const std::vector<std::vector<int8_t>> &grid) {
    std::vector<Node> neighbors;
    std::vector<std::pair<int, int>> directions = {{1,0},{-1,0},{0,1},{0,-1}};
    for (auto d : directions) {
        int nx = n.x + d.first;
        int ny = n.y + d.second;

        //based on data in /map topic : 0 = free; 100 = occupied;
        if (nx >= 0 && ny >= 0 && ny < grid.size() && nx < grid[0].size() && grid[ny][nx] == 0) {
            neighbors.push_back({nx, ny});
        }
    }
    return neighbors;
}

std::vector<Node> reconstruct_path(std::unordered_map<Node, Node, NodeHash>& cameFrom, Node current) {
    std::vector<Node> path = {current};
    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom[current];
        path.insert(path.begin(), current);
    }
    return path;
}

std::vector<Node> A_Star(Node start, Node goal, const std::vector<std::vector<int8_t>> &grid, const Grid_<float>& cost_map) {

    struct Compare {
        bool operator()(std::pair<int, Node> a, std::pair<int, Node> b) {
            return a.first > b.first;
        }
    };
    // define the priority queue
    std::priority_queue<std::pair<int, Node>, std::vector<std::pair<int, Node>>, Compare> openSet;
    openSet.push({0, start});

    std::unordered_map<Node, Node, NodeHash> cameFrom;
    std::unordered_map<Node, float, NodeHash> gScore, fScore;

    gScore[start] = 0.0;
    fScore[start] = heuristic(start, goal);

    while (!openSet.empty()) {
        Node current = openSet.top().second;
        openSet.pop();

        if (current == goal)
            return reconstruct_path(cameFrom, current);

        
        for (Node neighbor : get_neighbors(current, grid)) {
            // Use cost from cost_map for the neighbor cell
            float dist_to_obstacle = cost_map(neighbor.y, neighbor.x);
            float obstacle_penalty = 60.0f / (dist_to_obstacle + 1e-3f);

            float tentative_gScore = gScore[current] + obstacle_penalty;

            // if neighbor not met before or has an higher cost of the new one
            if (!gScore.count(neighbor) || tentative_gScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;   // parent of neighbor is current
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = tentative_gScore + heuristic(neighbor, goal);
                openSet.push({fScore[neighbor], neighbor});
            }
        }
    }

    ROS_WARN("A* failed to find a path.");
    return {};  
}