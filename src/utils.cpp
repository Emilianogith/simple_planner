#include "simple_planner/utils.h"

#include <queue>
#include <map>
#include <cmath>



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

bool inBounds (int& r, int& c, const GridMap& grid_map) {
        return r >= 0 && r < grid_map.rows && c >= 0 && c < grid_map.cols;
};

bool isFree (int& r, int& c, const GridMap& grid_map) {
    return inBounds(r, c, grid_map) && grid_map(r, c) > 200;
};

float heuristic (const Vector2i& a, const Vector2i& b) {
    return (a - b).cast<float>().norm();  // Euclidean distance
};

std::vector<Vector2f> astarWithCostMap(const GridMap& grid_map,
                                       const Grid_<float>& cost_map,
                                       const Vector2f& world_start,
                                       const Vector2f& world_goal) {
    Vector2i start = grid_map.world2grid(world_start).cast<int>();
    Vector2i goal  = grid_map.world2grid(world_goal).cast<int>();

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::map<Vector2i, Node, Vector2iCompare> all_nodes;

    Node start_node{start, 0.0f, heuristic(start, goal), nullptr};
    open.push(start_node);
    all_nodes[start] = start_node;

    const std::vector<Vector2i> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
    };

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (current.pos == goal) {
            // Reconstruct path
            std::vector<Vector2f> path;
            Node* n = &all_nodes[current.pos];
            while (n) {
                path.push_back(grid_map.grid2world(n->pos.cast<float>()));
                n = n->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& d : directions) {
            Vector2i neighbor_pos = current.pos + d;
            if (!isFree(neighbor_pos.y(), neighbor_pos.x(), grid_map))
                continue;

            float step_cost;
            if (d.x() == 0 || d.y() == 0){
                step_cost = 1.0f;
            }else{
                step_cost = std::sqrt(2.0f);
            }
           

            float dist_to_obstacle = cost_map(neighbor_pos.y(), neighbor_pos.x());
            float obstacle_penalty = 100.0f / (dist_to_obstacle + 1e-3f);  // higher if closer

            float tentative_g = current.g + step_cost + obstacle_penalty;

            Node& neighbor = all_nodes[neighbor_pos];
            if (tentative_g < neighbor.g) {
                neighbor.pos = neighbor_pos;
                neighbor.g = tentative_g;
                neighbor.f = tentative_g + heuristic(neighbor_pos, goal);
                neighbor.parent = &all_nodes[current.pos];
                open.push(neighbor);
            }
        }
    }

    ROS_WARN("A* failed to find a path.");
    return {};
}