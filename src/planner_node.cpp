#include <ros/ros.h>
#include "simple_planner/map.h"
#include "simple_planner/grid_map.h"
#include "simple_planner/utils.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <chrono>

float resolution = 0.05f; 

GridMap grid_map(resolution);
Vector2f world_initial_pos;
Vector2f world_goal_pos;
bool initial_pos_received = false, goal_pos_received = false, new_received = false;

ros::NodeHandle* nh_ptr = nullptr;
ros::Publisher marker_pub;



void initialCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    Vector2f world_pos = Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    if (!checkCollision(grid_map, world_pos)){
        ROS_INFO_STREAM("Initial pose: x=" << msg->pose.pose.position.x
                     << ", y=" << msg->pose.pose.position.y);
        world_initial_pos = world_pos;
        initial_pos_received = true;
        new_received = true;

        // display a green marker for the initial position
        displayMarker(marker_pub, world_initial_pos, 0, 0.0f, 1.0f, 0.0f);
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Vector2f world_pos = Vector2f(msg->pose.position.x, msg->pose.position.y);
    if (!checkCollision(grid_map, world_pos)){
        ROS_INFO_STREAM("Goal pose: x=" << msg->pose.position.x
                     << ", y=" << msg->pose.position.y);
        world_goal_pos = world_pos;
        goal_pos_received = true;
        new_received = true;

        // display a red marker for the goal position
        displayMarker(marker_pub, world_goal_pos, 1, 1.0f, 0.0f, 0.0f);    
    }
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun simple_planner planner_node <map_image.png>");
        return 1;
    }

    const char* map_filename = argv[1];
    ROS_INFO("Map file path: %s", map_filename);
    
    // Construct the grid_map
    grid_map.loadFromImage(map_filename, resolution);


    // Construct the distance map in a grid-like structure
    DMap dmap(grid_map.rows, grid_map.cols);
    std::vector<Vector2i> obstacle_cells = extractObstacles(grid_map);
    dmap.compute(obstacle_cells, 100);  

    Grid_<float> cost_map;
    dmap.copyTo(cost_map);

    ROS_INFO("Finished loading stuffs");


    ros::NodeHandle nh;
    nh_ptr = &nh;

    MapHandler map_handler(nh);

    // initialize publishers
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    ros::Subscriber initial_sub = nh.subscribe("/initialpose", 10, initialCallback);


    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (initial_pos_received && goal_pos_received && new_received) {
            ROS_INFO("Both initial and goal positions received. Ready to plan.");
            ROS_INFO_STREAM("Initial Position: " << world_initial_pos.transpose());
            ROS_INFO_STREAM("Goal Position:    " << world_goal_pos.transpose());
        

            auto start_time = std::chrono::high_resolution_clock::now();

            // converet world positions into grid positions because A_Star works for cells
            Eigen::Vector2i start_grid = grid_map.world2grid(world_initial_pos).cast<int>();
            Eigen::Vector2i goal_grid = grid_map.world2grid(world_goal_pos).cast<int>();

            Node start, goal;
            start.x = start_grid.x(); start.y = start_grid.y();
            goal.x = goal_grid.x();goal.y = goal_grid.y();

            std::vector<Node> path = A_Star(start, goal, map_handler.getMapMatrix(), cost_map);

            // outpath is in grid coords, in order to publish it we must convert it into world coords
            std::vector<Eigen::Vector2f> world_path;
            world_path.reserve(path.size());

            // Convert each Node position from grid to world coordinates
            for (const Node& node : path) {
                Eigen::Vector2f grid_pos(node.x, node.y);
                Eigen::Vector2f world_pos = grid_map.grid2world(grid_pos);
                world_path.push_back(world_pos);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            std::cout << "Planning execution time: " << duration.count() << " ms" << std::endl;

            publishPath(path_pub, world_path);
            new_received = false;
        }

        rate.sleep();
    }


    return 0;
}
