#include <ros/ros.h>
#include "simple_planner/map.h"
#include "simple_planner/grid_map.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

float resolution = 0.05f; 

GridMap grid_map(resolution);
Vector2f world_initial_pos;
Vector2f world_goal_pos;
bool initial_pos_received = false, goal_pos_received = false;


bool checkCollision(Vector2f& world_pos){
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

void initialCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    Vector2f world_pos = Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    if (!checkCollision(world_pos)){
        ROS_INFO_STREAM("Initial pose: x=" << msg->pose.pose.position.x
                     << ", y=" << msg->pose.pose.position.y);
        world_initial_pos = world_pos;
        initial_pos_received = true;

    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Vector2f world_pos = Vector2f(msg->pose.position.x, msg->pose.position.y);
    if (!checkCollision(world_pos)){
        ROS_INFO_STREAM("Goal pose: x=" << msg->pose.position.x
                     << ", y=" << msg->pose.position.y);
        world_goal_pos = world_pos;
        goal_pos_received = true;
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

    grid_map.loadFromImage(map_filename, resolution);

    ros::NodeHandle nh;

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    ros::Subscriber initial_sub = nh.subscribe("/initialpose", 10, initialCallback);


    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (initial_pos_received && goal_pos_received) {
            ROS_INFO("Both initial and goal positions received. Ready to plan.");
            ROS_INFO_STREAM("Initial Position: " << world_initial_pos.transpose());
            ROS_INFO_STREAM("Goal Position:    " << world_goal_pos.transpose());
            // TODO: call planner here
        }

        rate.sleep();
    }


    return 0;
}
