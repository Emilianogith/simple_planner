#include <ros/ros.h>
#include "simple_planner/map.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


void initialCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO_STREAM("Initial pose: x=" << msg->pose.pose.position.x
                     << ", y=" << msg->pose.pose.position.y);
}


void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO_STREAM("Received goal:");
  ROS_INFO_STREAM("Position -> x: " << msg->pose.position.x
                                   << ", y: " << msg->pose.position.y);
  ROS_INFO_STREAM("Orientation -> z: " << msg->pose.orientation.z
                                       << ", w: " << msg->pose.orientation.w);
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    MapHandler map_handler(nh);


    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    ros::Subscriber initial_sub = nh.subscribe("/initialpose", 10, initialCallback);


    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (map_handler.isReady()) {
            ROS_INFO_ONCE("Map is ready!");
            // Test: check if (10, 10) is free
            if (map_handler.isFree(10, 10))
                ROS_INFO("Cell (10,10) is free.");
            else
                ROS_WARN("Cell (10,10) is occupied.");

            if (map_handler.isFree(43, 144))
                ROS_INFO("Cell (43, 144) is free.");
            else
                ROS_WARN("Cell (43, 144) is occupied.");

        }

        rate.sleep();
    }


    return 0;
}
