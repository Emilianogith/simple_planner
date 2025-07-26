#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class MapHandler {
public:
    MapHandler(ros::NodeHandle& nh);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    bool isReady() const { return map_received_; }
    bool isFree(int x, int y) const;
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    const std::vector<std::vector<int8_t>>& getMapMatrix() const {
    return map_matrix_;
}

private:
    ros::Subscriber map_sub_;
    std::vector<int8_t> map_data_;
    int width_, height_;
    double resolution_;
    bool map_received_;
    std::vector<std::vector<int8_t>> map_matrix_;
    int threshold_ = 50;
};

#endif
