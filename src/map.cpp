#include "simple_planner/map.h"

MapHandler::MapHandler(ros::NodeHandle& nh) : map_received_(false) {
    map_sub_ = nh.subscribe("/map", 1, &MapHandler::mapCallback, this);
}

void MapHandler::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    width_ = msg->info.width;
    height_ = msg->info.height;
    resolution_ = msg->info.resolution;
    map_data_ = msg->data;
    map_received_ = true;

    // Resize and fill 2D matrix
    map_matrix_.resize(height_);  // rows = height
    for (int y = 0; y < height_; ++y) {
        map_matrix_[y].resize(width_);
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            map_matrix_[y][x] = map_data_[index];
        }
    }

    // to display
    // for (int y = 0; y < height_; ++y) {
    //     std::stringstream ss;
    //     for (int x = 0; x < width_; ++x) {
    //         ss << static_cast<int>(map_matrix_[y][x]) << " ";
    //     }
    //     ROS_INFO_STREAM("Row " << y << ": " << ss.str());
    // }



    ROS_INFO("Map received: %d x %d", width_, height_);
}

bool MapHandler::isFree(int x, int y) const {
    if (!map_received_ || x < 0 || x >= width_ || y < 0 || y >= height_)
        return false;


    int8_t value = map_matrix_[y][x];
    return (value >= 0 && value < threshold_);  // threshold: < 50 = free
}