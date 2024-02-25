#ifndef UPDATE_MAP_UPDATE_MAP_H
#define UPDATE_MAP_UPDATE_MAP_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

class UpdateMap 
{
public:
    UpdateMap(ros::NodeHandle &node_handle);

private:
    ros::NodeHandle *node_;

    // ros communication
    ros::Publisher current_pose;

    // ros::Subscriber spawnMap:  l map eli sna3neha bl create.cpp
    // oder nesta3mel viewer.cpp (fih l map published deja)

    // params

    // callbacks

    // methods
    void SpawnUAV(float resolution, double edgeSize);
    void publishMarker(const geometry_msgs::Point &markerPoint, const std::string text);
};

#endif // UPDATE_MAP_UPDATE_MAP_H