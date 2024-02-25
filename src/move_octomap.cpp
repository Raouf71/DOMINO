#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "nav_msgs/Path.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

ros::Publisher octomap_pub_;
std::shared_ptr<octomap::OcTree> my_uav;
ros::Publisher path_pub_;
nav_msgs::Path uav_path;
geometry_msgs::PoseStamped current_pose;


void publishOctomap(double x, double y) {
    octomap_msgs::Octomap octomap_msg;

    // Create a new octomap with updated positions

    std::shared_ptr<octomap::OcTree> updated_octomap = std::make_shared<octomap::OcTree>(my_uav->getResolution());
    std::cout << my_uav->getResolution() << std::endl;

    for (auto it = my_uav->begin_leafs(), end = my_uav->end_leafs(); it != end; ++it) {
        if (my_uav->isNodeOccupied(*it)) {
            octomap::point3d point = it.getCoordinate();
            point.x() += x;
            point.y() += y;
            point.z() = 0.5;
            
            updated_octomap->updateNode(point, true); // Set the new position as occupied
        }
    }

    // Replace the old octomap with the updated one
    my_uav = updated_octomap;

    if (octomap_msgs::binaryMapToMsg(*my_uav, octomap_msg)) {
        // ROS_INFO("Octomap converted to message successfully");
    }
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msg.header.frame_id = "map";
    octomap_pub_.publish(octomap_msg);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {

    // TODO: Axis action

    // Step-by-step movement using buttons
    if (joy_msg->buttons[0] == 1) {
        
        publishOctomap(0.0, -1.0);

        current_pose.pose.position.y -= 1.0;

        // Add pose to path
        ROS_INFO("Moving backward");
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(current_pose);
        // ROS_WARN("Added to path (Button)");
        path_pub_.publish(uav_path);

    }

    if (joy_msg->buttons[3] == 1) {

        publishOctomap(0.0, 1.0);
        current_pose.pose.position.y += 1.0;

        // Add pose to path
        ROS_INFO("Moving Forward");
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(current_pose);
        // ROS_WARN("Added to path (Button)");
        path_pub_.publish(uav_path);

    }

    // Button 7 - Start uav at the middle of the map
    if (joy_msg->buttons[7]==1)
    {
        publishOctomap(0.0, 0.0);

        current_pose.header.frame_id = "map";
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = 0.0;
        current_pose.pose.position.y = 0.0;
        current_pose.pose.position.z = 0.5;
        
        // Add initial pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.clear(); // Clear container for restart
        uav_path.poses.push_back(current_pose);
        ROS_WARN("Added to path (Restarting)");

        path_pub_.publish(uav_path);
    }
 
    // Button 2/1 - Take a step to the left/right
    if (joy_msg->buttons[2] == 1)
    {
        publishOctomap(-1.0, 0.0);
        ROS_INFO("Moving Left");
        current_pose.pose.position.x -= 1.0;

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(current_pose);
        // ROS_WARN("Added to path (Button)");
        path_pub_.publish(uav_path);

    } else if (joy_msg->buttons[1] == 1)
    {
        publishOctomap(1.0, 0.0);
        ROS_INFO("Moving Right");
        current_pose.pose.position.x += 1.0;

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(current_pose);
        // ROS_WARN("Added to path (Button)");
        path_pub_.publish(uav_path);
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gamepad_octomap_node");
    ros::NodeHandle nh_;
    ROS_INFO("Node is spinning ...");

    // Load the octomap from file
    std::string filename2 = "/home/uas/catkin_ws/src/update_map/uav_voxel.bt";
    my_uav = std::make_shared<octomap::OcTree>(filename2);

    ros::Subscriber joy_sub_;
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("uav_fov", 10);

    path_pub_ = nh_.advertise<nav_msgs::Path>("uav_path", 10);

    ros::spin();
    return 0;
}
