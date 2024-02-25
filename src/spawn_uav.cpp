#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
// #include <octomap/ColorOcTreeNode.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "cube_publisher");
    ros::NodeHandle nh;

    // Create a ROS publisher for octomap
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("spawn_uav_fov", 1);

    // Create an octree
    float resolution = 1.0;
    octomap::OcTree tree(resolution);

    // Create FOV
    for (double x = -2; x < 2; x += resolution) {
        for (double y = -2; y < 2; y += resolution) {
            // Update the node in the OctoMap
            tree.updateNode(octomap::point3d(x, y, 0.0), true);
    }

    // Update inner occupancy of the OctoMap
    tree.updateInnerOccupancy();

    // Convert octree to octomap_msgs::Octomap message
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = "map";
    octomap_msgs::binaryMapToMsg(tree, octomap_msg);

    // Loop rate (1 Hz)
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        // Publish the octomap message
        octomap_pub.publish(octomap_msg);

        // Spin to keep the node running
        ros::spinOnce();

        // Sleep to achieve the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}
}
