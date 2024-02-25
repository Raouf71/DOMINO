#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
// #include <octomap/ColorOcTreeNode.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_viewer");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<octomap_msgs::Octomap>("explored", 1);

    if (argc < 2) {
        ROS_ERROR("You must provide a path to the octomap file as an argument!");
        return 0;
    }

    // if (argc < 3) {
    //     ROS_ERROR("You must provide a topic name to publish to!");
    //     return 0;
    // }

    std::string octomap_file = argv[1];
    octomap::OcTree tree(0.05); // resolution

    if (tree.readBinary(octomap_file)) {
        ROS_INFO("Octomap file %s read successfully", octomap_file.c_str());
    } else {
        ROS_ERROR("Failed to read octomap file %s", octomap_file.c_str());
        return 0;
    }

    octomap_msgs::Octomap octomap_msg;

    if (octomap_msgs::binaryMapToMsg(tree, octomap_msg)) {
        ROS_INFO("Octomap converted to message successfully");
    } else {
        ROS_ERROR("Failed to convert octomap to message");
        return 0;
    }

    ros::Rate rate(1);  // 1 Hz
    while (ros::ok()) {
        octomap_msg.header.stamp = ros::Time::now();
        octomap_msg.header.frame_id = "map";
        map_pub.publish(octomap_msg);
        rate.sleep();
    }

    return 0;
}
