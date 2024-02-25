#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <queue>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include <geometry_msgs/Point.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ColorOcTree.h>
#include <octomap/AbstractOcTree.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "update_map/create_tree.h"
// #include "update_map/tree.h"


ros::Publisher octomap_pub_;
ros::Publisher explored_pub_;
ros::Publisher current_pose_;
ros::Publisher path_pub_;
ros::Publisher query_pose_;
ros::Subscriber joy_sub_;
ros::Subscriber map_sub_;
ros::Publisher marker_pub_;

geometry_msgs::PoseStamped uav_current_pose;
nav_msgs::Path uav_path;


double resolution = 1.0;    // TODO: Adjust resolution to input

octomap::ColorOcTree exploredTree(resolution);  

// Load map and uav
std::string filename1 = "/home/uas/catkin_ws/src/update_map/2d_map.bt";
// std::string filename1 = "/home/uas/catkin_ws/src/update_map/2d_map_obstacle.bt";
// std::string filename1 = "/home/uas/catkin_ws/src/update_map/creative_room.bt";
octomap::OcTree* my_map = new octomap::OcTree(filename1);

std::shared_ptr<octomap::OcTree> my_uav;

// TODO: Free map

// joystick parameters
double deadzone_threshold = 0.1;
bool turbo_on = false;

// TODO: Add my_octomap_package view_octomap 2d_map.bt 
//       to launch file

// TODO: Add utils.h file for better code overview
// Function to publish ray as a line marker

// Function to convert octomap::point3d to geometry_msgs::Point
geometry_msgs::Point pointOctomapToRos(const octomap::point3d& octomap_point) {
    geometry_msgs::Point ros_point;
    ros_point.x = octomap_point.x();
    ros_point.y = octomap_point.y();
    ros_point.z = octomap_point.z();
    return ros_point;
}


void publishUAV(const octomap::point3d &pose) {
    
    // Create the drone marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "uav_voxel";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.x();
    marker.pose.position.y = pose.y();
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0); //persistent marker

    query_pose_.publish(marker);
}


void publishPoint(const octomap::point3d &query_pose) {
    
    // Create the drone marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "query_point";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = query_pose.x();
    marker.pose.position.y = query_pose.y();
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.75;
    marker.scale.y = 0.75;
    marker.scale.z = 0.75; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1); //persistent marker

    query_pose_.publish(marker);
}


void publishRayAsMarker(const octomap::point3d& start, const octomap::point3d& end, ros::Publisher& marker_pub, const octomap::point3d& intersection_pt) {
    visualization_msgs::Marker ray_marker;
    ray_marker.header.frame_id = "map"; // Adjust the frame_id if necessary
    ray_marker.header.stamp = ros::Time::now();
    ray_marker.ns = "ray_markers";
    ray_marker.id += 1;

    ray_marker.type = visualization_msgs::Marker::LINE_STRIP;
    ray_marker.action = visualization_msgs::Marker::ADD;

    ray_marker.points.push_back(pointOctomapToRos(start));
    ray_marker.points.push_back(pointOctomapToRos(end));

    ray_marker.lifetime = ros::Duration(1);
    ray_marker.scale.x = 0.1; // Line width
    ray_marker.color.b = 1.0; // Red color
    ray_marker.color.a = 1.0; // Fully opaque

    publishPoint(intersection_pt);
    marker_pub_.publish(ray_marker);
}


double computeDistBetweenPoses(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
    double x = pose2.position.x - pose1.position.x;
    double y = pose2.position.y - pose1.position.y;
    return sqrt(pow(x,2)+pow(y,2));
}




void broadcastUAVtf(tf2_ros::TransformBroadcaster broadcaster) {

    // Broadcast TF transform

    ros::Rate loop_rate(5.0); 
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "uav_frame";

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = uav_current_pose.pose.position.x;
    transformStamped.transform.translation.y = uav_current_pose.pose.position.y;
    transformStamped.transform.translation.z = uav_current_pose.pose.position.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);  // Adjust orientation as needed
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    broadcaster.sendTransform(transformStamped);
    // loop_rate.sleep();
    ros::spin();
}


void update() {


    double drone_x = uav_current_pose.pose.position.x;
    double drone_y = uav_current_pose.pose.position.y;
    double drone_z = uav_current_pose.pose.position.z;

    octomap::KeyRay key_ray;

    // Third ray 
    key_ray.addKey(my_uav->coordToKey(octomap::point3d(0, 0, 0)));     

    // key_ray.addKey(my_uav->coordToKey(octomap::point3d(0.5 + drone_x, 0.5 + drone_y, drone_z)));     
    // key_ray.addKey(my_uav->coordToKey(octomap::point3d(0.5 + drone_x, 1.5 + drone_y, drone_z))); 

    for (auto& key : key_ray)
    {
        // my_map->getRayIntersection()
        octomap::OcTreeNode* arena_node = my_map->search(key);

        if (arena_node != NULL && my_map->isNodeOccupied(arena_node))
        {
            // Collision detected, handle it (e.g., add to explored_arena)

            ROS_INFO("Point inside map bounds");
            ROS_INFO("Point represents an occupied voxel");

            // Get the coordinates of the intersection point
            octomap::point3d intersectionPoint = my_map->keyToCoord(key);

            ROS_INFO_STREAM("Intersection Point " << ": (" << intersectionPoint.x() << ", " << intersectionPoint.y() << ", " << 0.5 << ")");

            octomap::ColorOcTreeNode* node = exploredTree.updateNode(intersectionPoint.x(), intersectionPoint.y(), 0.5, true);

            // Set color information (here, using red)
            node->setColor(1.0, 0.0, 0.0);
            
            // Update inner occupancy of the OctoMap
            exploredTree.updateInnerOccupancy();

            octomap_msgs::Octomap explored_octomap;
            if (octomap_msgs::binaryMapToMsg(exploredTree, explored_octomap)) {
                // ROS_INFO("Octomap converted to message successfully");
            }

            explored_octomap.header.stamp = ros::Time::now();
            explored_octomap.header.frame_id = "map";
            explored_pub_.publish(explored_octomap);

        }
    }
}


void updateEnv() {

    // TODO: Raouf
    // The data will be much smaller if you call octomap.toMaxLikelihood() and octomap.prune() before
    // my_map->toMaxLikelihood();
    // my_map->prune();

    // Iterate over all occupied nodes in the OctoMap

    ROS_INFO_STREAM("Current pose: " << "(" << uav_current_pose.pose.position.x << " , " << uav_current_pose.pose.position.y << " , " <<uav_current_pose.pose.position.z << ")");
    
    double drone_x = uav_current_pose.pose.position.x;
    double drone_y = uav_current_pose.pose.position.y;
    double drone_z = 0.5;


    // Add outer voxel coordinates to the octree (if needed)
    // std::vector<octomap::point3d> outerVoxels3x3 = {{1.5,0.5, 0.5},{1.5,1.5, 0.5}, {0.5,1.5, 0.5},{-0.5,1.5, 0.5}, {-0.5,0.5, 0.5}, {-0.5,-0.5, 0.5}, {0.5,-0.5, 0.5},{1.5,-0.5, 0.5}};
    std::vector<octomap::point3d> outerVoxels5x5 = {{2.5, -1.5, 0.5},{2.5,-0.5, 0.5},{2.5,0.5, 0.5},{2.5,1.5, 0.5},{2.5,2.5, 0.5},
                                                    {1.5,2.5, 0.5},{0.5,2.5, 0.5},{-0.5,2.5, 0.5},{-1.5,2.5, 0.5} ,
                                                    {-1.5,1.5, 0.5},{-1.5,0.5, 0.5},{-1.5,-0.5, 0.5},{-1.5,-1.5, 0.5}
                                                    ,{-0.5,-1.5, 0.5},{0.5,-1.5, 0.5},{1.5,-1.5, 0.5}};
    // std::vector<octomap::point3d> outerVoxels5x5 = {{3, -2, 0.5},{3, -1, 0.5},{3  , 1  , 0.5},{3  , 2 , 0.5},{3, 3, 0.5},
    //                                                 {2,  3, 0.5},{1,  3, 0.5},{-1 , 3  , 0.5},{-2 , 3 , 0.5} ,
    //                                                 {-2, 2, 0.5},{-2, 1, 0.5},{-2 , -1 , 0.5},{-2 , -2, 0.5},
    //                                                 {-1,-2, 0.5},{1, -2, 0.5},{2  , -2 , 0.5}};

    // Center point
    octomap::point3d center(drone_x+0.5, drone_y+0.5, drone_z);

    // Stack to store generated keys
    std::queue<octomap::point3d> keyQueue;

    // Iterate over outer voxels and compute rays
    for (size_t i = 0; i < outerVoxels5x5.size(); ++i) {
        auto& voxel = outerVoxels5x5[i];
        voxel.x() += drone_x;
        voxel.y() += drone_y;

        // Compute ray keys
        octomap::KeyRay rayKeys;
        if (my_uav->computeRayKeys(voxel, center, rayKeys)) {
            
            // Check adjacent voxels first
            for (auto it = rayKeys.rbegin(); it != rayKeys.rend(); ++it) {
                const octomap::OcTreeKey& key = *it;        

                // std::cout << "Ray Points " << ": (" << rayPoints.x() << ", " << rayPoints.y() << ")" << std::endl;
           
                octomap::OcTreeNode* arena_node = my_map->search(key);
                if (arena_node != NULL && my_map->isNodeOccupied(arena_node))
                {
                    // std::cout << count << std::endl;
                    // Collision detected, handle it (e.g., add to explored_arena)
                    ROS_INFO("Point inside map bounds");
                    ROS_INFO("Point represents an occupied voxel");

                    // Get the coordinates of the intersection point
                    octomap::point3d intersectionPoint = my_map->keyToCoord(key);
                    octomap::point3d intersectionPointUAV = my_uav->keyToCoord(key);

                    keyQueue.push(intersectionPoint);

                    publishRayAsMarker(center, voxel, marker_pub_, intersectionPoint);
                    ros::Duration(3.5).sleep();

                    ROS_INFO_STREAM("Intersection Point" << ": (" << intersectionPoint.x() << ", " << intersectionPoint.y() << ", " << intersectionPoint.z() << ")");
                    // ROS_INFO_STREAM("Intersection (UAV)" << ": (" << intersectionPointUAV.x()-drone_x << ", " << intersectionPointUAV.y()-drone_y << ", " << intersectionPointUAV.z() << ")");

                    break; // Break after detecting the first collision along the ray
                }
            }
            // std::cout << std::endl;
        }
    }

    ////////////////////////////// Create and update the new map

    while (!keyQueue.empty()) {

        std::cout << std::endl;
        // ROS_WARN("Intersection list not empty yet ...");

        // ROS_WARN("Im inside");
        // octomap::point3d frontPoint = keyStack.top();
        octomap::point3d frontPoint = keyQueue.front();

        std::cout << std::endl;
        ROS_INFO_STREAM("Front Point: (" << frontPoint.x() << ", " << frontPoint.y() << ", " << frontPoint.z() << ")");

        octomap::ColorOcTreeNode* node = exploredTree.updateNode(frontPoint.x(), frontPoint.y(), 0.5, true);

        // Set color information (here, using red)
        node->setColor(1.0, 0.0, 0.0);
       
        // Update inner occupancy of the OctoMap
        exploredTree.updateInnerOccupancy();

        octomap_msgs::Octomap explored_octomap;
        if (octomap_msgs::binaryMapToMsg(exploredTree, explored_octomap)) {
            // ROS_INFO("Octomap converted to message successfully");
        }

        explored_octomap.header.stamp = ros::Time::now();
        explored_octomap.header.frame_id = "map";
        explored_pub_.publish(explored_octomap);

        // ros::Duration(3).sleep();
        // keyStack.pop();
        keyQueue.pop();
    }
    
    if (keyQueue.empty())
        // std::cout << std::endl;
        ROS_WARN("List in Empty");


    // Measure execution time
    // auto start = std::chrono::high_resolution_clock::now();
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout<<std::endl;
    // ROS_WARN("Execution Time: ", duration.count(), " microseconds");
    // // }

    // std::cout << exploredTree.getNumLeafNodes() << std::endl;
    // std::cout << my_map->getResolution() << std::endl;

    // if (exploredTree.getNumLeafNodes() == 44) {
        // ROS_WARN("**** MAPPING FINISHED SUCCESSEFULLY");
}
  

void moveUAVwithFOV(double x, double y, double z, bool restart) {
    octomap_msgs::Octomap octomap_msg;

    // TODO: As long as no collision has been detected
    // TODO: Check for collision between FOV and the OctoMap nodes

    // if (checkForCollision(uav_pose, my_map->) {
    //     ROS_WARN("Collision detected!");
    //     // don't move

    // } else if (!checkForCollision(uav_pose, my_map)) {
    //     ROS_WARN("No collision detected!");
          // continue
    // }

    // Create a new octomap with updated positions
    std::shared_ptr<octomap::OcTree> updated_octomap = std::make_shared<octomap::OcTree>(my_uav->getResolution());

    if (restart)
    {
        for (auto it = my_uav->begin_leafs(), end = my_uav->end_leafs(); it != end; ++it) {
            if (my_uav->isNodeOccupied(*it)) {
                octomap::point3d point = it.getCoordinate();
                point.x() -= x;
                point.y() -= y;
                point.z() = 0.5;
                updated_octomap->updateNode(point, true); // Set the new position as occupied
            }
        }
    }
    
    else {
        for (auto it = my_uav->begin_leafs(), end = my_uav->end_leafs(); it != end; ++it) {
            if (my_uav->isNodeOccupied(*it)) {
                octomap::point3d point = it.getCoordinate();
                point.x() += x;
                point.y() += y;
                point.z() += z;
                updated_octomap->updateNode(point, true); // Set the new position as occupied
            }
        }
    }

    // Replace the old octomap with the updated one
    my_uav = updated_octomap;

    if (octomap_msgs::binaryMapToMsg(*my_uav, octomap_msg)) {
        // ROS_INFO("Octomap converted to message successfully");
        octomap_msg.header.stamp = ros::Time::now();
        octomap_msg.header.frame_id = "map";
        // publishUAV(octomap::point3d(uav_current_pose.pose.position.x+0.5, uav_current_pose.pose.position.y+0.5, 0.5));
        octomap_pub_.publish(octomap_msg);
    }
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
 
    // ROS_INFO("Im inside the callback");

    tf2_ros::TransformBroadcaster tf_broadcaster;

    /////////////////////////////////////////////////////// Axis action events

    /*
    // Axis 1 - Move forward/backward continuously
    if (joy_msg->axes[1] > deadzone_threshold)
    {
        // last_axis_value = joy_msg->axes[1];
        // ROS_INFO_STREAM("now_axis" << joy_msg->axes[1]);
        spawnUAVwithFOV(0.0, 0.05, uav_current_pose); 
        uav_current_pose.pose.position.y += 0.05;
        ROS_INFO("Moving forward continuously");

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        ROS_WARN("Added to path (Axis)");
        
        path_pub_.publish(uav_path);

    } else if (joy_msg->axes[1] < -deadzone_threshold)
    {
        spawnUAVwithFOV(0.0, -0.05, uav_current_pose); 
        uav_current_pose.pose.position.y -= 0.05;
        ROS_INFO("Moving backward continuously");

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        ROS_WARN("Added to path (Axis)");
        
        path_pub_.publish(uav_path);

    } 

    // Axis 3 - Move left/right continuously
    if (joy_msg->axes[3] > deadzone_threshold)
    {
        spawnUAVwithFOV(-0.05, 0.0, uav_current_pose); 
        uav_current_pose.pose.position.x -= 0.05;
        ROS_INFO("Moving left continuously");

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        ROS_WARN("Added to path (Axis)");
        
        path_pub_.publish(uav_path);

    } else if (joy_msg->axes[3] < -deadzone_threshold)
    {
        spawnUAVwithFOV(0.05, 0.05, uav_current_pose); 
        uav_current_pose.pose.position.x += 0.05;
        ROS_INFO("Moving right continuously");

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        ROS_WARN("Added to path (Axis)");
        
        path_pub_.publish(uav_path);
    }

    */

    /////////////////////////////////////////////////////// Button action events

    // Button 7 - Start uav at the middle of the map
    if (joy_msg->buttons[7]==1)
    {
        std::cout<<std::endl;
        ROS_WARN("***** RESTARTING *****");
        
        // Spawn/Respawn drone at (0,0,0.5)
        moveUAVwithFOV(uav_current_pose.pose.position.x, uav_current_pose.pose.position.y, 0.5, true);

        // todo
        // exploredTree.castRay(); 

        uav_current_pose.header.frame_id = "map";
        uav_current_pose.header.stamp = ros::Time::now();
        uav_current_pose.pose.position.x = 0.0;
        uav_current_pose.pose.position.y = 0.0;
        uav_current_pose.pose.position.z = 0.5;
        
        // Add initial pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.clear(); // Clear container for restart
        uav_path.poses.push_back(uav_current_pose);
        path_pub_.publish(uav_path);

        updateEnv();
        broadcastUAVtf(tf_broadcaster);
    }
    
    // Button 3/0 - Take a step forward/backward
    if (joy_msg->buttons[3] == 1)
    {
        std::cout<<std::endl;
        ROS_WARN("----------------------> Moving Forward");
        
        uav_current_pose.pose.position.y += 1.0;
        moveUAVwithFOV(0.0, 1.0, 0.0, false); // Supposing the uav is heading in y-axis, TODO: could be both, we need the orientation ...

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        path_pub_.publish(uav_path);

        updateEnv();
        broadcastUAVtf(tf_broadcaster);

    } else if (joy_msg->buttons[0] == 1)
    {
        std::cout<<std::endl;
        ROS_WARN("----------------------> Moving Backward");
        
        uav_current_pose.pose.position.y -= 1.0;
        moveUAVwithFOV(0.0, -1.0, 0.0, false);
        
        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        path_pub_.publish(uav_path);

        updateEnv();
        broadcastUAVtf(tf_broadcaster);
    }
    
    // Button 2/1 - Take a step to the left/right
    if (joy_msg->buttons[2] == 1)
    {
        std::cout<<std::endl;
        ROS_WARN("----------------------> Moving Left");
        
        uav_current_pose.pose.position.x -= 1.0;
        moveUAVwithFOV(-1.0, 0.0, 0.0, false);// Supposing the uav is heading in x-axis, TODO: could be both, we need the orientation ...

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        path_pub_.publish(uav_path);

        updateEnv();
        broadcastUAVtf(tf_broadcaster);

    } else if (joy_msg->buttons[1] == 1)
    {
        std::cout<<std::endl;
        ROS_WARN("----------------------> Moving Right");
        
        uav_current_pose.pose.position.x += 1.0;
        moveUAVwithFOV(1.0, 0.0, 0.0, false);

        // Add pose to path
        uav_path.header.frame_id = "map";
        uav_path.header.stamp = ros::Time::now();
        uav_path.poses.push_back(uav_current_pose);
        path_pub_.publish(uav_path);

        updateEnv();
        broadcastUAVtf(tf_broadcaster);
    }
    
    // TODO: implement turbo
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "update_map_node");
    ros::NodeHandle node_;
    ROS_INFO("UpdateMapNode is spinning...");

    // std::string filename2 = "/home/uas/catkin_ws/src/update_map/uav_voxel.bt";
    std::string filename2 = "/home/uas/catkin_ws/src/update_map/5x5_uav.bt";
    my_uav = std::make_shared<octomap::OcTree>(filename2);

    // ROS Communication/visualization
    marker_pub_ = node_.advertise<visualization_msgs::Marker>("ray_markers", 10);
    query_pose_= node_.advertise<visualization_msgs::Marker>("query", 10);

    octomap_pub_ = node_.advertise<octomap_msgs::Octomap>("uav_fov", 10);
    explored_pub_ = node_.advertise<octomap_msgs::Octomap>("explored", 10);
    path_pub_ = node_.advertise<nav_msgs::Path>("uav_path", 10);

    joy_sub_ = node_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    // Services
    // Params
    
    // TODO: Free space
    // delete my_map;

    ros::spin();
    return 0;
}