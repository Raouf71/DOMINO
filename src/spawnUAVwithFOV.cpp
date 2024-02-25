void publishPoints(std::vector<geometry_msgs::Point> &queries) {
    
    // Points around FOV
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < queries.size(); ++i) {
        visualization_msgs::Marker marker3;
        marker3.header.frame_id = "map";
        marker3.header.stamp = ros::Time::now();
        marker3.ns = "uav_fov";
        marker3.id = static_cast<int>(i)+1; 
        marker3.type = visualization_msgs::Marker::SPHERE; 
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.scale.x = 0.1;
        marker3.scale.y = 0.1;
        marker3.scale.z = 0.1;
        marker3.color.r = 1;
        marker3.color.g = 0.0;
        marker3.color.b = 0.0;
        marker3.color.a = 1.0;
        marker3.lifetime = ros::Duration(); // Persistent marker

 /*      
        // Old FOV
        // // left side
        // point.x = -1.5;
        // point.y = 0.5;
        // point.z = 0.5;
        // uav_fov.push_back(point);
        // point.y = -0.5;
        // uav_fov.push_back(point);
        // point.y = -1.5;
        // uav_fov.push_back(point);
        // point.y = 1.5;
        // uav_fov.push_back(point);

        // // right side
        // point.x = 1.5;
        // point.y = 0.5;
        // point.z = 0.5;
        // uav_fov.push_back(point);
        // point.y = -0.5;
        // uav_fov.push_back(point);
        // point.y = 1.5;
        // uav_fov.push_back(point);
        // point.y = -1.5;
        // uav_fov.push_back(point);

        // // top
        // point.x = 0.5;
        // point.y = 1.5;
        // point.z = 0.5;
        // uav_fov.push_back(point);
        // point.x = -0.5;
        // uav_fov.push_back(point);

        // // bottom
        // point.x = 0.5;
        // point.y = -1.5;
        // point.z = 0.5;
        // uav_fov.push_back(point);
        // point.x = -0.5;
        // uav_fov.push_back(point);
*/
        
        // Publish cube markers
        marker3.pose.position = queries[i];
        marker3.pose.orientation.x = 0.0;
        marker3.pose.orientation.y = 0.0;
        marker3.pose.orientation.z = 0.0;
        marker3.pose.orientation.w = 1.0;
        marker_array.markers.push_back(marker3);
    }
    
    queries_.publish(marker_array);
    
    // Rays along FOV
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";  
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 2;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.5; 
    line_strip.scale.y = 0.5; 
    line_strip.scale.z = 0.5;  

    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration(); // Persistent marker

    for (const auto &p : queries)
    {
        line_strip.points.push_back(p);
    }

    rays_.publish(line_strip);
} 


void publishPoint(const geometry_msgs::Point &query_pose) {
    
    // Create the drone marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "query_point";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = query_pose;
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
    marker.lifetime = ros::Duration(0); //persistent marker

    query_pose_.publish(marker);
}


void publishUAVwithFOV(const geometry_msgs::Point &uav_pose, geometry_msgs::Point &uav_fov) {
    
    // Create the drone marker

    visualization_msgs::Marker marker1;
    marker1.header.frame_id = "map";
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "uav_pose";
    marker1.id = 0;
    marker1.type = visualization_msgs::Marker::CUBE;
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.pose.position = uav_pose;
    marker1.pose.orientation.x = 0.0;
    marker1.pose.orientation.y = 0.0;
    marker1.pose.orientation.z = 0.0;
    marker1.pose.orientation.w = 1.0;
    marker1.scale.x = 2;
    marker1.scale.y = 2;
    marker1.scale.z = 1; 
    marker1.color.r = 0.0;
    marker1.color.g = 0.0;
    marker1.color.b = 1.0;
    marker1.color.a = 1.0;
    marker1.lifetime = ros::Duration(); //persistent marker

    current_pose_.publish(marker1);

    // Create the FOV marker

    // TODO: Adjust FOV
 
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "map";
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "uav_fov";
    marker2.id = 1;
    marker2.type = visualization_msgs::Marker::CYLINDER;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = uav_fov.x + uav_pose.x;
    marker2.pose.position.y = uav_fov.y + uav_pose.y; 
    marker2.pose.position.z = 0.5; 
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 6.5;
    marker2.scale.y = 6.5;
    marker2.scale.z = 0.01; 
    marker2.color.r = 173.0 / 255.0;
    marker2.color.g = 216.0 / 255.0;
    marker2.color.b = 230.0 / 255.0;
    marker2.color.a = 0.1;
    marker2.lifetime = ros::Duration(); //persistent marker

    // fov_current_pose.pose.position = marker2.pose.position;
    // fov_current_pose.pose.position = marker2.pose.position;
    // fov_current_pose.pose.position = marker2.pose.position;

    current_pose_fov_.publish(marker2);
}


// TODO: change nodes order and see what comes out
// Rekursive Funktion, um durch den Baum zu iterieren
void iterateTree(TreeNode* node) {
    // Verarbeite den aktuellen Knoten
    std::cout << "Node ID: " << node->id << " - Coordinates: (" << node->x << ", " << node->y << ", " << node->z << ")\n";

    // Iteriere über alle Kinder des aktuellen Knotens
    for (TreeNode* child : node->children) {
        iterateTree(child);
    }
}


void printTree(TreeNode *root, int depth = 0)
    {

        if (!root)
        {
            return;
        }

        for (int i = 0; i < depth; ++i)
        {
            std::cout << "  "; // Indentation for better visualization
        }

        std::cout << "ID: " << root->id << ", Coordinates: (" << root->x << ", " << root->y << ", " << root->z << ")" << std::endl;

        for (TreeNode *child : root->children)
        {
            printTree(child, depth + 1);
        }
}