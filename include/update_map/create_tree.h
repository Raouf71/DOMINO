#ifndef UPDATE_MAP_CREATE_TREE_H
#define UPDATE_MAP_CREATE_TREE_H

#include <iostream>
#include <vector>
 

class TreeNode {

public:
    int id;
    double x, y, z;
    std::vector<TreeNode*> children;
 
    TreeNode(int nodeID, double xCoord, double yCoord, double zCoord);
 
    void addChild(TreeNode* child); 


};

#endif