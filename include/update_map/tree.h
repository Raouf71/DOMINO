#ifndef UPDATE_MAP_TREE_H
#define UPDATE_MAP_TREE_H

#include <iostream>
#include <vector>

class TreeNode {
public:
    int data;
    std::vector<TreeNode*> children;

    TreeNode(int val);
};

class Tree {
public:
    TreeNode* root;

    Tree();
    void addChild(TreeNode* parent, int childData);
    void printTree(TreeNode* node, int depth = 0);
};

#endif
