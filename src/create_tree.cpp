#include <update_map/create_tree.h>


TreeNode::TreeNode(int nodeID, double xCoord, double yCoord, double zCoord)
    : id(nodeID), x(xCoord), y(yCoord), z(zCoord) {}


void TreeNode::addChild(TreeNode * child)
    {
        children.push_back(child);
    }




// int main() {

// void createTree() {

/*
    // Create FOV ray tree 
    TreeNode* root = new TreeNode(0, 0.0, 0.0, 0.5);    // root ID = 0
    
    TreeNode* child1 = new TreeNode(1, -0.5, 0.5, 0.5); // Ray 1
    TreeNode* child2 = new TreeNode(2, -1.5, 0.5, 0.5);
    TreeNode* child3 = new TreeNode(3, -2.5, 0.5, 0.5);

    root->addChild(child1);
    child1->addChild(child2);
    child2->addChild(child3);
 
    TreeNode* child4 = new TreeNode(4, -1.5, 1.5, 0.5); // Ray 2
    TreeNode* child5 = new TreeNode(5, -2.5, 1.5, 0.5);

    child1->addChild(child4);
    child4->addChild(child5);

    TreeNode* child6 = new TreeNode(6, -0.5, 1.5, 0.5); // Ray 3
    TreeNode* child7 = new TreeNode(7, -1.5, 2.5, 0.5);

    root->addChild(child6);
    child6->addChild(child7);

    TreeNode* child8 = new TreeNode(8, 0.5, 1.5, 0.5);  // Ray 4
    TreeNode* child9 = new TreeNode(9, -0.5, 2.5, 0.5);
    TreeNode* child10 = new TreeNode(10, -0.5, 3.5, 0.5);

    root->addChild(child8);
    child8->addChild(child9);
    child9->addChild(child10);

    TreeNode* child11 = new TreeNode(11, 0.5, 2.5, 0.5); // Ray 5    
    TreeNode* child12 = new TreeNode(12, 0.0, 3.5, 0.5);

    child8->addChild(child11);
    child11->addChild(child12);

    TreeNode* child13 = new TreeNode(13, 1.5, 2.5, 0.5); // Ray 6
    TreeNode* child14 = new TreeNode(14, 1.5, 3.5, 0.5); 

    child11->addChild(child13);
    child13->addChild(child14);

    TreeNode* child15 = new TreeNode(15, 1.5, 1.5, 0.5); // Ray 7
    TreeNode* child16 = new TreeNode(16, 2.5, 2.5, 0.5); 

    root->addChild(child15);
    child15->addChild(child16);

    TreeNode* child17 = new TreeNode(17, 1.5, 0.5, 0.5); // Ray 8
    TreeNode* child18 = new TreeNode(18, 2.5, 1.5, 0.5); 
    TreeNode* child19 = new TreeNode(19, 3.5, 1.5, 0.5); 

    root->addChild(child17);
    child17->addChild(child18);
    child18->addChild(child19);

    TreeNode* child20 = new TreeNode(20, 2.5, 0.5, 0.5); // Ray 9
    TreeNode* child21 = new TreeNode(21, 3.5, 0.5, 0.5); 

    child17->addChild(child20);
    child20->addChild(child21);

    // printTree(root);
*/
                // keyPoint.y() += drone_y;

// }

//     // Clean up allocated memory
//     // delete root;
//     // delete child1;
//     // delete child2;
//     // delete child3;
//     // delete child4;
//     // delete child5;
//     // delete child6;
//     // delete child7;
//     // delete child8;
//     // delete child9;
//     // delete child10;
//     // delete child11;
//     // delete child12;
 
//     return 0;
// }

    
