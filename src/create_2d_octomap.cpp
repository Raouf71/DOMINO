#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/ColorOcTree.h>

#define RED_TEXT "\033[1;31m"
#define RESET_TEXT "\033[0m"

using namespace octomap;


void createUAV005(ColorOcTree& octree) {
    // 3x3
    double size_x = 0.35; 
    double resolution = 0.05;
    // Populate the OctoMap with some example occupied voxels
    for (double x = -size_x / 2.0; x < size_x / 2.0; x += resolution) {
        for (double y = -size_x / 2.0; y < size_x / 2.0; y += resolution) {
                double distance = sqrt(x * x + y * y);
                if (distance <= size_x / 2.0) {
                    octomap::point3d endpoint(x, y, 0.0);
                    octree.updateNode(endpoint, true);
                }
        }
    }
}


void create12x12hollowMap(ColorOcTree& octree) {
    for (double x = -5.5; x < 6; x += 1) {
        for (double y = -5.5; y < 6; y += 1) {
                bool occupied = false;

                // Create walls
                if (abs(x) > 5.1 || abs(y) > 5.1) {
                    occupied = true;
                }
                
                octree.updateNode(octomap::point3d(x, y, 0.0), occupied);
        }
    }

    double resolution = 1.0;
    // //Creates obstacle
    // for (double x = -1; x < 0; x += resolution) {
    //     for (double y = -1; y < 0; y += resolution) {   
    //         octree.updateNode(octomap::point3d(x, y, 0.5), true);
    //     }
    // }

    //Creates inner walls
    for (double x = -4.5; x < 5; x += 1) {
        for (double y = 6; y < 7; y += resolution) {   
            octree.updateNode(octomap::point3d(x, y, 0.5), true);
        }
    }

    for (double x = -4.5; x < 5; x += 1) {
        for (double y = -8; y < -7; y += resolution) {   
            octree.updateNode(octomap::point3d(x, y, 0.5), true);
        }
    }

//     for (double x = -4.5; x < 5; x += 1) {
//         for (double y = -4; y < -3; y += resolution) {   
//             octree.updateNode(octomap::point3d(x, y, 0.5), true);
//         }
//     }
}


void create12x12in3dMap(ColorOcTree& octree) {
    for (double x = -5.5; x < 6; x += 1) {
        for (double y = -5.5; y < 6; y += 1) {
            for (double z = -5.5; z < 6; z += 1) {
                bool occupied = false;

                // Create walls
                if (abs(x) > 5.1 || abs(y) > 5.1 || abs(z) > 5.1) {
                    occupied = true;
                }
                
                octree.updateNode(octomap::point3d(x, y, z), occupied);
        }
    }

    double resolution = 1.0;
    //Creates obstacle
    for (double x = -3; x < -2; x += resolution) {
        for (double y = -3; y < -2; y += resolution) {   
            for (double z = -3; z < -2; z += resolution) {                
                octree.updateNode(octomap::point3d(x, y, z), true);
            }
        }
    }

    }
}


void create3x3(ColorOcTree& octree, int radius) {
    for (int x = -radius; x <= radius; ++x) {
        for (int y = -radius; y <= radius; ++y) {

        octomap::ColorOcTreeNode* node = octree.updateNode(x, y, 0.5, true);
        node->setColor(0.0, 0.0, 1.0);

        }
    }
}


void createSphere3D(ColorOcTree& octree, int radius=3.5) {
    for (int x = -radius; x <= radius; ++x) {
        for (int y = -radius; y <= radius; ++y) {
            for (int z = -radius; z <= radius; ++z) {
                double distance = sqrt(x * x + y * y + z * z);
                if (fabs(distance - radius) < 0.5) {
                    octree.updateNode(point3d(x, y, z), true);
                }
            }
        }
    }
}


void createSphere2D(ColorOcTree& octree, int radius=3.5) {
    for (int x = -radius; x <= radius; x+=1) {
        for (int y = -radius; y <= radius; y+=1) {
            double distance = sqrt(x * x + y * y);
            if (distance <= radius+0.5) {
                octomap::ColorOcTreeNode* node = octree.updateNode(point3d(x, y, 0.0), true);
                node->setColor(1.0, 0.0, 0.0);

            }
        }
    }

}


int main(int argc, char** argv) {

    if (argc != 2) {
        std::cerr << RED_TEXT << "You must specify a path and name for the Octomap file as an argument!" << RESET_TEXT << std::endl;
        return 1;
    }

    float resolution = 1;  // Set desired resolution
    octomap::ColorOcTree tree(resolution);   // Create an OcTree object

    
    // create12x12hollowMap(tree);
    create12x12hollowMap(tree);
    // Update inner occupancy of the OctoMap
    tree.updateInnerOccupancy();

    // save the tree
    std::string output_filename = argv[1];
    tree.writeBinary(output_filename);

    std::cout << "Wrote " << output_filename << std::endl;

    return 0;
}
