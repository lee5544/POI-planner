#include "SFCPlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SFCPlanner");
    ros::NodeHandle node("~");

    std::string filename;
    node.param<std::string>("paramfile/path", filename, "./src/sfc_manager/config/sfc.yaml");

    SFCPlanner planner;
    planner.init(filename, node);

    ros::spin();
    return 0;
}