#include "SFCPlannerLidar.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SFCPlannerLidar");
    ros::NodeHandle node("~");

    std::string filename;
    node.param<std::string>("paramfile/path", filename, "./src/sfc_manager/config/sfc_lidar.yaml");

    SFCPlannerLidar planner;
    planner.init(filename, node);

    ros::spin();
    return 0;
}