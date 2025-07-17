
#include "MissionXYZ.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "missionxyz");
    ros::NodeHandle nh("~");

    MissionXYZ mission;
    mission.init(nh);

    ros::Duration(0.5).sleep();
    ROS_INFO("[mission xyz]: ready.");
    ros::spin();

    return 0;
}
