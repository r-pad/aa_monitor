/**
 * @author: edwardahn
 */

#include "ros/ros.h"

#include "aa_monitor/utils/utils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aa_monitor");
    ros::NodeHandle n;
    printf("dummy function returned %d", dummyFunction());
    ros::spin();
    return 0;
}
