/*
 * dbow_driver.cpp
 *
 * Written by: Stylianos Piperakis.
 *
 * This file launches the bag of words ros wrapper
 */
#include <dbow_ros/dbow.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dbow_ros");
    ros::NodeHandle n;
    dbow db(n);
    ros::spin();
    return 0;
}