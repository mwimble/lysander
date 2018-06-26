#include <ros/ros.h>

#include <lysander/tablebot_strategy.h>

int
main(int argc, char **argv) {
  ros::init(argc, argv, "tablebot_node");

  lysander::TablebotStrategy tablebotStrategy;

  tablebotStrategy.solveChallenge1();

  return 0;
}