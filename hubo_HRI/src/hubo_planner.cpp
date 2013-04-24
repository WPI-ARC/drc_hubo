#include <ros/ros.h>
#include "hubo_srvs/LadderClimbingWalk.h"

/**
 * This function gets called whenever the user wants to send back a command to
 * the robot.  It needs to be updated to let the perception and arm_test
 * packages know that it got a command 
 */
bool ladder_climbing_walk_request(hubo_srvs::LadderClimbingWalk::Request  &req,
           	                      hubo_srvs::LadderClimbingWalk::Response &res ) {

  //Let everyone know that we got a command
  ROS_INFO("Received: %d", req.input);

  
  //PUT CODE TO RUN PLANNER HERE!


  //Respond with a 1 that we executed the command 
  res.output = 1;

  return true;
}

int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "hubo_planner");
  ros::NodeHandle nh;

  //Create a ROS Service Server
  ros::ServiceServer server = nh.advertiseService("ladder_climbing/walk", ladder_climbing_walk_request);

  // Spin
  ros::spin ();

return 0;
}
