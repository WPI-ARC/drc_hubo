#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>
#include <sstream>
#include "hubo_ros/HuboState.h"
#include "hubo_ros/HuboJointState.h"

#define EXTRA_TIME 30

bool bagging = false;
double end = 0;
int initialized = 0;
hubo_ros::HuboState prevState;
rosbag::Bag bag;

void state_cb(hubo_ros::HuboState myState){

    //Get the time of the message
    ros::Time bagTime = ros::Time::now();
    double msgTime =ros::Time::now().toSec();   


    //Save the state as the prev state
    if (initialized == 0){
        prevState = myState;
        initialized = 1;
    }


    //Compare all the commands to determine if htey changed
    for(int i = 0; i < myState.joints.size(); i++){
    
        //Check if any of the commands changed
        if(myState.joints[i].commanded != prevState.joints[i].commanded){

            //Set up appro. bagging parameters
            i = myState.joints.size();

            //If we are bagging, update time
            if (bagging == true){
            
                //Set the endtime
                end = msgTime + EXTRA_TIME;
                ROS_INFO("***      END = %f", end);
    
            }

            //If we are not bagging, set to bagging and update time
            else {
                
                //Create a bag
                std::stringstream name;
                name << "Hubo_Command_Bag_" << (int)msgTime;
                std::string bagName = name.str();
                bag.open(bagName, rosbag::bagmode::Write);
                ROS_INFO("*** Starting New Bag %f ***", msgTime);
                ROS_INFO("***      END = %f", end);
        
                //Start Bagging
                bagging = true;

                //Set the endtime
                end = msgTime + EXTRA_TIME;
                
            }
        }

    }


    //Bag if time hasn't run out
    if (end >= ros::Time::now().toSec() && bagging) {
       bag.write("Hubo/HuboState", bagTime, myState);
       ROS_INFO("***      ADDING NEW MSG");
    }
    else if (end < ros::Time::now().toSec() && bagging){
        bag.close();
        ROS_INFO("*** Bag Closed ***");
        bagging = false;
    }
}

int main (int argc, char** argv) {

  // Initialize ROS and the NodeHandle
  ros::init (argc, argv, "hubo_command_autobagger");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_state   = nh.subscribe ("Hubo/HuboState", 100, state_cb);
  
  ROS_INFO("Ready to Proceed Captain");

  // Spin
  ros::spin ();

  return 0;
}
