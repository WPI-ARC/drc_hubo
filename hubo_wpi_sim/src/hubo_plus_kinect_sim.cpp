#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define KINECT_POSE_SET_RATE 10

/** Creates and handles a simulated kinect in gazebo so that sensor data for the hubo+ can be spoofed.
 *
 * This node will perform a variety of functions:
 * [X] (1a) Use the generic_tf_broadcaster to create a fixed transform between a link on the robot and the kinect's position
 * [ ] (1b) Create a pose that represents the kinect's pose that we want in the world
 * [ ] (2)  Use that transform to set the location of the kinect in Gazebo
 * [ ] (A)  Create a new sensor_pkg launch file that has the new pointcloud topic specified.
 *
 * Discussion:
 * The pose of the hubo's head can either be specifically published from the tab in GRIP or based off the TF tree that
 * is being published for the system.  CHECK WITH CALDER THAT TF TREE WILL BE PUBLISHED. CONFIRMED.

 * Fixed frame broadcast example in the ARC-Utils repo there is a generic_tf_broadcaster and all that we have to do is set up the launch file
 * and it will publish a fixed TF frame.
 */

//Create a TransformListener
tf::TransformListener* myListener = NULL;

//Frames that we will need for this work
std::string kinect_frame, world_origin_frame;


void set_kinect_pose (void){

    //Check if the transform has changed and if so get the new position
    myListener->waitForTransform((std::string)pointcloud_frame, (std::string)target_frame, req.valve.header.stamp, ros::Duration(1));

    //Use that transform to find the kinects position in the world frame

    //Publish that position to gazebo

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "hubo_plus_sim_connect");

  //Setup everything for the publishes
  ros::NodeHandle nh;

  //Create a TransformListener
  myListener = new(tf::TransformListener);

  //Get the world frame that we are using as an origin
  ros::param::get("hubo_plus_world_frame", world_origin_frame);

  //Get the position of the kinect from the TF tree
  ros::param::get("/hubo_plus_kinect_sim_frame", kinect_frame);

  //Find the transform between the world origin that we want and the kinect frame
  bool found = false;
  while (found == false) {
    found = myListener->waitForTransform((std::string)world_origin_frame, (std::string)kinect_frame, ros::Time(), ros::Duration(1));
  }

  //Set up something to publish the kinect's position
  ros::Rate r(KINECT_POSE_SET_RATE);

  //Continue until ROS stops
  while(ros::ok()) {

      //Set the pose
      set_kinect_pose();

      //Uncomment this line if we add any subscribers that need to be checked
      //ros::spinOnce();

      //Sleep for the remainder
      r.sleep();

  }

  //Spin Forever
  ros::spin();

  return 0;

}
