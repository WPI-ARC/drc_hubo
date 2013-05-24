#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

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

//Create a new client that will set the position of the object in gazebo
ros::ServiceClient client;
gazebo_msgs::SetModelState srv;

void set_kinect_pose (void){

    ros::Time time = ros::Time::now();

    //Check if the transform has changed and if so get the new position
    myListener->waitForTransform(world_origin_frame, kinect_frame, time, ros::Duration(1));

    //Use that transform to find the kinects position in the world frame
    geometry_msgs::PoseStamped kinect_pose;
    kinect_pose.header.frame_id = kinect_frame;
    kinect_pose.header.stamp = time;
    kinect_pose.pose.orientation.w = 1;

    myListener->transformPose(world_origin_frame, kinect_pose, kinect_pose);

    ROS_INFO("(%f, %f, %f), (%f, %f, %f, %f)", kinect_pose.pose.position.x,
                                               kinect_pose.pose.position.y,
                                               kinect_pose.pose.position.z,
                                               kinect_pose.pose.orientation.x,
                                               kinect_pose.pose.orientation.y,
                                               kinect_pose.pose.orientation.z,
                                               kinect_pose.pose.orientation.w);

    //Publish that position to gazebo
    srv.request.model_state.model_name = "floating_kinect";
    srv.request.model_state.pose = kinect_pose.pose;

    client.call(srv);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "hubo_plus_kinect_sim");

  //Setup everything for the publishes
  ros::NodeHandle nh;

  //Create a TransformListener
  myListener = new(tf::TransformListener);

  //Create the ros service
  client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  world_origin_frame = "/Body_RAR";
  kinect_frame = "/kinect_sim_link";

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
