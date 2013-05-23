#include <hubo_msgs/JointCommandAction.h>
#include <hubo_msgs/JointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ach.h>
#include <motion-trajectory.h>

typedef actionlib::SimpleActionServer<hubo_msgs::JointTrajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionServer<hubo_msgs::JointCommandAction> CommandServer;


ach_channel_t chan_traj_cmd;
ach_channel_t chan_traj_state;

/* chopping the trajectory into multiple segments */
bool chopTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>& waypoints,
            std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > chopped)
{
  int numSegments = waypoints.size() / MAX_TRAJ_SIZE;
  std::vector<trajectory_msgs::JointTrajectoryPoint> trajSeg;
  trajSeg.clear();
  for(size_t i = 0; i < waypoints.size(); ++i)
  {
      trajSeg.push_back(waypoints[i]);
      if( (i > 0 && (i % MAX_TRAJ_SIZE) == 0) || (i == waypoints.size() - 1) )
      {
          chopped.push_back(trajSeg);
          trajSeg.clear();
      }
  }
  return true;
}

void execute(const hubo_msgs::JointTrajectoryGoalConstPtr& goal, Server* as)
{
  printf("entering\n");
  trajectory_msgs::JointTrajectory trajectory = goal->trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> waypoints = trajectory.points;

  std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > chopped;

  chopTrajectory(waypoints, chopped);

  hubo_traj_t ach_traj;
  memset( &ach_traj, 0, sizeof(ach_traj) );

  hubo_traj_output ach_state;
  memset( &ach_state, 0, sizeof(ach_state) );
  size_t fs;
  ach_get( &chan_traj_state, &ach_state, sizeof(ach_state), &fs, NULL, ACH_O_LAST );


  /* // Iterate through the chopped trajectories
    for( int c = 0; c < chopped.size(); c++ ) // Might be better to use an iterator instead of an int
    {

        //TODO: Define how many waypoints you will be using
      int num_waypoints = XXXX; // = chopped[c].size()?

      //feed the trajectory data into A structure
      // TODO: Fill in these fields with the appropriate values
      for(int j=0; j < num_waypoints; j++)
      {
          for(int i=0; i < HUBO_JOINT_COUNT; i++)
          {
            ach_traj.joint[i].position[j] = XXXX; // position of joint i at waypoint j
            ach_traj.joint[i].velocity[j] = XXXX; // velocity of joint i at waypoint j
            ach_traj.joint[i].acceleration[j] = XXXX; // acceleration of joint i at waypoint j
          }
          ach_traj.time[j] = XXXX; // the time at which the trajectory passes through waypoint j
      }
      ach_traj.endTime = XXXX; // the length of time that this chunk of trajectory should take to run
      ach_traj.trajID = c; // integer value to indentify this trajectory

      do {
        struct timespec t; // Wait for at most 1 second
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        ach_get( &chan_traj_state, &ach_state, sizeof(ach_state), &fs, &t, ACH_O_WAIT | ACH_O_LAST );
      } while( ach_state.trajID < c-1 && ach_state.status == TRAJ_RUNNING ); // Make sure we don't flood the ach channel with too many chunks

      ach_put( &chan_traj_cmd, &ach_traj, sizeof(ach_traj) );

    }
  */


  // TODO: Parse the ach_state fields to provide a response for the service call (probably not important)

  printf("print %s", trajectory.header.frame_id.c_str());
  as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hubo_trajectory_controller");
    ros::NodeHandle nh;
    // Set up the ACH channels to and from hubo-motion-rt
    char command[100];
    sprintf(command, "ach -1 -C %s -m 10 -n 1000000 -o 666", HUBO_TRAJ_CHAN);
    system(command);
    sprintf(command, "ach -1 -C %s -o 666", HUBO_TRAJ_STATE_CHAN);
    system(command);
    // Make sure the ACH channels are opened properly
    ach_status_t r = ach_open(&chan_traj_cmd, HUBO_TRAJ_CHAN, NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: HUBO_TRAJ_CHAN !");
        ros::shutdown();
    }
    r = ach_open(&chan_traj_state, HUBO_TRAJ_STATE_CHAN, NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: HUBO_TRAJ_STATE_CHAN !");
        ros::shutdown();
    }
    // Once everything on the ACH side is loaded, load the two action servers

  Server server(n, "ach_server", boost::bind(&execute, _1, &server), false);
  printf("start to wait\n");
  server.start();
  ros::spin();
  return 0;
}
