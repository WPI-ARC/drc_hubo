#include <hubo_msgs/JointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionServer<hubo_msgs::JointTrajectoryAction> Server;

#define MAX_TRAJ_SIZE 200

void execute(const hubo_msgs::JointTrajectoryGoalConstPtr& goal, Server* as)
{
  printf("entering\n");
  trajectory_msgs::JointTrajectory trajectory = goal->trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> waypoints = trajectory.points;
  //feed the trajectory data into ACH structure
  printf("print %s", trajectory.header.frame_id.c_str());
  as->setSucceeded();
}

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ach_server_node");
  ros::NodeHandle n;
  Server server(n, "ach_server", boost::bind(&execute, _1, &server), false);
  printf("start to wait\n");
  server.start();
  ros::spin();
  return 0;
}
