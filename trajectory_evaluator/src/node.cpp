#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "math.h"
#include <stdexcept>
#include "trajectory_evaluator/dtw.h"
#include "trajectory_evaluator/xtf.h"
#include "trajectory_evaluator/evaluator.h"
#include "trajectory_evaluator/EvaluateSingle.h"
#include "trajectory_evaluator/EvaluatePair.h"
#include "trajectory_evaluator/SetLibraryPath.h"
#include "trajectory_evaluator/SetPairedLibraryPath.h"
#include <ros/ros.h>

XTF::Parser traj_parser;
EVAL::PoseEvaluator traj_evaluator;
EVAL::PairedPoseEvaluator paired_evaluator;

double euclidean_distance(std::vector<double> P1, std::vector<double> P2)
{
    double total = 0.0;
    for (unsigned int i = 0; i < P1.size(); i++)
    {
        total = total + pow((P1[i] - P2[i]), 2);
    }
    return sqrt(total);
}

int omp_test(void)
{
    int th_id, nthreads;
    #pragma omp parallel private(th_id)
    {
        th_id = omp_get_thread_num();
        #pragma omp barrier
        if (th_id == 0)
        {
            nthreads = omp_get_num_threads();
        }
    }
    return nthreads;
}

bool EvaluateSingleTrajectory(trajectory_evaluator::EvaluateSingle::Request &req, trajectory_evaluator::EvaluateSingle::Response &res)
{
    XTF::Trajectory traj = traj_parser.ParseTraj(req.trajectory_file);
    EVAL::EvaluationResult match = traj_evaluator.EvaluateAgainstLibrary(traj, req.field_range, req.field);
    res.matching_uid = match.uid;
    res.match_cost = match.cost;
    res.matching_tags = match.tags;
    return true;
}

bool EvaluateTrajectoryPair(trajectory_evaluator::EvaluatePair::Request &req, trajectory_evaluator::EvaluatePair::Response &res)
{
    XTF::Trajectory traj1 = traj_parser.ParseTraj(req.trajectory_file1);
    XTF::Trajectory traj2 = traj_parser.ParseTraj(req.trajectory_file2);
    EVAL::EvaluationResult match1 = traj_evaluator.EvaluateAgainstLibrary(traj1, req.field_range, req.field);
    EVAL::EvaluationResult match2 = traj_evaluator.EvaluateAgainstLibrary(traj2, req.field_range, req.field);
    res.matching_uid1 = match1.uid;
    res.matching_uid2 = match2.uid;
    res.match_cost1 = match1.cost;
    res.match_cost2 = match2.cost;
    res.matching_tags1 = match1.tags;
    res.matching_tags2 = match2.tags;
    return true;
}

bool SetEvaluatorLibrary(trajectory_evaluator::SetLibraryPath::Request &req, trajectory_evaluator::SetLibraryPath::Response &res)
{
    try
    {
        EVAL::PoseEvaluator new_evaler(req.library_path);
        traj_evaluator = new_evaler;
        res.result = "Loaded library at: " + req.library_path;
    }
    catch (...)
    {
        res.result = "Unable to load library";
    }
    return true;
}

bool SetPairedLibrary(trajectory_evaluator::SetPairedLibraryPath::Request &req, trajectory_evaluator::SetPairedLibraryPath::Response &res)
{
    try
    {
        EVAL::PairedPoseEvaluator new_evaler(req.library_path1, req.library_path2);
        paired_evaluator = new_evaler;
        res.result = "Loaded library at: " + req.library_path1 + " + " + req.library_path2;
    }
    catch (...)
    {
        res.result = "Unable to load library";
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_evaluator");
    ros::NodeHandle nh;
    ROS_INFO("Launching trajectory evaluator with %d threads reported by OpenMP", omp_test());
    ros::ServiceServer sl = nh.advertiseService("trajectory_evaluator/SetLibrary", SetEvaluatorLibrary);
    ros::ServiceServer spl = nh.advertiseService("trajectory_evaluator/SetPairedLibrary", SetPairedLibrary);
    ROS_INFO("Loaded config services");
    ros::ServiceServer et = nh.advertiseService("trajectory_evaluator/EvaluateTrajectory", EvaluateSingleTrajectory);
    ros::ServiceServer ept = nh.advertiseService("trajectory_evaluator/EvaluateTrajectoryPair", EvaluateTrajectoryPair);
    ROS_INFO("Loaded trajectory evaluation services");
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}


