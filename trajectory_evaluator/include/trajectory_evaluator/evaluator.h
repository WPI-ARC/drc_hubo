#include "stdlib.h"
#include "stdio.h"
#include <vector>
#include <string>
#include <sstream>
#include "string.h"
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <dirent.h>
#include "trajectory_evaluator/dtw.h"
#include "trajectory_evaluator/xtf.h"
#include <Eigen/Geometry>
#include <Eigen/LU>

#ifndef EVALUATOR_H
#define EVALUATOR_H

namespace EVAL
{

std::vector<XTF::Trajectory> LoadTrajectoryLibrary(std::string library_path);

std::vector< std::vector<double> > ExtractFromTrajectory(XTF::Trajectory traj, std::string field_range, std::string field);

class PoseEvaluator
{
protected:

    DTW::SimpleDTW evaluator;
    std::vector<XTF::Trajectory> library;

public:

    PoseEvaluator(std::string library_root);

    ~PoseEvaluator()
    {
    }

    std::vector<std::string> EvaluateAgainstLibrary(XTF::Trajectory new_trajectory, std::string field_range, std::string field);

};

class PoseNormalizer
{
protected:

    Eigen::Matrix4d reference;

public:

    /*
      The pose normalizer operates on poses of the form (raw_pose) X (offset_pose) = (reference_pose),
      which allows all raw_pose values to be converted to a normalized offset pose from the provided
      reference pose. This is used for pose trajectory evaluation, where it normalizes end-effector
      pose trajectories relative to the robot's kinematic chain so that they are relative to the pose
      of the end effector task. This attempts to make trajectories normalized to the task even if they
      take place at wildly varying locations with respect to the robot itself.

      reference_pose is of the form [x, y, z, x_angle, y_angle, z_angle, w_angle]
    */
    PoseNormalizer(std::vector<double> reference_pose);

    ~PoseNormalizer()
    {
    }

    /*
      Returns the result of (offset_pose) = (raw_pose)^(-1) X (reference_pose)

      raw_pose is of the form [x, y, z, x_angle, y_angle, z_angle, w_angle]
    */
    inline std::vector<double> Normalize(std::vector<double> raw_pose);

};

}

#endif // EVALUATOR_H
