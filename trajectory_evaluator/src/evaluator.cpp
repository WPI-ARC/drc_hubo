#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <dirent.h>
#include "math.h"
#include <stdexcept>
#include "trajectory_evaluator/xtf.h"
#include "trajectory_evaluator/evaluator.h"

using namespace EVAL;

double pose_distance(std::vector<double> p1, std::vector<double> p2)
{
    if (p1.size() == 7 && p2.size() == 7)
    {
        double position_dist = sqrt(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2) + pow((p1[2] - p2[2]), 2));
        double dot_product = (p1[3] * p2[3]) + (p1[4] * p2[4]) + (p1[5] * p2[5]) + (p1[6] * p2[6]);
        double orientation_dist = 0.0;
        if (dot_product < 0.99999999)
        {
            orientation_dist = acos(2 * pow(dot_product, 2) - 1);
        }
        return position_dist + orientation_dist;
    }
    else
    {
        throw std::invalid_argument("Pose_Distance requires a pose to have 7 elements");
    }
}

PoseEvaluator::PoseEvaluator(std::string library_root)
{
    library = LoadTrajectoryLibrary(library_root);
    unsigned int max_length_in_library = 0;
    for (unsigned int i = 0; i < library.size(); i++)
    {
        unsigned int new_size = library[i].trajectory.size();
        if (new_size > max_length_in_library)
        {
            max_length_in_library = new_size;
        }
    }
    DTW::SimpleDTW new_eval(max_length_in_library, max_length_in_library, pose_distance);
    evaluator = new_eval;
}

std::vector<std::string> PoseEvaluator::EvaluateAgainstLibrary(XTF::Trajectory new_trajectory, std::string field_range, std::string field)
{
    std::vector<double> costs;
    double min_cost = INFINITY;
    unsigned int min_index = -1;
    std::vector< std::vector<double> > raw_trajectory = ExtractFromTrajectory(new_trajectory, field_range, field);
    for (unsigned int i = 0; i < library.size(); i++)
    {
        std::vector< std::vector<double> > raw_library = ExtractFromTrajectory(library[i], field_range, field);
        double returned_cost = evaluator.EvaluateCost(raw_trajectory, raw_library);
        if (returned_cost < min_cost)
        {
            min_cost = returned_cost;
            min_index = i;
        }
    }
    if (costs.size() == 0)
    {
        throw std::invalid_argument("Trajectory library is empty");
    }
    std::vector<std::string> return_keys;
    // Assemble return values
    // First element is the UID of the lowest-cost trajectory
    std::string uid = library[min_index].uid;
    return_keys.push_back(uid);
    // Remaining elements are the tags of the lowest-cost trajectory
    for (unsigned int i = 0; i < library[min_index].tags.size(); i++)
    {
        return_keys.push_back(library[min_index].tags[i]);
    }
    return return_keys;
}

PoseNormalizer::PoseNormalizer()
{
    ;
}

std::vector<double> PoseNormalizer::Normalize(std::vector<double> raw_pose)
{
    ;
}

