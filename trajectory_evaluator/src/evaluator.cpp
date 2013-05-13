#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <dirent.h>
#include "math.h"
#include <stdexcept>
#include <Eigen/Geometry>
#include <Eigen/LU>
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

std::vector<XTF::Trajectory> EVAL::LoadTrajectoryLibrary(std::string library_path)
{
    std::vector<XTF::Trajectory> library;
    std::vector<std::string> file_names;
    XTF::Parser library_parser;
    // Populate filenames with all relevant files (.xtf or .xml) in the provided directory
    struct dirent* de = NULL;
    DIR* d = NULL;
    d = opendir(library_path.c_str());
    if (d == NULL)
    {
        throw std::invalid_argument("Unable to open library directory");
    }
    while ((de = readdir(d)) != NULL)
    {
        std::string filename(de->d_name);
        if (filename.find(".xml") != std::string::npos)
        {
            file_names.push_back(filename);
        }
        else if (filename.find(".xtf") != std::string::npos)
        {
            file_names.push_back(filename);
        }
    }
    closedir(d);
    // Try to read in each of the files into the library
    for (unsigned int i = 0; i < file_names.size(); i++)
    {
        try
        {
            XTF::Trajectory new_traj = library_parser.ParseTraj(file_names[i]);
            library.push_back(new_traj);
        }
        catch (...)
        {
            std::cout << "Could not load file: " << file_names[i] << " into the library.\nIs it an XTF trajectory file?" << std::endl;
        }
    }
    return library;
}

std::vector< std::vector<double> > EVAL::ExtractFromTrajectory(XTF::Trajectory traj, std::string field_range, std::string field)
{
    std::vector< std::vector<double> > raw_data;
    for (unsigned int i = 0; i < traj.trajectory.size(); i++)
    {
        XTF::State current = traj.trajectory[i];
        if (field_range.compare("desired") == 0)
        {
            if (field.compare("position") == 0)
            {
                if (current.position_desired.size() > 0)
                {
                    raw_data.push_back(current.position_desired);
                }
            }
            else if (field.compare("velocity") == 0)
            {
                if (current.velocity_desired.size() > 0)
                {
                    raw_data.push_back(current.velocity_desired);
                }
            }
            else if (field.compare("acceleration") == 0)
            {
                if (current.acceleration_desired.size() > 0)
                {
                    raw_data.push_back(current.acceleration_desired);
                }
            }
        }
        else if (field_range.compare("actual") == 0)
        {
            if (field.compare("position") == 0)
            {
                if (current.position_actual.size() > 0)
                {
                    raw_data.push_back(current.position_actual);
                }
            }
            else if (field.compare("velocity") == 0)
            {
                if (current.velocity_actual.size() > 0)
                {
                    raw_data.push_back(current.velocity_actual);
                }
            }
            else if (field.compare("acceleration") == 0)
            {
                if (current.acceleration_actual.size() > 0)
                {
                    raw_data.push_back(current.acceleration_actual);
                }
            }
        }
    }
    return raw_data;
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
    struct timespec start, end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    std::vector<double> costs;
    double min_cost = INFINITY;
    unsigned int min_index = -1;
    std::vector< std::vector<double> > raw_trajectory = ExtractFromTrajectory(new_trajectory, field_range, field);
    for (unsigned int i = 0; i < library.size(); i++)
    {
        std::vector< std::vector<double> > raw_library = ExtractFromTrajectory(library[i], field_range, field);
        double returned_cost = evaluator.EvaluateCost(raw_trajectory, raw_library);
        costs.push_back(returned_cost);
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
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);
    float elapsed = (float)(end.tv_sec - start.tv_sec);
    elapsed = elapsed + (float)(end.tv_nsec - start.tv_nsec) / 1000000000.0;
    std::cout << "Evaluation against the library took " << elapsed << " seconds" << std::endl;
    return return_keys;
}

PoseNormalizer::PoseNormalizer(std::vector<double> reference_pose)
{
    Eigen::Quaternion<double> orientation(reference_pose[6], reference_pose[3], reference_pose[4], reference_pose[5]);
    reference.block<3,3>(0,0) = orientation.toRotationMatrix();
    reference(0,3) = reference_pose[0];
    reference(1,3) = reference_pose[1];
    reference(2,3) = reference_pose[2];
    reference(3,3) = 1.0;
}

inline std::vector<double> PoseNormalizer::Normalize(std::vector<double> raw_pose)
{
    // Convert pose to matrix form
    Eigen::Matrix4d raw;
    Eigen::Quaternion<double> orientation(raw_pose[6], raw_pose[3], raw_pose[4], raw_pose[5]);
    raw.block<3,3>(0,0) = orientation.toRotationMatrix();
    raw(0,3) = raw_pose[0];
    raw(1,3) = raw_pose[1];
    raw(2,3) = raw_pose[2];
    raw(3,3) = 1.0;
    // Do math
    Eigen::Matrix4d normal = raw.inverse() * reference;
    Eigen::Quaternion<double> normalrot(normal.block<3,3>(0,0));
    // Return pose in array form
    std::vector<double> normalized;
    normalized.resize(7);
    normalized[0] = normal(0,3);
    normalized[1] = normal(1,3);
    normalized[2] = normal(2,3);
    normalized[3] = normalrot.x();
    normalized[4] = normalrot.y();
    normalized[5] = normalrot.z();
    normalized[6] = normalrot.w();
    return normalized;
}

