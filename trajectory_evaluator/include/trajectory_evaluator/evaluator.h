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

#ifndef EVALUATOR_H
#define EVALUATOR_H

namespace EVAL
{

std::vector<XTF::Trajectory> LoadTrajectoryLibrary(std::string library_path)
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

std::vector< std::vector<double> > ExtractFromTrajectory(XTF::Trajectory traj, std::string field_range, std::string field)
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

public:

    PoseNormalizer();

    ~PoseNormalizer()
    {
    }

    inline std::vector<double> Normalize(std::vector<double> raw_pose);

};

}

#endif // EVALUATOR_H
