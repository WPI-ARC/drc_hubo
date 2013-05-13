#include "trajectory_evaluator/xtf.h"
#include <exception>

int main(int argc, char** argv)
{
    try
    {
        std::cout << "Making reader..." << std::endl;
        XTF::Parser parser;
        std::cout << "...done" << std::endl;
        std::cout << "Reading reference pose trajectory file..." << std::endl;
        XTF::Trajectory my_traj = parser.ParseTraj("pose_recorded.xtf");
        std::cout << "...done" << std::endl;
        std::cout << "Printing trajectory to the screen..." << std::endl;
        std::cout << my_traj << std::endl;
        std::cout << "...done" << std::endl;
        std::cout << "Saving pose example..." << std::endl;
        parser.ExportTraj(my_traj, "pose_recorded_copy.xtf", false);
        std::cout << "...done" << std::endl;
        std::cout << "Reading copy back in..." << std::endl;
        XTF::Trajectory my_traj_copy = parser.ParseTraj("pose_recorded_copy.xtf");
        std::cout << "...done" << std::endl;
        std::cout << "Printing copy to the screen..." << std::endl;
        std::cout << my_traj_copy << std::endl;
        std::cout << "...done" << std::endl;
        std::cout << "ALL PASS --- exiting..." << std::endl;
    }
    catch (const char* msg)
    {
        std::cout << "EXCEPTION: " << msg << " !" << std::endl;
    }
    return 0;
}
