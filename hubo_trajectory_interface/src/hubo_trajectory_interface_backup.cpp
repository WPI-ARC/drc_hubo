/*
 * JointTrajectory interface to the Hubo robot.
 *
 * This executable provides an externally-facing ActionServer interface for JointTrajectoryAction goals
 * which it connects internally to the hubo_trajectory_interface that passes trajectories to the
 * hubo-motion-rt system using ACH channels.
 *
 * Licensing:
 * -------------
 *
 * Copyright (c) 2013, Calder Phillips-Grafflin (WPI) and M.X. Grey (Georgia Tech), Drexel DARPA Robotics Challenge team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdlib.h>
#include <vector>
// System includes to handle safe shutdown
#include <signal.h>
// ROS & includes
#include <ros/ros.h>
// Boost includes
#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>
// Message and action includes for Hubo actions
#include <trajectory_msgs/JointTrajectory.h>
#include <hubo_msgs/JointTrajectoryState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// Includes for ACH and hubo-motion-rt
#include <ach.h>
#include <motion-trajectory.h>

// ACH channels
ach_channel_t chan_traj_cmd;
ach_channel_t chan_traj_state;

// Trajectory storage
std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > g_trajectory_chunks;
int g_tid = 0;

// Publisher and subscriber
ros::Publisher g_state_pub;
ros::Subscriber g_traj_sub;

// Stuff for thread synchronization
boost::signals2::mutex g_mtx;
boost::thread interface_thread;

void shutdown(int signum)
{
    if (signum == SIGINT)
    {
        ROS_INFO("Starting safe shutdown...");
        boost::lock_guard<boost::signals2::mutex> guard(g_mtx);
        g_trajectory_chunks.clear();
        ros::shutdown();
        interface_thread.join();
        ROS_INFO("Shutting down!");
    }
}

void sendTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> processed_traj)
{
    if (processed_traj.size() == 0)
    {
        return;
    }
    else
    {
        hubo_traj_t ach_traj;
        memset(&ach_traj, 0, sizeof(ach_traj));
        unsigned int points = processed_traj.size();
        for (unsigned int p = 0; p < points; p++)
        {
            for (int i = 0; i < HUBO_JOINT_COUNT; i++)
            {
                ach_traj.joint[i].position[p] = processed_traj[p].positions[i];
                ach_traj.joint[i].velocity[p] = processed_traj[p].velocities[i;]
                ach_traj.joint[i].acceleration[p] = processed_traj[p].accelerations[i];
            }
            ach_traj.time[p] = processed_traj[p].time_from_start;
        }
        ach_traj.endTime = processed_traj.back().time_from_start;
        ach_traj.trajID = g_tid;
        ach_put(&chan_traj_cmd, &ach_traj, sizeof(ach_traj));
        g_tid++;
    }
}

std::vector<trajectory_msgs::JointTrajectoryPoint> processTrajectory(hubo_traj_output* hubo_state)
{
    std::vector<trajectory_msgs::JointTrajectoryPoint> processed;
    boost::lock_guard<boost::signals2::mutex> guard(g_mtx);
    if (g_trajectory_chunks.size() == 0)
    {
        return processed;
    }
    else
    {
        for (unsigned int i = 0; i < g_trajectory_chunks[0].size(); i++)
        {
            ;
        }
        // Remove the current chunk from storage now that we've used it
        g_trajectory_chunks.erase(g_trajectory_chunks.begin(), g_trajectory_chunks.begin() + 1);
        return processed;
    }
}

void sendingLoop()
{
    // Loop until node shutdown
    while (ros::ok())
    {
        // Get the latest state from the hubo (this is used to populate the uncommanded joints!)
        hubo_traj_output hubo_state;
        memset(&hubo_state, 0, sizeof(hubo_state));
        size_t fs;
        ach_get(&chan_traj_state, &hubo_state, sizeof(hubo_state), &fs, NULL, ACH_O_WAIT);
        if (fs != sizeof(hubo_state))
        {
            ROS_ERROR("Hubo state size error!");
            continue;
        }
        std::vector<trajectory_msgs::JointTrajectoryPoint> cleaned_trajectory = processTrajectory(&hubo_state);
        // Send the latest trajectory chunk
        sendTrajectory(cleaned_trajectory);
    }
}

void trajectoryCB(const trajectory_msgs::JointTrajectory& traj)
{
    // Callback to chunk and save incoming trajectories
    // Before we do anything, check if the trajectory is empty - this is a special "stop" value that flushes the current stored trajectory
    if (traj.points.size() == 0)
    {
        boost::lock_guard<boost::signals2::mutex> guard(g_mtx);
        g_trajectory_chunks.clear();
        ROS_INFO("Flushing current trajectory");
        return;
    }
    // First, chunk the trajectory into parts that can be sent over ACH channels to hubo-motion-rt
    std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > new_chunks;
    // Second, store those chunks - first, we flush the stored trajectory
    boost::lock_guard<boost::signals2::mutex> guard(g_mtx);
    g_trajectory_chunks.clear();
    g_trajectory_chunks = new_chunks;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hubo_joint_trajectory_controller_interface_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    // Register a signal handler to safely shutdown the node
    signal(SIGINT, shutdown);
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
        exit(1);
    }
    r = ach_open(&chan_traj_state, HUBO_TRAJ_STATE_CHAN, NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: HUBO_TRAJ_STATE_CHAN !");
        exit(1);
    }
    ROS_INFO("Opened ACH channels to hubo-motion-rt");
    // Set up state publisher
    std::string pub_path = nh.getNamespace() + "/state";
    g_state_pub = nh.advertise<hubo_msgs::JointTrajectoryState>(pub_path, 1);
    // Spin up the thread for communicating with hubo
    interface_thread(&sendingLoop);
    // Set up the trajectory subscriber
    std::string sub_path = nh.getNamespace() + "/command";
    g_traj_sub = nh.subscribe(sub_path, 1, trajectoryCB);
    ROS_INFO("Loaded trajectory interface to hubo-motion-rt");
    // Spin until killed
    ros::spin();
    // Make the compiler happy
    return 0;
}

