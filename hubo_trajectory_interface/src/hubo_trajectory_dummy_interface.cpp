
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

#define MAX_TRAJ_LENGTH 10

// ACH channels
ach_channel_t chan_traj_cmd;
ach_channel_t chan_traj_state;

// Index->Joint name mapping
char *joint_names[] = {"HPY", "not in urdf1", "HNR", "HNP", "LSP", "LSR", "LSY", "LEP", "LWY", "not in urdf2", "LWP", "RSP", "RSR", "RSY", "REP", "RWY", "not in urdf3", "RWP", "not in ach1", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR_dummy", "not in ach1", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR_dummy", "not in urdf4", "not in urdf5", "not in urdf6", "not in urdf7", "not in urdf8", "not in urdf9", "not in urdf10", "not in urdf11", "not in urdf12", "not in urdf13", "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"};

// Trajectory storage
std::vector<std::string> g_joint_names;
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
                ach_traj.joint[i].velocity[p] = processed_traj[p].velocities[i];
                ach_traj.joint[i].acceleration[p] = processed_traj[p].accelerations[i];
            }
            ach_traj.time[p] = processed_traj[p].time_from_start.toSec();
        }
        ach_traj.endTime = processed_traj.back().time_from_start.toSec();
        ach_traj.trajID = g_tid;
        ach_put(&chan_traj_cmd, &ach_traj, sizeof(ach_traj));
        g_tid++;
    }
}

// From the name of the joint, find the corresponding joint index for the Hubo-ACH struct
int IndexLookup(std::string joint_name)
{
    //Find the Hubo joint name [from hubo.h!] and the relevant index
    //to map from the ROS HuboCommand message to the hubo-ach struct
    bool match = false;
    int best_match = -1;
    //See if we've got a matching joint name, and if so, return the
    //relevant index so we can map it into the hubo struct
    for (int i = 0; i < HUBO_JOINT_COUNT; i++)
    {
        if (strcmp(joint_name.c_str(), joint_names[i]) == 0)
        {
            match = true;
            best_match = i;
            break;
        }
    }
    if (match)
    {
        return best_match;
    }
    else
    {
        return -1;
    }
}

trajectory_msgs::JointTrajectoryPoint processPoint(trajectory_msgs::JointTrajectoryPoint raw, hubo_traj_output* hubo_state)
{
    trajectory_msgs::JointTrajectoryPoint processed;
    if (g_joint_names.size() != raw.positions.size())
    {
        ROS_ERROR("Stored joint names and received joint commands do not match");
        processed.positions.resize(HUBO_JOINT_COUNT);
        processed.velocities.resize(HUBO_JOINT_COUNT);
        processed.accelerations.resize(HUBO_JOINT_COUNT);
        return processed;
    }
    else
    {
        // Remap the provided joint trajectory point to the Hubo's joint indices
        processed.positions.resize(HUBO_JOINT_COUNT);
        processed.velocities.resize(HUBO_JOINT_COUNT);
        processed.accelerations.resize(HUBO_JOINT_COUNT);
        // First, fill in everything with data from the current Hubo state
        for (int i = 0; i < HUBO_JOINT_COUNT; i++)
        {
            processed.positions[index] = hubo_state->joint[i].position_actual;
            processed.velocities[index] = hubo_state->joint[i].velocity_actual;
            processed.accelerations[index] = hubo_state->joint[i].acceleration_actual;
        }
        // Now, overwrite with the commands in the current trajectory
        for (unsigned int i = 0; i < raw.positions.size(); i++)
        {
            int index = IndexLookup(g_joint_names[i]);
            if (index != -1)
            {
                processed.positions[index] = raw.positions[i];
                processed.velocities[index] = raw.velocities[i];
                processed.accelerations[index] = raw.accelerations[i];
            }
        }
        return processed;
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
        std::vector<trajectory_msgs::JointTrajectoryPoint> cur_set;
        for (unsigned int i = 0; i < cur_set.size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint processed_point = processPoint(cur_set[i], hubo_state);
            processed.push_back(processed_point);
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
        // Publish the latest hubo state back out
        hubo_msgs::JointTrajectoryState cur_state;
        cur_state.header.stamp = ros::Time::now();
        // Set the names
        boost::lock_guard<boost::signals2::mutex> guard(g_mtx);
        cur_state.joint_names = g_joint_names;
        unsigned int num_joints = g_joint_names.size();
        // Make the empty states
        trajectory_msgs::JointTrajectoryPoint cur_setpoint;
        trajectory_msgs::JointTrajectoryPoint cur_actual;
        trajectory_msgs::JointTrajectoryPoint cur_error;
        // Resize the states
        cur_setpoint.positions.resize(num_joints);
        cur_setpoint.velocities.resize(num_joints);
        cur_setpoint.accelerations.resize(num_joints);
        cur_actual.positions.resize(num_joints);
        cur_actual.velocities.resize(num_joints);
        cur_actual.accelerations.resize(num_joints);
        cur_error.positions.resize(num_joints);
        cur_error.velocities.resize(num_joints);
        cur_error.accelerations.resize(num_joints);
        // Fill in the setpoint and actual & calc the error in the process
        for (unsigned int i = 0; i < num_joints; i++)
        {
            // Fill in the setpoint and actual data
            int hubo_index = IndexLookup(cur_state.joint_names[i]);
            // Something along the lines of = hubo_state.joint[hubo_index].field?
            cur_setpoint.positions[i] = hubo_state.joint[i].position_setpoint;
            cur_setpoint.velocities[i] = hubo_state.joint[i].velocity_setpoint;
            cur_setpoint.accelerations[i] = hubo_state.joint[i].acceleration_setpoint;
            cur_actual.positions[i] = hubo_state.joint[i].position_actual;
            cur_actual.velocities[i] = hubo_state.joint[i].velocity_actual;
            cur_actual.accelerations[i] = hubo_state.joint[i].acceleration_actual;
            // Calc the error
            cur_error.positions[i] = cur_setpoint.positions[i] - cur_actual.positions[i];
            cur_error.velocities[i] = cur_setpoint.velocities[i] - cur_actual.velocities[i];
            cur_error.accelerations[i] = cur_setpoint.accelerations[i] - cur_actual.accelerations[i];
        }
        // Pack them together
        cur_state.desired = cur_setpoint;
        cur_state.actual = cur_actual;
        cur_state.error = cur_error;
        // Publish
        g_state_pub.publish(cur_state);
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
        g_joint_names.clear();
        ROS_INFO("Flushing current trajectory");
        return;
    }
    // First, chunk the trajectory into parts that can be sent over ACH channels to hubo-motion-rt
    std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > new_chunks;
    unsigned int i = 0;
    ros::Duration base_time(0.0);
    while (i < traj.points.size())
    {
        std::vector<trajectory_msgs::JointTrajectoryPoint> new_chunk;
        unsigned int index = 0;
        while (index < traj.points.size() && index < MAX_TRAJ_LENGTH)
        {
            // Make sure the JointTrajectoryPoint gets retimed to match its new trajectory chunk
            trajectory_msgs::JointTrajectoryPoint cur_point = traj.points[i];
            // Retime based on the end time of the previous trajectory chunk
            cur_point.time_from_start = cur_point.time_from_start - base_time;
            // Make sure position, velocity, and acceleration are all the same length
            if (cur_point.accelerations.size() != cur_point.positions.size())
            {
                cur_point.accelerations.resize(cur_point.positions.size());
            }
            if (cur_point.velocities.size() != cur_point.positions.size())
            {
                cur_point.velocities.resize(cur_point.positions.size());
            }
            // Store it
            new_chunk.push_back(cur_point);
            index++;
            i++;
        }
        // Save the ending time to use for the next chunk
        base_time = new_chunk.back().time_from_start;
        // Store it
        new_chunks.push_back(new_chunk);
    }
    // Second, store those chunks - first, we flush the stored trajectory
    boost::lock_guard<boost::signals2::mutex> guard(g_mtx);
    g_trajectory_chunks.clear();
    g_joint_names.clear();
    g_trajectory_chunks = new_chunks;
    g_joint_names = traj.joint_names;
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

