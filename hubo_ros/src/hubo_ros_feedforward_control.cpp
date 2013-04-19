/*
ROS code:
---------
Copyright (c) 2012, Calder Phillips-Grafflin, WPI DRC Team
(2-clause BSD)

ACH code:
---------
Copyright (c) 2012, Daniel M. Lofaro
(3-clause BSD)
*/

// ros includes
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>
#include <sstream>
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "std_msgs/String.h"
#include "hubo_ros/HuboCommand.h"
#include "hubo_ros/HuboHandCommand.h"
#include "hubo_ros/HuboJointCommand.h"
#include "ach.h"
//WPI includes
#include "../../wpi_hubo/hubo-ach/include/hubo.h"
#include "../../wpi_hubo/hubo-ach/include/hubo-ref-filter.h"
//Hubo includes
//#include "../../../hubo-ach/include/hubo.h"
//#include "../../../hubo-ach/include/hubo-ref-filter.h"

//Global variables
ach_channel_t chan_hubo_ref_filter;
int hubo_debug = 0;
char *urdf_joint_names[] = {"HPY", "not in urdf1", "HNR", "HNP", "LSP", "LSR", "LSY", "LEP", "LWY", "not in urdf2", "LWP", "RSP", "RSR", "RSY", "REP", "RWY", "not in urdf3", "RWP", "not in ach1", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR", "not in ach1", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR", "not in urdf4", "not in urdf5", "not in urdf6", "not in urdf7", "not in urdf8", "not in urdf9", "not in urdf10", "not in urdf11", "not in urdf12", "not in urdf13", "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"};

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
        if (strcmp(joint_name.c_str(), urdf_joint_names[i]) == 0)
        {
            match = true;
            best_match = i;
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

void hubo_cb(const hubo_ros::HuboCommand &msg)
{
    printf("Received command message\n");
    //Send the commands from the HuboCommand message onto ACH to the robot
    //Make the necessary hubo struct for ACH
    struct hubo_ref H_ref_filter;
    memset( &H_ref_filter, 0, sizeof(H_ref_filter));
    size_t fs;
    //First, get the current state of the Hubo from ACH
    int r = ach_get(&chan_hubo_ref_filter, &H_ref_filter, sizeof(H_ref_filter), &fs, NULL, ACH_O_LAST);
    if(ACH_OK != r)
    {
        if(hubo_debug)
        {
            printf("State ini r = %i\n",r);
        }
    }
    else
    {
        assert(sizeof(H_ref_filter) == fs);
    }
    printf("Converting ROS message containing [%d] joint commands to hubo-ach\n", msg.num_joints);
    //Add the joint values one at a time into the hubo struct
    //for each joint command, we lookup the best matching
    //joint in the header to set the index
    for (int i = 0; i < msg.num_joints; i++)
    {
        int index = IndexLookup(msg.joints[i].name);
        if (index != -1)
        {
            printf("Mapped URDF joint name [%s] to hubo joint index [%d]\n", msg.joints[i].name.c_str(), index);
            H_ref_filter.ref[index] = msg.joints[i].position;
        }
    }
    //If there are any joint values not assigned in the message, don't change them in the struct!
    printf("Sending a new state on ACH\n");
    //Put the new message into the ACH channel
    ach_put(&chan_hubo_ref_filter, &H_ref_filter, sizeof(H_ref_filter));
}


//NEW MAIN LOOP
int main(int argc, char **argv)
{
    printf("Initializing ROS-to-ACH bridge\n");
    //initialize ACH channel
    int r = ach_open(&chan_hubo_ref_filter, HUBO_CHAN_REF_FILTER_NAME , NULL);
    assert(ACH_OK == r);
    printf("Hubo-ACH channel loaded\n");
    //initialize ROS node
    ros::init(argc, argv, "hubo_ros_feedforward");
    ros::NodeHandle nh;
    printf("Node up\n");
    //construct ROS RT Subscriber
    ros::Subscriber hubo_command_sub = nh.subscribe("Hubo/HuboCommand", 1, hubo_cb);
    printf("Subscriber up\n");
    //spin
    ros::spin();
    //Satisfy the compiler
    return 0;
}
