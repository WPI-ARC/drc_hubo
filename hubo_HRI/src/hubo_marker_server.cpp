/*
Copyright (c) 2013,  Nicholas Alunni - Worcester Polytechnic Institute Darpa
Robotics Challenge (DRC) Team
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <math.h>
#include <string>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <std_srvs/Empty.h>

//Services for working with Hubo
#include "hubo_srvs/LadderClimbingWalk.h"

using namespace interactive_markers;
using namespace visualization_msgs;

ros::ServiceClient client_wave;
ros::ServiceClient client_walk;
ros::ServiceClient client_ladder;
std_srvs::Empty srv_wave;
std_srvs::Empty srv_walk;
hubo_srvs::LadderClimbingWalk srv_ladder;

void makeWaveIcon(geometry_msgs::Pose newPose);

interactive_markers::MenuHandler menu_handler;
interactive_markers::InteractiveMarkerServer * server = NULL;

std::string frame_id = "/base_link";
std::string marker_name;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

    geometry_msgs::Pose currentPose;
    currentPose = feedback->pose;

    switch (feedback->event_type){

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:

            //Set the position to where the valve current is

            switch(feedback->menu_entry_id){

            //WAVE
            case 1:

                ROS_INFO("CALLING THE WAVE SERVICE");
                
                //Try and make the call down to the robot
                if(client_wave.call(srv_wave)){ }
                else { }

                server->erase(marker_name);
                makeWaveIcon(currentPose);
                break;

            //WALK
            case 2:

                if(client_walk.call(srv_walk)){ }
                else { }

                break;

            //Ladder Climb
            case 3:

                srv_ladder.request.input = 10;

                if(client_ladder.call(srv_ladder)){ }
                else { }

                break;

      } //END switch(feedback->menu_entry_id){

            break;
    } //END switch (feedback->event_type){

  server->applyChanges();
}

////////////////////////////////////////////////////////////////////////////////////

void makeWaveIcon(geometry_msgs::Pose newPose){

// Variables needed for creating the valve

    Marker marker;
    InteractiveMarker int_marker;
    InteractiveMarkerControl control_box, control_menu;


// ********* Marker ********* //

    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://hubo_HRI/meshes/SimsIcon.dae";

    marker.scale.x = .01;
    marker.scale.y = .01;
    marker.scale.z = .01;

    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = .75;


  // ****** Create the Interactive Marker ****** //
    int_marker.name = "Sims_Icon";
    int_marker.description = "Trollin Aint Easy";
    int_marker.header.frame_id = "/base_link";
    marker_name = int_marker.name;

    int_marker.pose.position.x = newPose.position.x;
    int_marker.pose.position.y = newPose.position.y;
    int_marker.pose.position.z = newPose.position.z;
    int_marker.pose.orientation.x = newPose.orientation.x;
    int_marker.pose.orientation.y = newPose.orientation.y;
    int_marker.pose.orientation.z = newPose.orientation.z;
    int_marker.pose.orientation.w = newPose.orientation.w;


// ********* The Model to Control ********* //
    control_box.always_visible = true;
    control_box.interaction_mode = InteractiveMarkerControl::MENU;
    control_box.markers.push_back( marker );
    int_marker.controls.push_back( control_box );
    int_marker.controls.back();



// ********* The Menu Controller ********* //
//    control_menu.interaction_mode = InteractiveMarkerControl::MENU;
//    int_marker.controls.push_back(control_menu);

// ********* Update The Server ********* //
    //"Publish" the marker to the server
    server->insert(int_marker, &processFeedback);
    menu_handler.apply( *server, int_marker.name );

}

double count = 0;
void check(void){

    count += .01;

    if (count >= 1) count = 0;

    geometry_msgs::Pose start;
    start.position.x = 0;
    start.position.y = 0;
    start.position.z = 2;
    start.orientation.x = 0;
    start.orientation.y = 0;
    start.orientation.z = count;
    start.orientation.w = 1 - count;

    server->erase(marker_name);
    makeWaveIcon(start);
    server->applyChanges();

}

// %Tag(main)%
int main(int argc, char** argv)
{
    ros::init(argc, argv, "hubo_marker_server");
    ros::NodeHandle n;

    server = new interactive_markers::InteractiveMarkerServer("hubo_marker_server","",false);
    ros::Duration(.1).sleep();

    client_wave = n.serviceClient<std_srvs::Empty>("/hubo_wave");
    client_walk = n.serviceClient<std_srvs::Empty>("/hubo_walk");
    client_ladder = n.serviceClient<hubo_srvs::LadderClimbingWalk>("ladder_climbing/walk");

    //Setup the menu options, this may change location
    menu_handler.insert( "Wave", &processFeedback );
    menu_handler.insert( "Walk", &processFeedback );
    menu_handler.insert( "Ladder Climb", &processFeedback );

    geometry_msgs::Pose start;
    start.position.x = 0;
    start.position.y = 0;
    start.position.z = 2;
    start.orientation.x = 0;
    start.orientation.y = 0;
    start.orientation.z = 0;
    start.orientation.w = 1;

    makeWaveIcon(start);
    server->applyChanges();

    ros::Rate r(20);

    while(ros::ok()){

        check();

        ros::spinOnce();

        r.sleep();
    }

    ros::spin();
}
// %EndTag(main)%
