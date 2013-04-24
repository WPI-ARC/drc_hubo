#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <libxml/parser.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
//#include "beginner_tutorials/AddTwoInts.h"
#include "hackdrc_wave_planner/HuboWaveTraj.h"

using namespace std;

std::string robot_name = "rlhuboplus";
std::string file_name_0 = "movetraj0.txt";
std::string file_name_1 = "movetraj1.txt";
std::string directory_name = "/home/jmainpri/workspace/drc/wpi_openrave/hubo/python/";
std::vector<std::string> joint_names;

void set_openrave_hubo_joints()
{
    joint_names.clear();
    joint_names.push_back("HPY");                          
    joint_names.push_back("RHY");                           
    joint_names.push_back("LHY");                               
    joint_names.push_back("RHR");                              
    joint_names.push_back("LHR");                         
    joint_names.push_back("RHP");                             
    joint_names.push_back("LHP");                              
    joint_names.push_back("RKP");                                
    joint_names.push_back("LKP");                             
    joint_names.push_back("RAP");                               
    joint_names.push_back("LAP");                                
    joint_names.push_back("RAR");                                 
    joint_names.push_back("LAR");                                
    joint_names.push_back("RSP");                              
    joint_names.push_back("LSP");                              
    joint_names.push_back("RSR");                       
    joint_names.push_back("LSR");                             
    joint_names.push_back("RSY");                              
    joint_names.push_back("LSY");                               
    joint_names.push_back("REP");                               
    joint_names.push_back("LEP");                            
    joint_names.push_back("RWY");                        
    joint_names.push_back("LWY");                    
    joint_names.push_back("RWP");                        
    joint_names.push_back("LWP");                  
    joint_names.push_back("HNR");                 
    joint_names.push_back("HNP");                    
    joint_names.push_back("rightIndexKnuckle2");    
    joint_names.push_back("rightIndexKnuckle3");        
    joint_names.push_back("rightIndexKnuckle1");        
    joint_names.push_back("rightMiddleKnuckle2");      
    joint_names.push_back("rightMiddleKnuckle3");        
    joint_names.push_back("rightMiddleKnuckle1");   
    joint_names.push_back("rightRingKnuckle2");       
    joint_names.push_back("rightRingKnuckle3");      
    joint_names.push_back("rightRingKnuckle1");    
    joint_names.push_back("rightPinkyKnuckle2");         
    joint_names.push_back("rightPinkyKnuckle3");         
    joint_names.push_back("rightPinkyKnuckle1");        
    joint_names.push_back("rightThumbKnuckle2");    
    joint_names.push_back("rightThumbKnuckle3");          
    joint_names.push_back("rightThumbKnuckle1");         
    joint_names.push_back("leftIndexKnuckle2");         
    joint_names.push_back("leftIndexKnuckle3");        
    joint_names.push_back("leftIndexKnuckle1");        
    joint_names.push_back("leftMiddleKnuckle2");     
    joint_names.push_back("leftMiddleKnuckle3");       
    joint_names.push_back("leftMiddleKnuckle1");       
    joint_names.push_back("leftRingKnuckle2");            
    joint_names.push_back("leftRingKnuckle3");     
    joint_names.push_back("leftRingKnuckle1");          
    joint_names.push_back("leftPinkyKnuckle2");          
    joint_names.push_back("leftPinkyKnuckle3");         
    joint_names.push_back("leftPinkyKnuckle1");      
    joint_names.push_back("leftThumbKnuckle2");       
    joint_names.push_back("leftThumbKnuckle3");     
    joint_names.push_back("leftThumbKnuckle1");   
}

template <class T>
bool convert_text_to_num(T& t,
                         const std::string& s,
                         std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

class openraveTrajectory
{
public:
    std::vector<Eigen::VectorXd> positions;
    std::vector<Eigen::VectorXd> velocities;
    std::vector<double> deltatime;
};

struct robot_and_dof
{
    int nb_dofs;
    std::string robot_name;
    std::string type;
};

bool fct_sort( std::pair<int,robot_and_dof> a, std::pair<int,robot_and_dof> b)
{
    return a.first < b.first;
}

void loadTrajecoryFromFile( std::string filename, openraveTrajectory& traj )
{
    cout << "-------------------------------------------" << endl;
    cout << " load file : " << filename << endl;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;
    xmlChar* tmp;

    doc = xmlParseFile(filename.c_str());
    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return;
    }

    root = xmlDocGetRootElement(doc);
    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return;
    }

    if (xmlStrcmp(root->name, xmlCharStrdup("trajectory")))
    {
        cout << "Document of the wrong type root node not trajectory" << endl;
        xmlFreeDoc(doc);
        return;
    }

    cur = root->xmlChildrenNode->next;

    //    while (cur != NULL)
    //    {
    //        cout << cur->name << endl;
    //        cur = cur->next->next;
    //    }

    if (xmlStrcmp(cur->name, xmlCharStrdup("configuration")))
    {
        cout << "Error : no node named configuration" << endl;
        xmlFreeDoc(doc);
        return;
    }

    std::vector< std::pair<int,robot_and_dof> > offsets;

    xmlNodePtr node =  cur->xmlChildrenNode->next;

    while( node != NULL )
    {
        //cout << xmlGetProp( node, xmlCharStrdup("name") ) << endl;
        robot_and_dof rd;

        offsets.push_back(std::make_pair(0,rd));

        tmp = xmlGetProp( node, xmlCharStrdup("offset") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().first, (char*)tmp, std::dec );
        //cout << offsets.back().first << endl;

        tmp = xmlGetProp( node, xmlCharStrdup("dof") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().second.nb_dofs, (char*)tmp, std::dec );
        //cout << offsets.back().second.nb_dofs << endl;

        std::stringstream ss( (char *)xmlGetProp( node, xmlCharStrdup("name") ) );
        std::string line;

        std::getline( ss, line, ' ' );
        offsets.back().second.type = line;
        //cout << offsets.back().second.type << endl;

        std::getline( ss, line, ' ' );
        offsets.back().second.robot_name = line;
        //cout << offsets.back().second.robot_name << endl;

        node = node->next->next;
    }

    std::sort( offsets.begin(), offsets.end(), fct_sort );

    // ------------------------------------------------

    cur = cur->next->next;

    if (xmlStrcmp(cur->name, xmlCharStrdup("data")))
    {
        cout << "Error : no node named data" << endl;
        xmlFreeDoc(doc);
        return;
    }

    tmp = xmlGetProp( cur, xmlCharStrdup("count") );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        return;
    }
    int count = 0;
    convert_text_to_num<int>( count, (char*)tmp, std::dec );
    //cout << count << endl;

    tmp = xmlNodeGetContent( cur );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        return;
    }

    std::string configuration( (char*)(tmp) );
    // cout << configuration << endl;
    std::stringstream ss( configuration );
    std::vector<double> values;
    std::string line;
    while( std::getline(ss,line,' ') )
    {
        double val;
        convert_text_to_num<double>( val, line, std::dec );
        values.push_back( val );
    }

    cout << "values.size() : " << values.size() << endl;

    xmlFreeDoc(doc);

    traj.positions.resize(count);
    traj.velocities.resize(count);
    traj.deltatime.resize(count);

    //cout << "count : " << count << endl

    int ith_value=0;
    int configuration_offset=0;

    for(int i=0;i<count;i++)
    {
        for(int k=0;k<int(offsets.size());k++)
        {
            if( offsets[k].second.type != "deltatime" &&
                offsets[k].second.robot_name != robot_name )
            {
                ith_value += offsets[k].second.nb_dofs;
                continue;
            }

            int start = ith_value + offsets[k].first;
            int end = ith_value + offsets[k].first + offsets[k].second.nb_dofs;

            if( end > values.size() )
            {
                cout << " name : "  <<  offsets[k].second.robot_name << ", ith_value : " << ith_value << endl;
                cout << " type : " << offsets[k].second.type << endl;
                cout << " nb of dof : " << offsets[k].second.nb_dofs << endl;
                cout << " end : "  <<   end << endl;
                cout << "ERROR Reading trajectory" << endl;
                continue;
            }

            configuration_offset += offsets[k].second.nb_dofs;

            if( offsets[k].second.type == "joint_values" )
            {
                traj.positions[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.positions[i][l++] = values[j];
                }
                //cout << "position : start = " << start << " , end = " << end << endl;
                //cout << traj.positions[i].transpose() << endl;
            }

            if( offsets[k].second.type == "joint_velocities" )
            {
                traj.velocities[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.velocities[i][l++] = values[j];
                }
                //cout << "velocities : start = " << start << " , end = " << end << endl;
            }

            if( offsets[k].second.type == "deltatime" )
            {
                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.deltatime[i] = values[l++];
                }

                ith_value += configuration_offset;
                configuration_offset = 0;
                //cout << "deltatime : start = " << start << " , end = " << end << endl;
            }
        }
    }
}

void printConfigurations( const std::vector<openraveTrajectory>& trajs)
{
    for(int i=0;i<int(trajs.size());i++)
    {
        for(int j=0;j<int(trajs[i].positions.size());j++)
        {
            cout << trajs[i].positions[j].transpose() << endl;
        }
    }
}

void setTrajectoryMessage( const std::vector<openraveTrajectory>& trajs, hackdrc_wave_planner::HuboWaveTraj::Response& response )
{
    cout << "set stamp " << endl;
    
    // When to start the trajectory: 1s from now
    response.Response.header.stamp = ros::Time::now() + ros::Duration(1.0);
    //traj_client_->sendGoal(goal);
    
    // First, the joint names, which apply to all waypoints
    for(int i=0;i<57;i++)
    {
        response.Response.joint_names.push_back( joint_names[i].c_str() );
    }
    
    int nb_way_points=0;
    
    for(int i=0;i<int(trajs.size());i++)
    {
        nb_way_points += trajs[i].positions.size();
    }

   response.Response.points.resize( nb_way_points );
   
   cout << "nb_way_points : " << nb_way_points << endl;
          
   int ind = 0;
      
    for(int i=0;i<int(trajs.size());i++)
    {
        for(int j=0;j<int(trajs[i].positions.size());j++)
        {
            //cout << "set way_point : " << j << " in traj "  << i << endl;
            
            response.Response.points[ind].positions.resize(57);
            
            for(int k=0;k<int(trajs[i].positions[j].size());k++)
            {
                response.Response.points[ind].positions[k] = trajs[i].positions[j][k];
            }
            
            // Velocities
//            response.Response.points[ind].velocities.resize(57);
//            for (size_t k = 0; k < 57; ++k)
//            {
//              response.Response.points[ind].velocities[k] = 0.0;
//            }
            // To be reached 1 second after starting along the trajectory
            response.Response.points[ind].time_from_start = ros::Duration(1.0);

            ind++;
        }
    }
}

bool parseHuboTrajectory( hackdrc_wave_planner::HuboWaveTraj::Request& request, hackdrc_wave_planner::HuboWaveTraj::Response& response )
{
    std::vector<openraveTrajectory> trajs;
    
    Eigen::VectorXd q(57);

    trajs.clear();
    trajs.resize(2);
    
    loadTrajecoryFromFile( directory_name + file_name_0, trajs[0] );
    loadTrajecoryFromFile( directory_name + file_name_1, trajs[1] );
    //printConfigurations( trajs );
    setTrajectoryMessage( trajs, response );
    return true;
}

//bool add(beginner_tutorials::AddTwoInts::Request  &req,
//         beginner_tutorials::AddTwoInts::response.Response &res)
//{
//  res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response.Response: [%ld]", (long int)res.sum);
//  return true;
//}

int main(int argc, char **argv)
{
    set_openrave_hubo_joints();

    ros::init(argc, argv, "parse_wave_trajectory_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService( "parse_wave_trajectory", parseHuboTrajectory );
    ROS_INFO("Ready to read trajectory");
    ros::spin();

    return 0;
}
