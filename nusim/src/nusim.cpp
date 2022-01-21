

#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Empty.h"
#include "nusim/Tele.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

/// \file
/// \brief This node runs the nusimulator. It loads in robot and obstacles into RVIZ
///
/// PARAMETERS:
///     ~rate (integer): publishing rate
/// PUBLISHES:
///     ~timestep (std_msgs::UInt64): simulation timestep
///     /red/joint_states (sensor_msgs::JointState): turtlebot jointstates
///     transform_broadcaster between world and red:basefootprint
/// SUBSCRIBES:
///     No subscribers
/// SERVICES:
///     reset (std_srvs::Empty): This service resets the simulation
///     tele (nusim::Tele): This service teleports the robot to x,y,theta defined by user




namespace{
    int counter = 0;
   
    double x;
    double y;
    double theta;
    
    double xorigin;
    double theta0;
    double yorigin;
}

    /// \brief reset simulation to start
    /// \returns boolean true upon completion of service
bool reset(std_srvs::Empty::Request& , std_srvs::Empty::Response& ){

    counter = 0;
    x = xorigin;
    y = yorigin;
    theta = theta0;
    return true;
}

    /// \brief teleport the robot to an x,y,theta position in the world frame
    /// \param request - x,y, theta coordinates to teleport to
    /// \returns the angle in degrees
bool tele(nusim::Tele::Request& request, nusim::Tele::Response& ){

    x = request.x;
    y = request.y;
    theta = request.t;
    return true;
}



using namespace std;

int main(int argc, char * argv[]){



    int f;

    long unsigned int i;

    //Obstacle coordinates
    vector<double> o_r;
    vector<double> o_x;
    vector<double> o_y;

    ros::init(argc,argv,"nusim");
    ros::NodeHandle nh("~");

    visualization_msgs::Marker m;
    visualization_msgs::MarkerArray m_array;


    //Load in parameters from basic_world.yaml
    nh.param("rate", f, 500);
    nh.param("x0", xorigin, 0.0);
    nh.param("theta0", theta0, 0.0);
    nh.param("y0",yorigin,0.0);
    nh.getParam("obstacles/x",o_x); //noservice needed, just read from the yaml file and create from the yaml
    nh.getParam("obstacles/y",o_y);
    nh.getParam("obstacles/r",o_r);        

    ros::Duration life(0);


    // Add obstacles
    for(i=0;i<o_x.size();i++){
        m.header.frame_id = "world";
        m.header.stamp = ros::Time::now();
        m.ns = "obstacles";
        m.action = visualization_msgs::Marker::ADD;
        m.id = i;
        m.type = visualization_msgs::Marker::CYLINDER;
        
        m.scale.x = o_r[i];
        m.scale.y = o_r[i];
        m.scale.z = 0.25;
        
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        m.color.a = 1.0;

        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        
        m.pose.position.x = o_x[i];
        m.pose.position.y = o_y[i];
        m.pose.position.z = 0.125;

        m.lifetime = life;
        m_array.markers.push_back(m); //source (01/18): https://answers.ros.org/question/35246/add-markers-to-markerarray-in-c/

    }


    x = xorigin;
    y = yorigin;
    theta = theta0;

    ros::Publisher m_pub;
    m_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10, true);

    m_pub.publish(m_array);

    ros::Publisher count_pub;
    count_pub = nh.advertise<std_msgs::UInt64>("timestep", 10);

    ros::Publisher joint_pub;
    joint_pub = nh.advertise<sensor_msgs::JointState>("/red/joint_states", 10);

    ros::ServiceServer srv_reset;
    srv_reset = nh.advertiseService("reset", reset);

    ros::ServiceServer srv_tele;
    srv_tele = nh.advertiseService("tele", tele);


    //Source (01/17) start: https://github.com/wsnewman/davinci_wsn/blob/master/wsn_move_davinci_rviz/src/davinci_joint_state_publisher.cpp
    sensor_msgs::JointState state;
    state.name.push_back("red-wheel_left_joint");
    state.name.push_back("red-wheel_right_joint");

    state.position.push_back(0.0);
    state.position.push_back(0.0);

    state.velocity.push_back(0.0);
    state.velocity.push_back(0.0);

    state.effort.push_back(0.0);
    state.effort.push_back(0.0);
    //source end

    tf2_ros::TransformBroadcaster b;
    geometry_msgs::TransformStamped ts;

    ros::Rate rate(f);

    while(ros::ok()){

        //Continuously broadcast transform
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "red-base_footprint";
        ts.transform.translation.x = x;
        ts.transform.translation.y = y;
        ts.transform.translation.z = 0;
        tf2::Quaternion ang;
        ang.setRPY(0,0,theta);
        ts.transform.rotation.x = ang.x();
        ts.transform.rotation.y = ang.y();
        ts.transform.rotation.z = ang.z();;
        ts.transform.rotation.w = ang.w();
        b.sendTransform(ts);

        state.header.stamp = ros::Time::now();

        std_msgs::UInt64 num;
        num.data = counter;
        
        //Publish joint states and timer
        count_pub.publish(num);
        counter++;

        joint_pub.publish(state);

        ros::spinOnce();
        rate.sleep();

    }
    
    return 0;
}