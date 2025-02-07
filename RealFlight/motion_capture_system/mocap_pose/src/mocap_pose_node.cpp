#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

/*
vrpn pose ros send and dds publish
*/

static ros::Publisher pose_2_mavros_pub, pose_pub, odom_pub;
geometry_msgs::PoseStamped mocappose;

std::string vrpn_topic = "";
std::string mavros_topic ="";
std::string drone_pose_topic = "";
std::string drone_odom_topic = "";
double offset_x = 0.0, offset_y = 0.0;

void mocap_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& ptr){
    mocappose = *ptr;
    
    geometry_msgs::PoseStamped mp;
    mp.header = ptr->header;
    mp.header.stamp = ros::Time::now();
    mp.header.frame_id="world";
    mp.pose = ptr->pose;  
    
    mp.pose.position.x += offset_x;
    mp.pose.position.y += offset_y;

    if(mavros_topic.length() > 0)pose_2_mavros_pub.publish(mp); 

    if(pose_pub.getNumSubscribers() > 1)
    {
        pose_pub.publish(mp); //to pose topic
    }   
    
    nav_msgs::Odometry op;
    op.header = ptr->header;
    op.header.stamp = ros::Time::now();
    op.header.frame_id = "world";
    op.pose.pose = ptr->pose;
    if(mavros_topic.length() > 0)
    {
        odom_pub.publish(op); //to odom topic
    }   


    tf2::Quaternion q(mp.pose.orientation.x,mp.pose.orientation.y,mp.pose.orientation.z,mp.pose.orientation.w);
    double roll=0.0,pitch=0.0,yaw=0.0;
    tf2::Matrix3x3 rmat(q);
    rmat.getRPY(roll,pitch,yaw);

    // ROS_INFO("mocap pose: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f.",
    //         mp.pose.position.x, mp.pose.position.y,mp.pose.position.z,
    //         roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);

}


int main(int argc, char** argv){

    ros::init(argc,argv,"mocap_pose_node");
    ros::NodeHandle node("~");

    node.param<std::string>("vrpn_topic", vrpn_topic,""); 
    node.param<std::string>("mavros_topic", mavros_topic,"");
    node.param<std::string>("drone_pose_topic", drone_pose_topic,"");
    node.param<std::string>("drone_odom_topic", drone_odom_topic,"");
    node.param<double>("offset_x", offset_x, 0.0);
    node.param<double>("offset_y", offset_y, 0.0);

    if(mavros_topic.length() > 0)
    {
        pose_2_mavros_pub = node.advertise<geometry_msgs::PoseStamped>(mavros_topic,30);
    }

    pose_pub = node.advertise<geometry_msgs::PoseStamped>(drone_pose_topic,30);
    odom_pub = node.advertise<nav_msgs::Odometry>(drone_odom_topic,30);
    ros::Subscriber posesub = node.subscribe<geometry_msgs::PoseStamped>(vrpn_topic,30,mocap_pose_callback);
    ros::spin();
    node.shutdown();

    return 0;
}
