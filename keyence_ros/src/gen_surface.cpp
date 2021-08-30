#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <ctime>
//#include <fstream>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
const static std::string DEFAULT_FRAME_ID = "lj_v7080_optical_frame";
class Scanner{
    public:
        Scanner();
        ~Scanner();
        bool init();
        float pos_x;
        float pos_y;
        float pos_z;
        float temp;
        
        std::string frame_id;
        std::string tf_frame;
        //std_msgs::Float64MultiArray arr_profile;
        std_msgs::Float32MultiArray profile_sum;
        Cloud::Ptr  point3d{new Cloud};
        Cloud::Ptr  sum_pointcloud{new Cloud};
        Cloud::Ptr  temp_pointcloud{new Cloud};
        pcl::PointCloud<pcl::PointXYZ> sum_point;
        tf::TransformListener listener;
        tf::StampedTransform btolj; //base to LJ sensor
        //vector<xyz> point_storage;
        string filepath = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner"; // basic file path
    private:

        ros::NodeHandle nh_, pnh_{"~"};

        ros::Publisher  point_pub_;
        //ros::Publisher  arr_profile_;
        ros::Publisher  pub_test_;
        ros::Subscriber cloud_sub_;
        ros::Publisher profile_sum_;
        void cloudMsgCallback(const sensor_msgs::PointCloud2& msg);
};

Scanner::Scanner() // init func
{
    ROS_INFO("Generate surface Node init");
    auto ret = init();
    ROS_ASSERT(ret);
}

Scanner::~Scanner() // destroy
{
    ros::shutdown();
}

bool Scanner::init()
{
    ROS_INFO("init!");
    point_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("CMM_profile", 100); 
    //arr_profile_  = nh_.advertise<std_msgs::Float64MultiArray>("arr_profile", 100);
    cloud_sub_    = nh_.subscribe("/profiles", 1, &Scanner::cloudMsgCallback, this);
    pub_test_     = nh_.advertise<std_msgs::Float32>("test_topic", 100); 
    profile_sum_  = nh_.advertise<std_msgs::Float32MultiArray>("profile_sum", 100);

    pos_y    = 0;
    pos_x    = 0;
    pos_z    = 0;
    temp     = 0;

    point3d.reset(new Cloud);
    point3d->header.frame_id  = "tool0";
    point3d->is_dense         = false;
    point3d->width            = 800;
    point3d->height           = 1;

    sum_pointcloud.reset(new Cloud);
    temp_pointcloud.reset(new Cloud);

    return true;
}

void Scanner::cloudMsgCallback(const sensor_msgs::PointCloud2& msg)
{
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("base_link", "lj_v7080_optical_frame", now, ros::Duration(0.0001));
        listener.lookupTransform("base_link", "lj_v7080_optical_frame", ros::Time(0), btolj);
        pos_x = btolj.getOrigin().x();
        pos_y = btolj.getOrigin().y();
        pos_z = btolj.getOrigin().z(); //sensor coordinate from base_link
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what()); 
        ros::Duration(0.01).sleep();
    }
    //ROS_INFO("tf time ");
    pcl::PCLPointCloud2 pcl_pc;  // pcl point cloud
    pcl_conversions::toPCL(msg, pcl_pc);  // sensor msg to pcl

    pcl::PointCloud<pcl::PointXYZ> input_cloud; // pcl point xyz
    pcl::fromPCLPointCloud2(pcl_pc, input_cloud); // pcl to Pointcloud 

    int profile_size = msg.width;
    float x = 0,y = 0,z =0; //5e-05sec
    for(int i =0; i<profile_size; i+=8)
    {
        x = input_cloud.points[i].x + pos_x;
        y = input_cloud.points[i].y + pos_y;
        z = input_cloud.points[i].z + pos_z;
        if(z>10) // inf
        {
            z = 10;
            y = 10;
        }
        profile_sum.data.push_back(x);
        profile_sum.data.push_back(y);
        profile_sum.data.push_back(z);
        if(i==profile_size-8)
        {
            //ROS_INFO("pubping");
            profile_sum_.publish(profile_sum);
            profile_sum.data.clear();
        }
    }
    std_msgs::Float32 test;
    test.data = pos_x;
    pub_test_.publish(test);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "generate_surface");
    Scanner Scanner;
    ros::spin();
    // if (!ros::ok()) {
    //     ROS_INFO("save pcd");
    //     std::string fileName = Scanner.filepath +"/pcd_data/profile_hz.pcd";
    //     pcl::io::savePCDFileASCII(fileName, *Scanner.sum_pointcloud);
    // }
}