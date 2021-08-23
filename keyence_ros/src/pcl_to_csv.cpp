#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
//pcl related
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
//#include <fstream>

using namespace std;

/*struct xyz
{
    double x,y,z;
};*/

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
const static std::string DEFAULT_FRAME_ID = "lj_v7080_optical_frame";

class Surface{
    public:

        Surface();
        ~Surface();
        bool init();
        double pos_y;
        double temp;
        double lk_laser;
        std::string frame_id;
        std::string tf_frame;
        std_msgs::Float64MultiArray profile_sum;
        Cloud::Ptr  point3d{new Cloud};
        //vector<xyz> point_storage;

    private:

        ros::NodeHandle nh_, pnh_{"~"};

        ros::Publisher  point_pub_;
        ros::Publisher profile_sum_;

        ros::Subscriber posy_sub_;
        ros::Subscriber cloud_sub_;
        ros::Subscriber lk_laser_sub_;

        void posMsgCallback(const std_msgs::Float32::ConstPtr& msg);
        void cloudMsgCallback(const sensor_msgs::PointCloud2& msg);
        void lkMsgCallback(const std_msgs::Float32::ConstPtr& msg);
};

Surface::Surface() // init func
{
    ROS_INFO("Surface Node init");
    auto ret = init();
    ROS_ASSERT(ret);
}
Surface::~Surface() // destroy
{
    ros::shutdown();
}

bool Surface::init()
{
    point_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("CMM_profile", 100); 
    posy_sub_     = nh_.subscribe("posy", 100, &Surface::posMsgCallback, this);
    cloud_sub_    = nh_.subscribe("/profiles", 1, &Surface::cloudMsgCallback, this);
    lk_laser_sub_ = nh_.subscribe("LK_laser", 1, &Surface::lkMsgCallback, this);
    profile_sum_  = nh_.advertise<std_msgs::Float64MultiArray>("profile_sum", 100);

    pos_y    = 0;
    temp     = 0;
    lk_laser = 0;

    point3d.reset(new Cloud);
    point3d->header.frame_id  = tf_frame;
    point3d->is_dense         = false;
    point3d->width            = 16;
    point3d->height           = 1;

    return true;
}
void Surface::posMsgCallback(const std_msgs::Float32::ConstPtr& msg)
{
    pos_y = msg->data;
}
void Surface::lkMsgCallback(const std_msgs::Float32::ConstPtr& msg)
{
    lk_laser = msg->data;
}
void Surface::cloudMsgCallback(const sensor_msgs::PointCloud2& msg)
{
    pcl::PCLPointCloud2 pcl_pc;  // pcl point cloud
    pcl_conversions::toPCL(msg, pcl_pc);  // sensor msg to pcl

    pcl::PointCloud<pcl::PointXYZ> input_cloud; // pcl point xyz
    pcl::fromPCLPointCloud2(pcl_pc, input_cloud); // pcl to Pointcloud 
    int profile_size = msg.width;
    point3d -> points.clear();
    point3d -> points.reserve(profile_size);
    //std::cout << "size : " << msg.width << std::endl;
    point3d->header.frame_id = "sensor_optical_frame";
    point3d->is_dense = false; // cloud could have NaNs
    // message is essentially a line-strip of points
    point3d->height = 1;
    double x = 0,y = 0,z =0;

    for(int i =0; i<profile_size; i++){
        x = input_cloud.points[i].x;
        y = pos_y;
        z = input_cloud.points[i].z-lk_laser;
        //std::cout << "z : " << input_cloud.points[i].z << std::endl;
        //std::cout << "extracted z : " << z << std::endl;
        point3d->points.push_back(pcl::PointXYZ(x, y, z));
        if(y!=temp && y < temp)
        {   
            //point_storage.push_back({x,y,z});
            profile_sum.data.push_back(x);
            profile_sum.data.push_back(y);
            profile_sum.data.push_back(z);
            if(i==profile_size-1)
            {
                //std::cout << "posy :" << y << "temp :" << temp << std::endl;
                temp = pos_y;
                profile_sum_.publish(profile_sum);
                profile_sum.data.clear();
            }
        }
        else if(y>temp)
        {
            temp = pos_y;
        }
    }
    //temp = pos_y;
    //point_pub_.publish(point3d);
}

int main(int argc, char**argv)
{
    std_msgs::Float32 mid_laser;
    ros::init(argc, argv, "pcl_to_csv");
    Surface surface;
    ros::spin ();
}