#include "ros/ros.h"
#include "std_msgs/Float32.h"
//#include "keyence_ros/keyence_laser.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
//pcl related
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
//define
#define KEYENCE "169.254.29.71"
#define PORT 24691
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
const static std::string DEFAULT_FRAME_ID = "lj_v7080_optical_frame";
double pos_y = 0;
double temp = 0;

int main(int argc, char**argv)
{
    std_msgs::Float32 mid_laser;
    ros::init(argc, argv, "stream_16");
    ros::NodeHandle nh, pnh("~");

    std::string frame_id;
    std::string tf_frame;
    pnh.param<std::string>("frame_id", tf_frame, std::string("/base_link"));
    Cloud::Ptr pc_msg(new Cloud);
    //vector<XYZCoordinate> values;

    pc_msg->header.frame_id  = tf_frame;
    pc_msg->is_dense         = false;
    pc_msg->width            = 16;
    pc_msg->height           = 1;
    ros::Publisher pub_pc    = nh.advertise<Cloud>("profiles", 100);
    //ros::Publisher pub       = nh.advertise<std_msgs::Float32>("mid_laser" ,100);
    ros::Rate loop_rate(1000); //1000Hz = 0.05sec
    //ros node & publisher init

    int sockfd;
    struct sockaddr_in servaddr,cliaddr;
    unsigned char LJV_GetMeasurementValues[] = { 0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0xF0, 0x00, \
                                        0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, \
                                        0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    size_t LJV_GetMeasurementValues_s   = sizeof(LJV_GetMeasurementValues);  //get size

    sockfd                    = socket(AF_INET,SOCK_STREAM,0);
    servaddr.sin_family       = AF_INET;
    servaddr.sin_addr.s_addr  = inet_addr(KEYENCE);
    servaddr.sin_port         = htons(PORT);
    int connect_check         = connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
    //socket init
    
    double init_val = 0;
    if(connect_check == 0){
        while(ros::ok()){
            bool init_flag = true;

            send(sockfd, LJV_GetMeasurementValues, LJV_GetMeasurementValues_s, MSG_DONTROUTE);
            char recvline[1000];  //byte-buffer for response

            int n;
            usleep(5000); //simply wait x ms for controller to answer - replace this with an event-driven approach if needed
            n = read(sockfd,recvline,1000); //read socket/received data || ,0,NULL,NULL
            recvline[n]=0; //add string termination character 

            pc_msg -> points.clear(); //clear points
            double x = 0, y = 0, z = 0;
            pc_msg->points.reserve(16);
            
            for(int i =1; i<=16; i++){
                int msb, lsb, byte2, byte3, outStartBytePosition, outMeasurementValue, byteOffset; //integer-vairables
                double outMeasurementValueMM; //measurement value in mm-scale should be a float value
                byteOffset = (i-1) * 8; //every Out has an Offset of 8 byte to the prvious one (except Out1 which starts in byte 232) 
                outStartBytePosition = 232 + byteOffset; //Out1 measurement value starts with lsb at byte 232 therefore we add OutNumber*8 to 
                msb = (unsigned char)recvline[outStartBytePosition+3]; //due to network byte order, msb is the last of 4 bytes
                byte2 =  (unsigned char)recvline[outStartBytePosition+2]; //followed by byte #2
                byte3 = (unsigned char)recvline[outStartBytePosition+1];  //byte #3
                lsb = (unsigned char)recvline[outStartBytePosition]; //and lsb
                outMeasurementValue = msb << 24 | byte2 << 16 | byte3 << 8 | lsb; //shift bytes to big endian
                outMeasurementValueMM = outMeasurementValue * 0.00001; //since values are stored in 10nm units we have to multiply with factor 0.00001 to reach a mm-scale
                x = i*0.001;
                y = pos_y *0.001;
                z = outMeasurementValueMM;
                printf("%d :: Real OUT value = %fmm \n",i, outMeasurementValueMM); //output decimal value for OUT1-16
                pc_msg->points.push_back(pcl::PointXYZ(x, y, z));
            }
            pub_pc.publish(pc_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    else{
        printf("not connecting \n ");
    }
   close(sockfd);
   return 0;
}