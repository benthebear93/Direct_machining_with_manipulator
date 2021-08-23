#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
//#include "keyence_ros/keyence_laser.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#define KEYENCE "169.254.29.71"
#define PORT 24691

int main(int argc, char**argv)
{
    std_msgs::Float32 mid_laser;
    std_msgs::Int32 vertical_error;
    ros::init(argc, argv, "keyence_output");
    ros::NodeHandle nh;
    ros::Publisher mid_laser_pub = nh.advertise<std_msgs::Float32>("mid_laser" ,100);
    ros::Publisher vertical_error_pub = nh.advertise<std_msgs::Int32>("ver_error" ,100);
    ros::Rate loop_rate(500); //1000Hz = 0.05sec

    int sockfd;
    struct sockaddr_in servaddr,cliaddr;
    unsigned char LJV_GetMeasurementValues[] = { 0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0xF0, 0x00, \
                                        0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, \
                                        0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    size_t LJV_GetMeasurementValues_s = sizeof(LJV_GetMeasurementValues);  //get size

    sockfd=socket(AF_INET,SOCK_STREAM,0);
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=inet_addr(KEYENCE);
    servaddr.sin_port=htons(PORT);
    int a = connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));\

    double init_val = 0;
    if(a == 0){
        bool init_flag = true;
        while(ros::ok()){
            send(sockfd, LJV_GetMeasurementValues, LJV_GetMeasurementValues_s, MSG_DONTROUTE);
            char recvline[1000];  //byte-buffer for response
            int n;
            usleep(5000); //simply wait x ms for controller to answer - replace this with an event-driven approach if needed
            n=read(sockfd,recvline,1000); //read socket/received data || ,0,NULL,NULL
            recvline[n]=0; //add string termination character 
            int msb, lsb, byte2, byte3, outStartBytePosition, outMeasurementValue, byteOffset; //integer-vairables
            double outMeasurementValueMM; //measurement value in mm-scale should be a float value
            byteOffset = (0) * 8; //every Out has an Offset of 8 byte to the prvious one (except Out1 which starts in byte 232) 
            outStartBytePosition = 232 + byteOffset; //Out1 measurement value starts with lsb at byte 232 therefore we add OutNumber*8 to 
            msb = (unsigned char)recvline[outStartBytePosition+3]; //due to network byte order, msb is the last of 4 bytes
            byte2 =  (unsigned char)recvline[outStartBytePosition+2]; //followed by byte #2
            byte3 = (unsigned char)recvline[outStartBytePosition+1];  //byte #3
            lsb = (unsigned char)recvline[outStartBytePosition]; //and lsb
            outMeasurementValue = msb << 24 | byte2 << 16 | byte3 << 8 | lsb; //shift bytes to big endian
            outMeasurementValueMM = outMeasurementValue * 0.00001; //since values are stored in 10nm units we have to multiply with factor 0.00001 to reach a mm-scale
            /*if(outMeasurementValueMM<-100){
                outMeasurementValueMM = - 50;
            }
            else{
                outMeasurementValueMM = outMeasurementValueMM;
            }*/
            ROS_INFO("%f", outMeasurementValueMM);
            //printf("Real OUT value = %fmm \n", outMeasurementValueMM); //output decimal value for OUT1-16
            //mid_laser.data = outMeasurementValueMM;
            //mid_laser_pub.publish(mid_laser);
            if(init_flag ==true){
                for (int i =0; i < 10; i++){
                    ROS_INFO("Setting Origin values");
                    init_val+= outMeasurementValueMM;
                }
                init_val = init_val/10.0;
                printf("Pub OUT value = %fmm \n", init_val);
                init_flag = false;
            } // save first 10 data for initialize origin point
            else{
                float changes = outMeasurementValueMM; //-init_val + 
                //printf("OUT value = %fmm \n", changes); //output decimal value for OUT1-16
                mid_laser.data = changes;
                //vertical_error.data = changes*100;
                //vertical_error_pub.publish(vertical_error);
                mid_laser_pub.publish(mid_laser);
            }
            /*
            if(outMeasurementValueMM < 0){
                outMeasurementValueMM =-1*outMeasurementValueMM + 80;
                printf("here?");
            }
            else {
                outMeasurementValueMM = 80 - outMeasurementValueMM;
                printf("here?2");
            }*/
                //keyence_output_pub.publish(laser_dist);
                //ROS_INFO("Value : %f",outMeasurementValueMM);
            //}
        loop_rate.sleep();
        }
    }
    else{
        printf("not connecting \n ");
    }
   close(sockfd);
   return 0;
}