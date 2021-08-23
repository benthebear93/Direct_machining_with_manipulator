#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h> 
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;
#define KEYENCE "192.168.10.10"
#define KEYENCE_PORT 24687
void PrintHexa(char* buff, size_t len);

int main(int argc, char**argv) {
    std_msgs::Float32 measurement_mm;
    ros::init(argc, argv, "lk_keyence_output");
    ros::NodeHandle nh;
    ros::Publisher laser_pub = nh.advertise<std_msgs::Float32>("LK_laser", 100);
    ros::Rate loop_rate(200);

    struct sockaddr_in sock_addr;
    int ret = 0;
    int sockfd;
    sockfd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);


    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = inet_addr(KEYENCE);
    sock_addr.sin_port = htons(KEYENCE_PORT);

    char LK_GetMeasurementValue[] = { 0x07, 0x00, 0x07, 0x14, 0x00, 0x00, 0x00 };// register, register, CR, LF
    size_t LK_GetMeasurementValue_s = sizeof(LK_GetMeasurementValue);
    char OutputMeasurement[116];
    if (connect(sockfd, (struct sockaddr*)&sock_addr, sizeof(sock_addr)) == -1) {
        perror("connect()");
        close(sockfd);
        return 0;
    }
    double init_val = 0;
    bool init_flag = true;
    while (ros::ok()) {
        if (send(sockfd, LK_GetMeasurementValue, LK_GetMeasurementValue_s, 0) == -1) {
            perror("send()");
            close(sockfd);
            return 0;
        }
        ret = recv(sockfd, OutputMeasurement, 116, 0);
        if (ret == -1) {
            perror("recv()"); close(sockfd);
            return 0;
        }
        //printf("\n");
        //PrintHexa(OutputMeasurement, ret);
        int byte = 0, byte2 = 0, msb = 0, byte3 = 0;
        double outMeasurementValueMM = 0;
        byte = (unsigned char)OutputMeasurement[68]; //B1
        byte2 = (unsigned char)OutputMeasurement[69]; //05
        byte3 = (unsigned char)OutputMeasurement[70];
        msb =(unsigned char)OutputMeasurement[71];
        outMeasurementValueMM = msb << 24| byte3<< 16 | byte2 << 8 | byte;
        //measurement_mm.data = outMeasurementValueMM/1000.0;
        //laser_pub.publish(measurement_mm);
        if(init_flag ==true){
            for (int i =0; i < 10; i++){
                //ROS_INFO("Setting Origin values");
                init_val+= outMeasurementValueMM;
                }
            init_val = init_val/10.0;
            //printf("Pub OUT value = %fmm \n", init_val);
            init_flag = false;
        } // save first 10 data for initialize origin point
        else{
            float changes = -init_val + outMeasurementValueMM;
            measurement_mm.data = changes/1000.0;
            ROS_INFO("%f",changes/1000.0);
            //printf("OUT value = %fmm \n", changes/1000.0); //output decimal value for OUT1-16
            //vertical_error.data = changes*100;
            //vertical_error_pub.publish(vertical_error);
            laser_pub.publish(measurement_mm);
        }
      //printf("out put : %f mm\n", outMeasurementValueMM/1000.0);
      loop_rate.sleep();
   }
   close(sockfd);
   system("pause");
}

void PrintHexa(char* buff, size_t len) {
   size_t i;
   for (i = 0; i < len; i++)
   {
      printf("%02X ", (unsigned char)buff[i]);
   }
   printf("\n");
}