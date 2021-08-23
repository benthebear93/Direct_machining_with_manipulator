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
#define KEYENCE "169.254.29.71"
#define PORT 24691

int main(int argc, char**argv)
{
	std_msgs::Float32 mid_laser;
	ros::init(argc, argv, "keyence_laser");
	ros::NodeHandle nh;
	ros::Publisher mid_laser_pub = nh.advertise<std_msgs::Float32>("mid_laser" ,100);
	ros::Rate loop_rate(500); //min : 0.000s, max : 0.015s

	int sockfd;
	struct sockaddr_in server_addr, client_addr;

	unsigned char LJV_GetMeasurementValues[] = { 0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0xF0, 0x00, \
                                        0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, \
                                        0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    size_t LJV_GetMeasurementValues_s = sizeof(LJV_GetMeasurementValues);

    sockfd=socket(AF_INET,SOCK_STREAM,0);
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr=inet_addr(KEYENCE);
    server_addr.sin_port=htons(PORT);
    int check = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if(check == 0){
    	while(ros::ok()){
    		send(sockfd, LJV_GetMeasurementValues, LJV_GetMeasurementValues_s, MSG_DONTROUTE);
    		char recvline[1000];
    		int recv_data;
			usleep(5000);
    		recv_data = read(sockfd, recvline, 1000);
    		recvline[recv_data] = 0;
    		for(int i =3; i<=5; i++){
    			int msb, lsb, byte2, byte3, outStartBytePosition, outMeasurementValue, byteOffset;
    			double outMeasurementValueMM;
    			byteOffset = (i-1)*8; //each laser measurment has 8bit offset;
	            outStartBytePosition = 232 + byteOffset; //Out1 measurement value starts with lsb at byte 232 therefore we add OutNumber*8 to 
	            msb = (unsigned char)recvline[outStartBytePosition+3]; //due to network byte order, msb is the last of 4 bytes
	            byte2 =  (unsigned char)recvline[outStartBytePosition+2]; //followed by byte #2
	            byte3 = (unsigned char)recvline[outStartBytePosition+1];  //byte #3
	            lsb = (unsigned char)recvline[outStartBytePosition]; //and lsb
	            outMeasurementValue = msb << 24 | byte2 << 16 | byte3 << 8 | lsb; //shift bytes to big endian
            	outMeasurementValueMM = outMeasurementValue * 0.00001; //since values are stored in 10nm units we have to multiply with factor 0.00001 to reach a mm-scale
    			if(i == 5){
    				mid_laser.data = outMeasurementValueMM;
    				mid_laser_pub.publish(mid_laser);
					printf("%f", mid_laser);
    			}
    		}
    	loop_rate.sleep();
    	}
    }else{
    	printf("not connect\n ");
    }
    close(sockfd);
    return 0;
}