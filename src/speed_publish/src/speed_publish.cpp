#include <stdio.h>
#include <ros/ros.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <libpcan.h>

#include "common.h"

#define DEFAULT_NODE "/dev/pcan3"
#ifndef bool
#define bool	int
#define true	1
#define false	0
#endif

HANDLE h;
const char *current_release;
__u32 rx_msg_count = 0;

void do_exit(int error)
{
	if (h) {
		print_diag("receivetest");
		CAN_Close(h);
	}
	printf("receivetest: finished (%d): %u message(s) received\n\n",
			error, rx_msg_count);
	exit(error);
}
 
void signal_handler(int signal)
{
	do_exit(0);
}

void init()
{
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
}

void print_msg(const TPCANRdMsg &m){
	for (int i = 0; i < m.Msg.LEN; i++)
			printf("%02x ", m.Msg.DATA[i]);

	printf("\n");
};

int read_loop()
{
	TPCANRdMsg m;
	__u32 status;

	if (LINUX_CAN_Read(h, &m)) {
		perror("receivetest: LINUX_CAN_Read()");
		return errno;
	}

	rx_msg_count++;
	print_msg(m);
	if (m.Msg.MSGTYPE & MSGTYPE_STATUS) {
		status = CAN_Status(h);
		if ((int)status < 0) {
			errno = nGetLastError();
			perror("receivetest: CAN_Status()");
			return errno;
		}	
	}
	return 0;
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"speed_publish");
    ros::NodeHandle nh;

	const char  *szDevNode = DEFAULT_NODE;
	errno = 0;

	init();
	// printf("receivetest: device node=\"%s\"\n", szDevNode);
	h = LINUX_CAN_Open(szDevNode, O_RDWR);
	if (!h) {
			printf("receivetest: can't open %s\n", szDevNode);
			do_exit(errno);
			return errno;
	}
	ros::Rate loop_rate(1);
	while(ros::ok()){
		errno=read_loop();

        ros::spinOnce();
	}
    loop_rate.sleep();
}
