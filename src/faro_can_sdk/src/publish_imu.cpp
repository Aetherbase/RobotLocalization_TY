#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>

extern "C" { 
#include "faro_can_sdk.h" 
}

static uint8_t scan_port = 0;
std::string port_name;
static int stop_thread = 0;

#define fprintf(fmt,...) {if (!g_quiet_print_flag) printf(__VA_ARGS__);}

void sig_handler(int signo)
{
	if(signo == SIGINT) {
		fprintf(stderr, "catch SIGINT\n");
		stop_thread = 1;
	}

}

static int do_scan_port(void)
{
	int ret = 0;
	char buf[PATH_MAX];
	int serial_port_num = 0;

	memset(buf, sizeof(buf), 0x00);
	AZ_VC_GetCommPort(buf, &serial_port_num);
	fprintf(stdout, "Found %d serial ports\n", serial_port_num);
	if(serial_port_num > 0)
		fprintf(stdout, "buf -> \n%s", buf);

	return ret;
}

inline void try_Assert(int _val, const char* errstr){
	if(_val!=0){
		scan_port = 0;
		fprintf(stdout,errstr);
		exit(1);
		}
}
void makeReadable(const sensor_ctrl_format_t& input_msg,uint16_t* msgarr){
	uint8_t* tempmsg=(uint8_t*)msgarr;
	tempmsg[0]=input_msg.ctrl1;
	tempmsg[1]=input_msg.ctrl2;
	tempmsg[2]=input_msg.ctrl3;
	tempmsg[3]=input_msg.ctrl4;
	tempmsg[4]=input_msg.ctrl5;
	tempmsg[5]=input_msg.ctrl6;
};
void printACCdata(){
	try_Assert(AZ_VC_GetACCData(),"Calling AZ_VC_GetACCData fail\n");
	struct sensor_ctrl_format_t msg={0};
	uint32_t read_size=0;
	AZ_VC_Sensor_Read(&msg, &read_size);
	try_Assert((msg.cmd!=0x91),"Unexpect cmd");
	fprintf(stdout,"ACC Data:\n");
	uint16_t tempmsg[3];
	makeReadable(msg,tempmsg);
	std::cout<<"\tXX ="<<(uint32_t)(tempmsg[0])<<" YY ="<<(uint32_t)(tempmsg[1])<<" ZZ ="<<(uint32_t)(tempmsg[2])<<std::endl;

}

void printGyroData(){
	try_Assert(AZ_VC_GetGyroData(),"Calling AZ_VC_GetGyroData fail\n");
	struct sensor_ctrl_format_t msg={0};
	uint32_t read_size=0;
	AZ_VC_Sensor_Read(&msg, &read_size);
	try_Assert((msg.cmd!=0x8A),"Unexpect cmd");
	fprintf(stdout,"Gyro Data:\n");
	uint16_t tempmsg[3];
	makeReadable(msg,tempmsg);
	std::cout<<"\tXX ="<<(uint32_t)(tempmsg[0])<<" YY ="<<(uint32_t)(tempmsg[1])<<" ZZ ="<<(uint32_t)(tempmsg[2])<<std::endl;

}
int main(int argc, char *argv[])
{
	ros::init(argc,argv,"publish_imu");
    ros::NodeHandle nh;

	nh.param<std::string>("port",port_name,"/dev/ttyUSB0");

	if(signal(SIGINT, sig_handler) == SIG_ERR)
		fprintf(stderr, "fail to catch SIGINT\n");

	try_Assert(do_scan_port(),"do_scan_port fail\n");
	static char* _port=(char*)port_name.c_str();

	
#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "starting do_sensor_test on %s\n", port_name.c_str());
#endif
	try_Assert(AZ_VC_Init(_port),"sensor initialization failed\n");

	try_Assert(AZ_VC_ModeActive(0),"Calling AZ_VC_ModeActive fail\n");

	
	
	ros::Rate loop_rate(1);


	while(ros::ok()){

		printACCdata();
		printGyroData();

        ros::spinOnce();
		}
    loop_rate.sleep();
	AZ_VC_DeInit();
    fprintf(stderr, "rosspin stopped\n");
	fflush(stderr);
}

