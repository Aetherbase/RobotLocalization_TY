#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>

extern "C" { 
#include "faro_can_sdk.h" 
}
const char* ROSTOPIC_FOR_PUBLISHER="IMU_topic";
const char* ROSTOPIC_SPEED_PUBLISHER="/ezypilot/canspeed";
static uint8_t scan_port = 0;
std::string port_name;
static int stop_thread = 0;
sensor_msgs::Imu imu;
std_msgs::Int32 speed; 

#define fprintf(fmt,...) {if (!g_quiet_print_flag) printf(__VA_ARGS__);}

void sig_handler(int signo)
{
	if(signo == SIGINT) {
		fprintf(stderr, "catch SIGINT\n");
		stop_thread = 1;
	}
}
#ifdef FARO_CAN_SDK_DEBUG
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
#endif
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

int calibAcc(){
	struct sensor_ctrl_format_t msg;
	memset(&msg, 0x00, sizeof(msg));
	msg.cmd=0x55;
	msg.dlc_cfg=0x00;
	msg.ctrl1=0x00;
	msg.ctrl2=0x00;
	msg.ctrl3=0x00;
	msg.ctrl4=0x00;
	msg.ctrl5=0x00;
	msg.ctrl6=0x00;
	msg.ctrl7=0x00;
	return AZ_VC_Sensor_Write(msg);
}

int calibGyro(){
	struct sensor_ctrl_format_t msg;
	memset(&msg, 0x00, sizeof(msg));
	msg.cmd=0x46;
	msg.dlc_cfg=0x00;
	msg.ctrl1=0x00;
	msg.ctrl2=0x00;
	msg.ctrl3=0x00;
	msg.ctrl4=0x00;
	msg.ctrl5=0x00;
	msg.ctrl6=0x00;
	msg.ctrl7=0x00;
	return AZ_VC_Sensor_Write(msg);
}

int setGyroScale(){
	struct sensor_ctrl_format_t msg;
	memset(&msg, 0x00, sizeof(msg));
	msg.cmd=0x47;
	msg.dlc_cfg=0x23;
	msg.ctrl1=0x01;
	msg.ctrl2=0x20;
	return AZ_VC_Sensor_Write(msg);
}
int setACCScale(){
	struct sensor_ctrl_format_t msg;
	memset(&msg, 0x00, sizeof(msg));
	msg.cmd=0x5F;
	msg.ctrl1=0x00; // for +-2g
	return AZ_VC_Sensor_Write(msg);
}
void can_config()
{

#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "starting can_config on %s\n", port_name);
#endif
	struct can_config_t can_config_data;
	can_config_data.port=0;
	can_config_data.speed=(can_speed)2;
	try_Assert(AZ_VC_CAN_Config(can_config_data),"CAN config failed/n");

}
bool print_speed()
{
	struct can_message_t msg;
	memset(&msg, 0x00, sizeof(msg));
	uint32_t size = 0;

	try_Assert(AZ_VC_CAN_Read(&msg, &size),"Error reading CAN\n");
	fprintf(stdout, "port = %d, ID = 0x%08X\n",  msg.port, msg.id);									
	if(msg.id == 0x80)		//Ccveh_speed data
		{
		float speed_f = (float)((msg.data[0] << 8) | msg.data[1]);
		speed_f*=0.05555555555;
		fprintf(stdout,"speed : %.1f km\n",speed_f);
		speed.data=(speed_f*0.2777*1000);
		return true;
		}
	return false;
}
inline float mapACC(uint x)
{
	constexpr float accval=9.80665*2;
	return ((x * 2*accval) / 65535) - accval;
}
inline float mapGyro(uint x)
{
	constexpr float gyrval=245;
	return ((x * 2*gyrval) / 65535) - gyrval;
}
void printACCdata(){
	try_Assert(AZ_VC_GetACCData(),"Calling AZ_VC_GetACCData fail\n");
	struct sensor_ctrl_format_t msg;
	memset(&msg, 0x00, sizeof(msg));
	uint32_t read_size=0;
	AZ_VC_Sensor_Read(&msg, &read_size);

	try_Assert((msg.cmd!=0x91),"Unexpect cmd");
	fprintf(stdout,"ACC Data:\n");
	uint16_t tempmsg[3];
	makeReadable(msg,tempmsg);
	std::cout<<"\tXX ="<<(uint32_t)(tempmsg[0])<<" YY ="<<(uint32_t)(tempmsg[1])\
	<<" ZZ ="<<(uint32_t)(tempmsg[2])<<std::endl;

	imu.linear_acceleration.x=mapACC((uint32_t)tempmsg[0]);
	imu.linear_acceleration.y=mapACC((uint32_t)tempmsg[1]);
	imu.linear_acceleration.z=mapACC((uint32_t)tempmsg[2]);
}
void printGyroData(){
	try_Assert(AZ_VC_GetGyroData(),"Calling AZ_VC_GetGyroData fail\n");
	struct sensor_ctrl_format_t msg;
	memset(&msg, 0x00, sizeof(msg));
	uint32_t read_size=0;
	AZ_VC_Sensor_Read(&msg, &read_size);

	try_Assert((msg.cmd!=0x8A),"Unexpect cmd");
	fprintf(stdout,"Gyro Data:\n");
	uint16_t tempmsg[3];
	makeReadable(msg,tempmsg);
	std::cout<<"\tXX ="<<(uint32_t)(tempmsg[0])<<" YY ="<<(uint32_t)(tempmsg[1])<<" ZZ ="<<(uint32_t)(tempmsg[2])<<std::endl;
	
	imu.angular_velocity.x=mapGyro((uint32_t)tempmsg[0]);
	imu.angular_velocity.y=mapGyro((uint32_t)tempmsg[1]);
	imu.angular_velocity.z=mapGyro((uint32_t)tempmsg[2]);
}
int main(int argc, char *argv[])
{
	ros::init(argc,argv,"publish_imu");
    ros::NodeHandle nh;

	nh.param<std::string>("port",port_name,"/dev/ttyUSB0");
    ros::Publisher IMUPub = nh.advertise<sensor_msgs::Imu>(ROSTOPIC_FOR_PUBLISHER, 1000);
    ros::Publisher SpeedPub = nh.advertise<std_msgs::Int32>(ROSTOPIC_SPEED_PUBLISHER, 10);

	if(signal(SIGINT, sig_handler) == SIG_ERR)
		fprintf(stderr, "fail to catch SIGINT\n");
	
	static char* _port=(char*)port_name.c_str();
#ifdef FARO_CAN_SDK_DEBUG
	try_Assert(do_scan_port(),"do_scan_port fail\n");
	fprintf(stdout, "starting do_sensor_test on %s\n", port_name.c_str());
#endif
	try_Assert(AZ_VC_Init(_port),"sensor initialization failed\n");
	try_Assert(AZ_VC_ModeActive(0),"Calling AZ_VC_ModeActive fail\n");
	try_Assert(calibAcc(),"failed to Calibrate Acc");
	try_Assert(calibGyro(),"failed to Calibrate Gyro");
	//can_config();
	try_Assert(setACCScale(),"failed setting ACCScale");
	try_Assert(setGyroScale(),"failed setting GyroScale");
	

#ifdef FARO_CAN_SDK_DEBUG
	ros::Rate loop_rate(100);
#else
	ros::Rate loop_rate(1);
#endif
	while(ros::ok()){
		imu.header.stamp=ros::Time::now();
		imu.header.frame_id="imu_link";

		printACCdata();
		printGyroData();
		//if(print_speed()) SpeedPub.publish(speed);
		
		IMUPub.publish(imu);
        ros::spinOnce();
		}
    loop_rate.sleep();
	AZ_VC_DeInit();
    fprintf(stderr, "rosspin stopped\n");
	fflush(stderr);
}

