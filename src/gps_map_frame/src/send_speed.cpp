
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <serial/serial.h>
#include <sstream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

typedef uint8_t byte_t; 
const int Length=12;
byte_t ubx[20] = {0};
byte_t ubx1[20] ={0xB5,0x62,0x10,0x02,0x0C,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x38,0x4C,0xC9,0x16,0x00,0x0B,0x8C,0x03};
int *const tempSpeed=(int*)(ubx+14);
int *const tempCount=(int*)(ubx+6);



serial::Serial mySerial;
const int BAUD_RATE = 115200;
const char* SERIAL_PORT= "/dev/ttyUSB1";
const char* ROSTOPIC_FOR_PUBLISHER="GPS_read";
const char* ROSTOPIC_SPEED_SUBSCRIBE="speed32";

inline void CountTime(){
    static uint32_t timerCount = 0;
    *tempCount=timerCount;
    timerCount++;
    };

inline void updateSpeed(const std_msgs::Int32::ConstPtr& speed){
    *tempSpeed=speed->data;
    ubx[17] = 0x0B; //data type
};
   
void checksumRefresh(){
    byte_t CA = 0, CB = 0;
    for(int I=2;I< (Length+ 6);I++){
        CA = CA + ubx[I];
        CB = CB + CA;
        }
    ubx[18] = CA; //checksum
    ubx[19] = CB;//checksum
    };


void write_speed(const std_msgs::Int32::ConstPtr& speed){
    CountTime();
    updateSpeed(speed);
    checksumRefresh();
    mySerial.write(ubx,sizeof(ubx));
};
void parseGPS(const char* inputS, sensor_msgs::NavSatFix& toSend){
    float Lat,Long,Alt;
    char ns,ew;
    sscanf(inputS,"%*200c $PUBX,%*d,%*f,%f,%[NS],%f,%[EW],%f,%*[NDGRT]%*[FR23KT],%*f,%*f,%*f,%*f,%*f,,%*f,%*f,%*f,%*d,%*d,%*f",&Lat,&ns,&Long,&ew,&Alt);
    Lat=(ns=='S') ? Lat/100*(-1) : Lat/100;
    Long=(ew=='W') ? Long/100*(-1) : Long/100;
    toSend.latitude=Lat;
    toSend.longitude=Long;
    toSend.altitude=Alt;
    /*'time', 'latitude', 'NSindicator', 'Longitude', 'EWindicator', 'altitude', 'navigationStatus',
             'horizontalAcc', 'verticalAcc', 'speedOverGround', 'courseOverGround', 'verticalVelocity',
             'diffAge', 'Hdop', 'Vdop', 'Tdop', 'NumberOfSatalites'*/
};


int main(int argc, char** argv){
    ros::init(argc, argv, "speedWrite");
    ros::NodeHandle nh;
    ros::Publisher GPSReadPub = nh.advertise<sensor_msgs::NavSatFix>(ROSTOPIC_FOR_PUBLISHER, 1000);
    sensor_msgs::NavSatFix GpsSend;

    try
    {
        mySerial.setPort(SERIAL_PORT);
        mySerial.setBaudrate(BAUD_RATE);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        mySerial.setTimeout(to);
        mySerial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Port unavailable ");
        return -1;
    }

    ros::Rate loop_rate(10);

    ubx[0] = 0xB5; // Preamble
    ubx[1] = 0x62; // Preamble
    ubx[2] = 0x10; // Class
    ubx[3] = 0x02; // ID
    ubx[4] = Length;  //0x0C; // Length (low) Length=8+N*4 = 8+4 = 12
    ubx[5] = 0; // Length (high)

    ubx[10] = 0; // flags (low)
    ubx[11] = 0; // flags (high)
    ubx[12] = 0x38; // id (low)
    ubx[13] = 0x4c; // id (high)
    ubx[17] = 0x0B; // Data Type

    ros::Subscriber speed_sub = nh.subscribe("Speed", 1000, write_speed);

    while(ros::ok()){
        ros::spinOnce();

        if(mySerial.available()){
            std_msgs::String readFromSerial,sendToTopic;
            readFromSerial.data = mySerial.read(mySerial.available());
            std::stringstream tempstream;
            ROS_INFO_STREAM("Read: " << readFromSerial.data);
            parseGPS(readFromSerial.data.c_str(),GpsSend);
            GPSReadPub.publish(GpsSend);
        }
        loop_rate.sleep();
    }
    
    return 0;
    }
