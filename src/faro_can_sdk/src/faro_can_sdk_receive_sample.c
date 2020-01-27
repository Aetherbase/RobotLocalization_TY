
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <linux/limits.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include "faro_can_sdk_demo_config.h"
#include "faro_can_sdk.h"

#define FARO_CAN_SDK_CAN_CONFIG_ARGS					2

#define fprintf(fmt,...) {if (!g_quiet_print_flag) printf(__VA_ARGS__);}


static int deinit_in_seconds = (10 * 3);
static int stop_thread = 0;
static char *port_name = NULL;
static uint8_t get_can_fw_ver = 0;
static uint8_t scan_port = 0;
static uint8_t can_config = 0;
static uint8_t receive_raw = 0;
static uint8_t receive_j1939 = 0;
static struct can_config_t can_config_data;


static void display_help(int argc, char *argv[])
{
	fprintf(stdout, "Usage: %s [OPTION]\n"
			"\n"
			"  -h, --help\n"
			"  -s, --scan, scan available serial port\n"
			"  -p, --port\n"
			"		#1-port, ex \"/dev/ttyUSB0\"\n"
			"  -d, --deinit\n"
			"		#1-deinit after x seconds, -1 endless until ctrl-c\n"
			"  -g, --get_fw_ver, get CAN firmware version\n"
			"  -c, --can_config, can config\n"
			"		#1-port: 1 byte\n"
			"		#2-speed: 1 byte, 0: 1M, 1: 800K, 2: 500K, 3: 250K, 4: 200K, 5: 125K\n"
			"  -r, --receive_raw\n"
			"  -j, --receive_j1939\n"
			"  -v, --version\n"
			"  -q, --quiet mode, set 1 to quiet mode\n"
			"\n",
			argv[0]);
	exit(0);
}

static int process_options(int argc, char *argv[])
{
	unsigned int tmp = 0;

	if(argc < 2) {
		display_help(argc, argv);
		return -1;
	}

	for (;;) {
		int option_index = 0;
		static const char *short_options = "qv:hp:sd:gc:rj";
		static const struct option long_options[] = {
			{"quiet_mode", required_argument, 0, 'q'},
			{"help", no_argument, 0, 'h'},
			{"port", required_argument, 0, 'p'},
			{"scan", no_argument, 0, 's'},
			{"deinit", required_argument, 0, 'd'},
			{"get_fw_ver", no_argument, 0, 'g'},
			{"can_config", required_argument, 0, 'c'},
			{"receive_raw", no_argument, 0, 'r'},
			{"receive_j1939", no_argument, 0, 'j'},
			{"version", no_argument, 0, 'v'},
			{0,0,0,0},
		};

		int c = getopt_long(argc, argv, short_options, long_options, &option_index);
		if (c == EOF) {
			break;
		}

		switch (c) {
		case 'h':
			display_help(argc, argv);
			break;

		case 'p':
			port_name = strdup(optarg);
			break;

		case 's':
			scan_port = 1;
			break;

		case 'd':
			deinit_in_seconds = atoi(optarg);
			if(FARO_CAN_SDK_DEBUG == 1)
				fprintf(stdout, "deinit after %d seconds\n", deinit_in_seconds);
			break;

		case 'g':
			get_can_fw_ver = 1;
			break;

		case 'c':
			{
				int cnt = 0;
				int index = optind - 1;
				int arg_num = 0;

				can_config = 1;
				memset(&can_config_data, 0x00, sizeof(can_config_data));

				while(index < argc) {
					if(argv[index][0] == '-') {
						break;
					}

					if(FARO_CAN_SDK_DEBUG == 1)
						fprintf(stdout, "argv[%d] = %s\n", index, argv[index]);

					tmp = 0;

					switch(cnt) {
						case 0:
							can_config_data.port = atoi(argv[index]);
							if(can_config_data.port < 0 || can_config_data.port > 1)
								goto c_err;

							break;

						case 1:
							can_config_data.speed = atoi(argv[index]);
							if(can_config_data.speed < 0 || can_config_data.speed > 5)
								goto c_err;

							break;

						default:
							goto c_err;
					}

					index++;
					cnt++;
				};

				arg_num = FARO_CAN_SDK_CAN_CONFIG_ARGS;

				if(cnt != arg_num)
					goto c_err;

				optind = index - 1;
				goto c_ok;

c_err:
				fprintf(stderr, "invalid argument number, should be %d\n", arg_num);
				display_help(argc, argv);

c_ok:
				if(FARO_CAN_SDK_DEBUG == 1)
					fprintf(stdout, "checking -c arguments done\n");
			}
			break;

		case 'r':
			receive_raw = 1;
			break;

		case 'j':
			receive_j1939 = 1;
			break;		

		case 'v':
			fprintf(stdout, "%s Version %d.%d.%d\n",
				argv[0],
				FARO_CAN_SDK_DEMO_VERSION_MAJOR,
				FARO_CAN_SDK_DEMO_VERSION_MINOR,
				FARO_CAN_SDK_DEMO_VERSION_PATCH);
			return -1;

		case 'q':
			g_quiet_print_flag = atoi(optarg);
			sleep(1);
			if(g_quiet_print_flag > 1 || g_quiet_print_flag < 0)
			{
				fprintf(stdout, "quiet mode set error, 0:Normal mode, 1:Quiet mode\n");
				return -1;
			}
			break;
		}
	}

	return 0;
}

static void *deinit_wait(void *args)
{
	int i;

	if(deinit_in_seconds == -1) {
		while(stop_thread == 0)
			sleep(1);
	} else {
		for(i=0; i<deinit_in_seconds; i++) {
			if(stop_thread != 0)
				break;

			sleep(1);
		}
	}

	if(stop_thread == 0) {
		stop_thread = 1;
		sleep(1);
	}

	AZ_VC_DeInit();

	pthread_exit(NULL);
}

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

static void *do_get_can_fw_ver(void *args)
{
	int ret = 0;
	int i;
	struct can_fw_ver_t can_fw_ver;

	if(FARO_CAN_SDK_DEBUG == 1)
		fprintf(stdout, "starting get firmware version on %s\n", port_name);

	if(port_name == NULL) {
		fprintf(stderr, "invalid port_name, assigned by -p /dev/ttyUSBx\n");
		goto err;
	}

	// initialize
	if((ret = AZ_VC_Init(port_name)) != 0)
		goto err;

	for(i=0; i<1; i++) {
		if(stop_thread != 0)
			break;

		memset(&can_fw_ver, 0x00, sizeof(can_fw_ver));
		if((ret = AZ_VC_GetFWVer(&can_fw_ver)) != 0) {
			fprintf(stderr, "Calling AZ_VC_GetFWVer fail, test_cnt = %d\n", i);
			continue;
		}

		fprintf(stdout, "get firmware version : v%02X.%02X.%02X\n",
			can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

err:
	pthread_exit(NULL);
}

static void *do_can_config(void *args)
{
	int ret = 0;
	int i;

	if(FARO_CAN_SDK_DEBUG == 1)
		fprintf(stdout, "starting can_config on %s\n", port_name);

	if(port_name == NULL) {
		fprintf(stderr, "invalid port_name, assigned by -p /dev/ttyUSBx\n");
		goto err;
	}

	// initialize
	if((ret = AZ_VC_Init(port_name)) != 0)
		goto err;

	{
		struct can_fw_ver_t can_fw_ver;

		memset(&can_fw_ver, 0x00, sizeof(can_fw_ver));
		if((ret = AZ_VC_GetFWVer(&can_fw_ver)) != 0) {
			fprintf(stderr, "Calling AZ_VC_GetFWVer fail\n");
			goto err;
		}

		fprintf(stdout, "get firmware version : v%02X.%02X.%02X\n", can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

	for(i=0; i<1; i++) {
		if(stop_thread != 0)
			break;

		if((ret = AZ_VC_CAN_Config(can_config_data)) != 0)
			goto err;

		fprintf(stdout, "test_cnt = %d, port = %d, speed = %d\n",
			i+1, can_config_data.port, can_config_data.speed);
	}

err:
	stop_thread = 1;
	pthread_exit(NULL);
}

static void *do_receive_raw_data(void *args)
{
	int ret = 0;
	int receivecount = 0;

	if(FARO_CAN_SDK_DEBUG == 1)
		fprintf(stdout, "starting get firmware version on %s\n", port_name);

	if(port_name == NULL) {
		fprintf(stderr, "invalid port_name, assigned by -p /dev/ttyUSBx\n");
		goto err;
	}

	// initialize
	if((ret = AZ_VC_Init(port_name)) != 0)
		goto err;

	{
		struct can_fw_ver_t can_fw_ver;

		memset(&can_fw_ver, 0x00, sizeof(can_fw_ver));
		if((ret = AZ_VC_GetFWVer(&can_fw_ver)) != 0) {
			fprintf(stderr, "Calling AZ_VC_GetFWVer fail\n");
			goto err;
		}

		fprintf(stdout, "get firmware version : v%02X.%02X.%02X\n", can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

	if((ret = AZ_VC_ModeActive(0X00)) != 0) {
		fprintf(stderr, "calling AZ_VC_ModeActive fail\n");
		goto err;
	}

	while(1)
	{
		if(stop_thread != 0)
			break;

		struct can_message_t msg;
		memset(&msg, 0x00, sizeof(msg));
		uint32_t size = 0;

		if((ret = AZ_VC_CAN_Read(&msg, &size)) == 0) {
		
			fprintf(stdout, "Receive count = %d, port = %d, ID = 0x%08X\n", receivecount+1, msg.port, msg.id);									

			if((msg.id & 0x00FFFF00) == 0x00FEF100)		//Ccveh_speed data
			{
				float speed = (float)((msg.data[2] << 8) | msg.data[1]) / 256;
				fprintf(stdout,"speed : %.1f km\n",speed);
			}
		
			else if((msg.id & 0x00FFFF00) == 0x00FFA100)	//AUX_STAT_ZBR1 data
			{
				if(msg.data[5] == 0x1F)		//Right Turn Signal
				{    
					fprintf(stdout,"Right Turn signal turn ON\n");
				}

				else if(msg.data[5] == 0x4F)	//Left Turn Signal
				{
					fprintf(stdout,"Left Turn signal turn ON\n");
				}

				else if(msg.data[5] == 0x0F)	//Idle Signal
				{
					fprintf(stdout,"Idle state\n");
				}
			}			
		
			receivecount++;
		}
	}

err:
	pthread_exit(NULL);
}

static void *do_receive_j1939_data(void *args)
{
	int ret = 0;
	int receivecount = 0;

	if(FARO_CAN_SDK_DEBUG == 1)
		fprintf(stdout, "starting get firmware version on %s\n", port_name);

	if(port_name == NULL) {
		fprintf(stderr, "invalid port_name, assigned by -p /dev/ttyUSBx\n");
		goto err;
	}

	// initialize
	if((ret = AZ_VC_Init(port_name)) != 0)
		goto err;

	{
		struct can_fw_ver_t can_fw_ver;

		memset(&can_fw_ver, 0x00, sizeof(can_fw_ver));
		if((ret = AZ_VC_GetFWVer(&can_fw_ver)) != 0) {
			fprintf(stderr, "Calling AZ_VC_GetFWVer fail\n");
			goto err;
		}

		fprintf(stdout, "get firmware version : v%02X.%02X.%02X\n", can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

	if((ret = AZ_VC_ModeActive(0X02)) != 0) {
		fprintf(stderr, "calling AZ_VC_ModeActive fail\n");
		goto err;
	}

	while(1)
	{
		if(stop_thread != 0)
			break;

		struct J1939_message_t msg;
		memset(&msg, 0x00, sizeof(msg));
		uint32_t size = 0;

		if(ret = AZ_VC_J1939_Read(&msg, &size) == 0)
		{
			fprintf(stdout, "J1939 read size = %d , count = %d\n", size,receivecount);
			fprintf(stdout, "\tid = 0x%08X\n", msg.id);
			fprintf(stdout, "\tp = 0x%02X\n", msg.p);
			fprintf(stdout, "\tdp = 0x%02X\n", msg.dp);
			fprintf(stdout, "\tpf = 0x%02X\n", msg.pf);
			fprintf(stdout, "\tps = 0x%02X\n", msg.ps);
			fprintf(stdout, "\tsa = 0x%02X\n", msg.sa);
			fprintf(stdout, "\tpgn= 0x%02X\n", msg.pgn);
			fprintf(stdout, "\tdlc = 0x%02X\n", msg.dlc);
			for(int i=0; i<8; i++) {
				fprintf(stdout, "\tdata[%d] = 0x%02X\n", i, msg.data[i]);
			}
			receivecount++;
		}
	}

err:
	pthread_exit(NULL);
}

static int do_functions(void)
{
	int ret = 0;
	pthread_t deinit_wait_thread;
	pthread_t do_get_can_fw_ver_thread;
	pthread_t do_can_config_thread;
	pthread_t do_receive_raw_thread;
	pthread_t do_receive_j1939_thread;

	fprintf(stdout, "SDK version : v%d.%d.%d\n",
		FARO_CAN_SDK_DEMO_VERSION_MAJOR,
		FARO_CAN_SDK_DEMO_VERSION_MINOR,
		FARO_CAN_SDK_DEMO_VERSION_PATCH);

	if(scan_port) {
		if((ret = do_scan_port()) != 0) {
			fprintf(stderr, "do_scan_port fail\n");
		}

		goto err;
	}

	// spawn deinit_wait_thread if necessary
	if(FARO_CAN_SDK_DEBUG == 1)
		fprintf(stdout, "spawning deinit_wait thread...\n");

	if((ret = pthread_create(&deinit_wait_thread, NULL, deinit_wait, NULL)) != 0) {
		fprintf(stderr, "create deinit_wait thread fail\n");
			goto err;
	}

	if(get_can_fw_ver) {
		if(FARO_CAN_SDK_DEBUG == 1)
			fprintf(stdout, "spawning do_get_can_fw_ver thread...\n");

		if((ret = pthread_create(&do_get_can_fw_ver_thread, NULL, do_get_can_fw_ver, NULL)) != 0) {
			fprintf(stderr, "create do_get_can_fw_ver thread fail\n");
			goto err;
		}

		ret = pthread_join(do_get_can_fw_ver_thread, NULL);
		if(ret) {
			fprintf(stderr, "do_get_can_fw_ver_thread thread error: %s\n", strerror(ret));
			goto err;
		}

		goto normal_exit;
	}

	if(can_config) {
		if(FARO_CAN_SDK_DEBUG == 1)
			fprintf(stdout, "spawning do_can_config thread...\n");
		if((ret = pthread_create(&do_can_config_thread, NULL, do_can_config, NULL)) != 0) {
			fprintf(stderr, "create do_can_config thread fail\n");
			goto err;
		}

		ret = pthread_join(do_can_config_thread, NULL);
		if(ret) {
			fprintf(stderr, "do_can_config_thread error: %s\n", strerror(ret));
			goto err;
		}

		goto normal_exit;
	}

	if(receive_raw) {
		if(FARO_CAN_SDK_DEBUG == 1)
			fprintf(stdout, "spawning do_receive_raw thread...\n");

		pthread_create(&do_receive_raw_thread, NULL, do_receive_raw_data, NULL);

		pthread_join(do_receive_raw_thread, NULL);

		goto normal_exit;
	}

	if(receive_j1939) {
		if(FARO_CAN_SDK_DEBUG == 1)
			fprintf(stdout, "spawning do_receive_j1939 thread...\n");

		pthread_create(&do_receive_j1939_thread, NULL, do_receive_j1939_data, NULL);

		pthread_join(do_receive_j1939_thread, NULL);

		goto normal_exit;
	}

normal_exit:
	ret = pthread_join(deinit_wait_thread, NULL);
	if(ret) {
		fprintf(stderr, "deinit_wait thread error: %s\n", strerror(ret));
		goto err;
	}


	fprintf(stderr, "threads are stopped\n");
	fflush(stderr);

err:
	scan_port = 0;
	get_can_fw_ver = 0;
	can_config = 0;
	receive_raw = 0;
	receive_j1939 = 0;

	return ret;
}

int main(int argc, char *argv[])
{
	int ret;

	if(signal(SIGINT, sig_handler) == SIG_ERR)
		fprintf(stderr, "fail to catch SIGINT\n");

	if(process_options(argc, argv) == 0)
		do_functions();

    if(port_name != NULL)
    	free(port_name);

	return 0;
}
