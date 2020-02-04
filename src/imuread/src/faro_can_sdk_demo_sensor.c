/***********************************************************************
 *
 *  Copyright (c) 2017 - 2018 ANTZER TECH CO., LTD.
 *  All Rights Reserved
 *
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <linux/limits.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include "faro_can_sdk_demo_sensor_config.h"
#include "faro_can_sdk.h"

#define fprintf(fmt,...) {if (!g_quiet_print_flag) printf(__VA_ARGS__);}

#define FARO_CAN_SDK_SENSOR_READ_ARGS				(1)
#define FARO_CAN_SDK_SENSOR_WRITE_ARGS				(1 + 9)
#define FARO_CAN_SDK_SENSOR_GET_ACC_DATA			(1)
#define FARO_CAN_SDK_SENSOR_GET_ACC_INT_DATA		(1)
#define FARO_CAN_SDK_SENSOR_DO_ACC_CALIBRATION		(1)
#define FARO_CAN_SDK_SENSOR_CONFIG_ACC_INT			(1 + 7)
#define FARO_CAN_SDK_SENSOR_GET_GYRO_DATA			(1)
#define FARO_CAN_SDK_SENSOR_GET_GYRO_THS			(1)
#define FARO_CAN_SDK_SENSOR_GET_GYRO_DUR			(1)
#define FARO_CAN_SDK_SENSOR_DO_GYRO_CALIBRATION		(1)
#define FARO_CAN_SDK_SENSOR_SET_GYRO_THS			(1 + 9)
#define FARO_CAN_SDK_SENSOR_SET_GYRO_DUR			(1 + 4)
#define FARO_CAN_SDK_SENSOR_CONFIG_GYRO_INT			(1 + 6)

enum SENSOR_TEST_IDX
{
	SENSOR_TEST_IDX_READ = 0,
	SENSOR_TEST_IDX_WRITE,
	SENSOR_TEST_IDX_GET_ACC_DATA,
	SENSOR_TEST_IDX_GET_ACC_INT_DATA,
	SENSOR_TEST_IDX_DO_ACC_CALIBRATION,
	SENSOR_TEST_IDX_CONFIG_ACC_INT,
	SENSOR_TEST_IDX_GET_GYRO_DATA,
	SENSOR_TEST_IDX_GET_GYRO_THS,
	SENSOR_TEST_IDX_GET_GYRO_DUR,
	SENSOR_TEST_IDX_DO_GYRO_CALIBRATION,
	SENSOR_TEST_IDX_SET_GYRO_THS,
	SENSOR_TEST_IDX_SET_GYRO_DUR,
	SENSOR_TEST_IDX_CONFIG_GYRO_INT,
};

static int deinit_in_seconds = 10;
static int stop_thread = 0;
static char *port_name = NULL;
static uint8_t get_can_fw_ver = 0;
static uint8_t scan_port = 0;
static int test_cnt = 1;
static uint8_t mode_active = 0;
static int mode_active_arg = 1;
static uint8_t launch_receiver = 0;
static int sensor_test_arg = 0;
static uint8_t sensor_test = 0;

static struct sensor_ctrl_format_t sensor_ctrl_format;
static struct sensor_Gyro_format_t sensor_gyro_format;
static struct sensor_GyroDUR_format_t sensor_gyrodur_format;

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
			"  -m, --mode_active\n"
			"		#1-mode active: 0: RAW CAN, 1: OBDII, 2: J1939, 3: J1708\n"
			"  -x, --sensor_test\n"
			"		0 - read\n"
			"		1 - write\n"
			"			#1 - cmd: 1 byte, ex: 0x01\n"
			"			#2 - dlc_cfg: 1 byte, ex: 0x01\n"
			"			#3 - ctrl1: 1 byte, ex: 0x01\n"
			"			#4 - ctrl2: 1 byte, ex: 0x01\n"
			"			#5 - ctrl3: 1 byte, ex: 0x01\n"
			"			#6 - ctrl4: 1 byte, ex: 0x01\n"
			"			#7 - ctrl5: 1 byte, ex: 0x01\n"
			"			#8 - ctrl6: 1 byte, ex: 0x01\n"
			"			#9 - ctrl7: 1 byte, ex: 0x01\n"
			"		2 - get ACC data\n"
			"		3 - get ACC INT data\n"
			"		4 - do ACC calibration\n"
			"		5 - config ACC INT\n"
			"			#1 - cmd: 1 byte, ex: 0x54\n"
			"			#2 - dlc_cfg: 1 byte, ex: 0x05\n"
			"			#3 - INT1CFG: 1 byte, ex: 0x70\n"
			"			#4 - INT1CTRL: 1 byte, ex: 0x40\n"
			"			#5 - INT2CFG: 1 byte, ex: 0x00\n"
			"			#6 - INT2CTRL: 1 byte, ex: 0x00\n"
			"			#7 - INTMODE: 1 byte, ex: 0x84\n"
			"		6 - get Gyro data\n"
			"		7 - get Gyro THS\n"
			"		8 - get Gyro DUR\n"
			"		9 - do Gyro calibration\n"
			"		10 - set Gyro THS\n"
			"			#1 - cmd: 1 byte, ex: 0x43\n"
			"			#2 - dlc: 1 byte, ex: 0x07\n"
			"			#3 - dcrm: 0: reset, 1: DRCM, 1 byte, ex: 0x01\n"
			"			#4 - XX_L: 1 byte, ex: 0x00\n"
			"			#5 - XX_H: 1 byte, ex: 0x00\n"
			"			#6 - YY_L: 1 byte, ex: 0x00\n"
			"			#7 - YY_h: 1 byte, ex: 0x00\n"
			"			#8 - ZZ_L: 1 byte, ex: 0x00\n"
			"			#9 - ZZ_h: 1 byte, ex: 0x00\n"
			"		11 - set Gyro DUR\n"
			"			#1 - cmd: 1 byte, ex: 0x45\n"
			"			#2 - dlc: 1 byte, ex: 0x02\n"
			"			#3 - wait: 0: disable, 1: enable, 1 byte, ex: 0x01\n"
			"			#4 - data: 1 byte, ex: 0x01\n"
			"		12 - config Gyro INT\n"
			"			#1 - cmd: 1 byte, ex: 0x49\n"
			"			#2 - dlc_cfg: 1 byte, ex: 0x04\n"
			"			#3 - INT1CFG: 1 byte, ex: 0x00\n"
			"			#4 - INT1CTRL: 1 byte, ex: 0x00\n"
			"			#5 - INT2CFG: 1 byte, ex: 0x00\n"
			"			#6 - INT2CTRL: 1 byte, ex: 0x00\n"
			"  -l, --launch_receiver, using -d X to set waiting time in seconds\n"
			"  -t, --test_cnt, set counter for testing loop\n"
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
		static const char *short_options = "q:hp:sd:gm:x:lt:v";
		static const struct option long_options[] = {
			{"help", no_argument, 0, 'h'},
			{"port", required_argument, 0, 'p'},
			{"scan", no_argument, 0, 's'},
			{"deinit", required_argument, 0, 'd'},
			{"get_fw_ver", no_argument, 0, 'g'},
			{"mode_active", required_argument, 0, 'm'},
			{"sensor_test", required_argument, 0, 'x'},
			{"launch_receiver", no_argument, 0, 'l'},
			{"test_cnt", required_argument, 0, 't'},
			{"version", no_argument, 0, 'v'},
			{"quiet_mode", required_argument, 0, 'q'},
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
			#ifdef FARO_CAN_SDK_DEBUG
				fprintf(stdout, "deinit after %d seconds\n", deinit_in_seconds);
			#endif
			break;

		case 'g':
			get_can_fw_ver = 1;
			break;

		case 'm':
			mode_active = 1;
			mode_active_arg = atoi(optarg);
			if(mode_active_arg != 0 && mode_active_arg != 1 && mode_active_arg != 3) {
				fprintf(stderr, "invalid argument, %d, should be 0, 1, 2 or 3\n", mode_active_arg);
				display_help(argc, argv);
			}
			break;

		case 'x':
			{
				int cnt = 0;
				int index = optind - 1;
				int arg_num = 0;

				sensor_test = 1;

				while(index < argc) {
					if(argv[index][0] == '-') {
						break;
					}

					#ifdef FARO_CAN_SDK_DEBUG
						fprintf(stdout, "argv[%d] = %s\n", index, argv[index]);
					#endif

					tmp = 0;

					switch(cnt) {
						case 0:
							fprintf(stdout,"case 0 start\n");
							sensor_test_arg = atoi(argv[index]);
							if(sensor_test_arg < SENSOR_TEST_IDX_READ ||
								sensor_test_arg > SENSOR_TEST_IDX_CONFIG_GYRO_INT)
								goto x_err;

							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								memset(&sensor_ctrl_format, 0x00, (sizeof(struct sensor_ctrl_format_t)/sizeof(uint8_t)));
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								memset(&sensor_gyro_format, 0x00, (sizeof(struct sensor_Gyro_format_t)/sizeof(uint8_t)));
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_DUR) {
								memset(&sensor_gyrodur_format, 0x00, (sizeof(struct sensor_GyroDUR_format_t)/sizeof(uint8_t)));
							}
							fprintf(stdout,"case 0 break\n");
							break;

						case 1:
							fprintf(stdout,"case 1 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.cmd = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.cmd = tmp;
							}  else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_DUR) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyrodur_format.cmd = tmp;
							}
							fprintf(stdout,"case 1 break\n");
							break;

						case 2:
							fprintf(stdout,"case 2 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.dlc_cfg = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.dlc_cfg = tmp;
							}  else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_DUR) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyrodur_format.dlc_cfg = tmp;
							}
							fprintf(stdout,"case 2 break\n");
							break;

						case 3:
							fprintf(stdout,"case 3 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl1 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.dcrm = tmp;
							}  else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_DUR) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyrodur_format.wait = tmp;
							}
							fprintf(stdout,"case 3 break\n");
							break;

						case 4:
							fprintf(stdout,"case 4 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl2 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.xx_L = tmp;
							}  else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_DUR) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyrodur_format.durData = tmp;
							}
							fprintf(stdout,"case 4 break\n");
							break;

						case 5:
							fprintf(stdout,"case 5 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl3 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.xx_H = tmp;
							}
							fprintf(stdout,"case 5 break\n");
							break;

						case 6:
							fprintf(stdout,"case 6 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_GYRO_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl4 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.yy_L = tmp;
							}
							fprintf(stdout,"case 6 break\n");
							break;

						case 7:
							fprintf(stdout,"case 7 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE ||
								sensor_test_arg == SENSOR_TEST_IDX_CONFIG_ACC_INT) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl5 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.yy_H = tmp;
							}
							fprintf(stdout,"case 7 break\n");
							break;

						case 8:
							fprintf(stdout,"case 8 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl6 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.zz_L = tmp;
							}
							fprintf(stdout,"case 8 break\n");
							break;

						case 9:
							fprintf(stdout,"case 9 start\n");
							if(sensor_test_arg == SENSOR_TEST_IDX_WRITE) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_ctrl_format.ctrl7 = tmp;
							} else if(sensor_test_arg == SENSOR_TEST_IDX_SET_GYRO_THS) {
								sscanf(argv[index], "0x%02X", &tmp);
								sensor_gyro_format.zz_H = tmp;
							}
							fprintf(stdout,"case 9 break\n");
							break;

						default:
							goto x_err;
					}

					index++;
					cnt++;
				};

				switch(sensor_test_arg) {
					case SENSOR_TEST_IDX_READ:					arg_num = FARO_CAN_SDK_SENSOR_READ_ARGS;			break;
					case SENSOR_TEST_IDX_WRITE:					arg_num = FARO_CAN_SDK_SENSOR_WRITE_ARGS;			break;
					case SENSOR_TEST_IDX_GET_ACC_DATA:			arg_num = FARO_CAN_SDK_SENSOR_GET_ACC_DATA;			break;
					case SENSOR_TEST_IDX_GET_ACC_INT_DATA:		arg_num = FARO_CAN_SDK_SENSOR_GET_ACC_INT_DATA;		break;
					case SENSOR_TEST_IDX_DO_ACC_CALIBRATION:	arg_num = FARO_CAN_SDK_SENSOR_DO_ACC_CALIBRATION;	break;
					case SENSOR_TEST_IDX_CONFIG_ACC_INT:		arg_num = FARO_CAN_SDK_SENSOR_CONFIG_ACC_INT;		break;
					case SENSOR_TEST_IDX_GET_GYRO_DATA:			arg_num = FARO_CAN_SDK_SENSOR_GET_GYRO_DATA;		break;
					case SENSOR_TEST_IDX_GET_GYRO_THS:			arg_num = FARO_CAN_SDK_SENSOR_GET_GYRO_THS;			break;
					case SENSOR_TEST_IDX_GET_GYRO_DUR:			arg_num = FARO_CAN_SDK_SENSOR_GET_GYRO_DUR;			break;
					case SENSOR_TEST_IDX_DO_GYRO_CALIBRATION:	arg_num = FARO_CAN_SDK_SENSOR_DO_GYRO_CALIBRATION;	break;
					case SENSOR_TEST_IDX_SET_GYRO_THS:			arg_num = FARO_CAN_SDK_SENSOR_SET_GYRO_THS;			break;
					case SENSOR_TEST_IDX_SET_GYRO_DUR:			arg_num = FARO_CAN_SDK_SENSOR_SET_GYRO_DUR;			break;
					case SENSOR_TEST_IDX_CONFIG_GYRO_INT:		arg_num = FARO_CAN_SDK_SENSOR_CONFIG_GYRO_INT;		break;
				}

				if(cnt != arg_num)
					goto x_err;

				optind = index - 1;
				goto x_ok;

x_err:
				fprintf(stderr, "invalid argument number, should be %d\n", arg_num);
				display_help(argc, argv);

x_ok:
				#ifdef FARO_CAN_SDK_DEBUG
					fprintf(stdout, "checking -x arguments done\n");
				#endif
			}
			break;

		case 'l':
			launch_receiver = 1;
			break;

		case 't':
			test_cnt = atoi(optarg);
			if(test_cnt <= 0)
				test_cnt = 1;
			break;

		case 'v':
			fprintf(stdout, "%s Version %d.%d.%d\n",
				argv[0],
				FARO_CAN_SDK_DEMO_SENSOR_VERSION_MAJOR,
				FARO_CAN_SDK_DEMO_SENSOR_VERSION_MINOR,
				FARO_CAN_SDK_DEMO_SENSOR_VERSION_PATCH);
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

	if(launch_receiver) {
		AZ_VC_CAN_Dump_Pkt_Cnt();
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

	#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "starting get firmware version on %s\n", port_name);
	#endif

	if(port_name == NULL) {
		fprintf(stderr, "invalid port_name, assigned by -p /dev/ttyUSBx\n");
		goto err;
	}

	// initialize
	if((ret = AZ_VC_Init(port_name)) != 0)
		goto err;

	for(i=0; i<test_cnt; i++) {
		if(stop_thread != 0)
			break;

		memset(&can_fw_ver, 0x00, sizeof(can_fw_ver));
		if((ret = AZ_VC_GetFWVer(&can_fw_ver)) != 0) {
			fprintf(stderr, "Calling AZ_VC_GetFWVer fail, test_cnt = %d\n", i);
			continue;
		}

		fprintf(stdout, "test_cnt = %d, can_fw_ver: %02X.%02X.%02X\n",
			i, can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

err:
	pthread_exit(NULL);
}

static void *do_mode_active(void *args)
{
	int ret = 0;
	int i;

	#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "starting do_mode_active on %s\n", port_name);
	#endif

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

		fprintf(stdout, "can_fw_ver: %02X.%02X.%02X\n", can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

	for(i=0; i<test_cnt; i++) {
		if(stop_thread != 0)
			break;

		if((ret = AZ_VC_ModeActive(mode_active_arg)) != 0) {
			fprintf(stderr, "calling AZ_VC_ModeActive fail\n");
			goto err;
		}

		fprintf(stdout, "test_cnt = %d, mode_active_arg = %d\n", i, mode_active_arg);
	}

err:
	pthread_exit(NULL);
}

static void *do_launch_receiver(void *args)
{
	int ret = 0;
	int i;

	#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "starting do_launch_receiver on %s\n", port_name);
	#endif

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

		fprintf(stdout, "can_fw_ver: %02X.%02X.%02X\n", can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

	AZ_VC_CAN_Start_Pkt_Consumer(start_consume_pkt_on);

err:
	pthread_exit(NULL);
}

static void dump_sensor_ctrl_format(struct sensor_ctrl_format_t msg)
{
	fprintf(stdout, "cmd = 0x%02X, dlc = 0x%02X --> \n", msg.cmd, msg.dlc_cfg);
	switch(msg.cmd) {
		case 0x91:
			fprintf(stdout, "\tXX_L = 0x%02X, XX_H = 0x%02X, YY_L = 0x%02X, YY_H = 0x%02X, ZZ_L = 0x%02X, ZZ_H = 0x%02X\n",
				msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4, msg.ctrl5, msg.ctrl6);
			break;

		case 0x92:
			fprintf(stdout, "\tTHS1 = 0x%02X, THS2 = 0x%02X, DUR1 = 0x%02X, DUR2 = 0x%02X\n",
				msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4);
			break;

		case 0x93:
			fprintf(stdout, "\tINT_SRC = 0x%02X, STATUS = 0x%02X, INT1_SRC = 0x%02X, INT2_SRC = 0x%02X\n",
				msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4);
			break;

		case 0x94:
			fprintf(stdout, "\tAddr = 0x%02X, Data = 0x%02X\n",
				msg.ctrl1, msg.ctrl2);
			break;

		case 0x8A:
			fprintf(stdout, "\tXX_L = 0x%02X, XX_H = 0x%02X, YY_L = 0x%02X, YY_H = 0x%02X, ZZ_L = 0x%02X, ZZ_H = 0x%02X\n",
				msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4, msg.ctrl5, msg.ctrl6);
			break;

		case 0x8B:
			fprintf(stdout, "\tDCRM = 0x%02X, XX_L = 0x%02X, XX_H = 0x%02X, YY_L = 0x%02X, YY_H = 0x%02X, ZZ_L = 0x%02X, ZZ_H = 0x%02X\n",
				msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4, msg.ctrl5, msg.ctrl6, msg.ctrl7);
			break;

		case 0x8C:
			fprintf(stdout, "\tWait = 0x%02X, Data = 0x%02X\n",
				msg.ctrl1, msg.ctrl2);
			break;

		case 0x8D:
			fprintf(stdout, "\tAddr = 0x%02X, Data = 0x%02X\n",
				msg.ctrl1, msg.ctrl2);
			break;

		default:
			fprintf(stdout, "\tctrl1 = 0x%02X, ctrl2 = 0x%02X, ctrl3 = 0x%02X, ctrl4 = 0x%02X, ctrl5 = 0x%02X, ctrl6 = 0x%02X, ctrl7 = 0x%02X\n",
				msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4, msg.ctrl5, msg.ctrl6, msg.ctrl7);

	}
}

static void *do_sensor_test(void *args)
{
	int ret = 0;

	#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "starting do_sensor_test on %s\n", port_name);
	#endif

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

		fprintf(stdout, "can_fw_ver: %02X.%02X.%02X\n", can_fw_ver.main, can_fw_ver.minor, can_fw_ver.type);
	}

	// change mode to 0 for sensor testing
	ret = AZ_VC_ModeActive(0);
	if(ret != 0) {
		fprintf(stderr, "Calling AZ_VC_ModeActive fail\n");
		goto err;
	}

	switch(sensor_test_arg) {
		case SENSOR_TEST_IDX_READ:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;
				int read_packet_cnt = 0;

				// CMD 0x51: 100255000000000000000000000000000000001003 (Do ACC calibration)
				ret = AZ_VC_ACCCalibration();
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_ACCCalibration fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_ACCCalibration successful\n");
				}

				// CMD 0x53: 100253011000000000000000000000000000001003 (INT1_THS=250mg)
				memset(&msg, 0x00, sizeof(msg));
				msg.cmd = 0x53;
				msg.dlc_cfg = 0x01;
				msg.ctrl1 = 0x10;

				ret = AZ_VC_Sensor_Write(msg);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_Sensor_Write, cmd = 0x%02X, fail\n", msg.cmd);
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_Sensor_Write, cmd = 0x%02X, successful\n", msg.cmd);
				}

				// CMD 0x54: 100254057040000084000000000000000000001003 (INT1 high active, 6-direction movement recognition)
				memset(&msg, 0x00, sizeof(msg));
				msg.cmd = 0x54;
				msg.dlc_cfg = 0x05;
				msg.ctrl1 = 0x70;
				msg.ctrl2 = 0x40;
				msg.ctrl3 = 0x00;
				msg.ctrl4 = 0X00;
				msg.ctrl5 = 0x84;

				ret = AZ_VC_ConfigACCINT(msg);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_ConfigACCINT fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_ConfigACCINT successful\n");
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						fprintf(stdout, "Read packet cnt = %d\n", ++read_packet_cnt);
						dump_sensor_ctrl_format(msg);
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_WRITE:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;

				ret = AZ_VC_Sensor_Write(sensor_ctrl_format);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_Sensor_Write fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_Sensor_Write successful\n");
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						dump_sensor_ctrl_format(msg);
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_GET_ACC_DATA:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;

				ret = AZ_VC_GetACCData();
				if(ret !=  0) {
					fprintf(stderr, "Calling AZ_VC_GetACCData fail\n");
					goto err;
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						if(msg.cmd == 0x91) {
							fprintf(stdout, "ACC data:\n");
							dump_sensor_ctrl_format(msg);
						} else {
							fprintf(stderr, "Unexpect cmd (0x%02X)\n", msg.cmd);
							dump_sensor_ctrl_format(msg);
						}
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_GET_ACC_INT_DATA:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;

				ret = AZ_VC_GetACCINTData();
				if(ret !=  0) {
					fprintf(stderr, "Calling AZ_VC_GetACCINTData fail\n");
					goto err;
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						if(msg.cmd == 0x92) {
							fprintf(stdout, "ACC INT Data:\n");
							dump_sensor_ctrl_format(msg);
						} else {
							fprintf(stderr, "Unexpect cmd (0x%02X)\n", msg.cmd);
							dump_sensor_ctrl_format(msg);
						}
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_DO_ACC_CALIBRATION:
			{
				ret = AZ_VC_ACCCalibration();
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_ACCCalibration fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_ACCCalibration successful\n");
				}
			}
			break;

		case SENSOR_TEST_IDX_CONFIG_ACC_INT:
			{
				ret = AZ_VC_ConfigACCINT(sensor_ctrl_format);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_ConfigACCINT fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_ConfigACCINT successful\n");
				}
			}
			break;

		case SENSOR_TEST_IDX_GET_GYRO_DATA:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;

				ret = AZ_VC_GetGyroData();
				if(ret !=  0) {
					fprintf(stderr, "Calling AZ_VC_GetGyroData fail\n");
					goto err;
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						if(msg.cmd == 0x8A) {
							fprintf(stdout, "Gyro data:\n");
							dump_sensor_ctrl_format(msg);
						} else {
							fprintf(stderr, "Unexpect cmd (0x%02X)\n", msg.cmd);
							dump_sensor_ctrl_format(msg);
						}
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_GET_GYRO_THS:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;

				ret = AZ_VC_GetGyroTHS();
				if(ret !=  0) {
					fprintf(stderr, "Calling AZ_VC_GetGyroTHS fail\n");
					goto err;
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						if(msg.cmd == 0x8B) {
							fprintf(stdout, "Gyro THS:\n");
							dump_sensor_ctrl_format(msg);
						} else {
							fprintf(stderr, "Unexpect cmd (0x%02X)\n", msg.cmd);
							dump_sensor_ctrl_format(msg);
						}
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_GET_GYRO_DUR:
			{
				struct sensor_ctrl_format_t msg;
				uint32_t read_size;

				ret = AZ_VC_GetGyroDUR();
				if(ret !=  0) {
					fprintf(stderr, "Calling AZ_VC_GetGyroDUR fail\n");
					goto err;
				}

				while(stop_thread == 0) {
					memset(&msg, 0x00, sizeof(msg));
					read_size = 0;

					ret = AZ_VC_Sensor_Read(&msg, &read_size);
					if(ret == 0) {
						if(msg.cmd == 0x8C) {
							fprintf(stdout, "Gyro DUR:\n");
							dump_sensor_ctrl_format(msg);
						} else {
							fprintf(stderr, "Unexpect cmd (0x%02X)\n", msg.cmd);
							dump_sensor_ctrl_format(msg);
						}
					}

					usleep(1);
				}
			}
			break;

		case SENSOR_TEST_IDX_DO_GYRO_CALIBRATION:
			{
				ret = AZ_VC_GyroCalibration();
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_GyroCalibration fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_GyroCalibration successful\n");
				}
			}
			break;

		case SENSOR_TEST_IDX_SET_GYRO_THS:
			{
				ret = AZ_VC_SetGyroTHS(sensor_gyro_format);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_SetGyroTHS fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_SetGyroTHS successful\n");
				}
			}
			break;

		case SENSOR_TEST_IDX_SET_GYRO_DUR:
			{
				ret = AZ_VC_SetGyroDUR(sensor_gyrodur_format);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_SetGyroDUR fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_SetGyroDUR successful\n");
				}
			}
			break;

		case SENSOR_TEST_IDX_CONFIG_GYRO_INT:
			{
				ret = AZ_VC_ConfigGyroINT(sensor_ctrl_format);
				if(ret != 0) {
					fprintf(stderr, "Calling AZ_VC_ConfigGyroINT fail\n");
					goto err;
				} else {
					fprintf(stdout, "Calling AZ_VC_ConfigGyroINT successful\n");
				}
			}
			break;

		default:
			fprintf(stderr, "Invalid arguments\n");
			goto err;
	}

err:
	pthread_exit(NULL);
}

static int do_functions(void)
{
	int ret = 0;
	pthread_t deinit_wait_thread;
	pthread_t do_get_can_fw_ver_thread;
	pthread_t do_mode_active_thread;
	pthread_t do_launch_receiver_thread;
	pthread_t do_sensor_test_thread;

	fprintf(stdout,"version %d.%d.%d\n",
		FARO_CAN_SDK_DEMO_SENSOR_VERSION_MAJOR,
		FARO_CAN_SDK_DEMO_SENSOR_VERSION_MINOR,
		FARO_CAN_SDK_DEMO_SENSOR_VERSION_PATCH);

	if(scan_port) {
		if((ret = do_scan_port()) != 0) {
			fprintf(stderr, "do_scan_port fail\n");
		}

		goto err;
	}

	// spawn deinit_wait_thread if necessary
	#ifdef FARO_CAN_SDK_DEBUG
		fprintf(stdout, "spawning deinit_wait thread...\n");
	#endif

	if((ret = pthread_create(&deinit_wait_thread, NULL, deinit_wait, NULL)) != 0) {
		fprintf(stderr, "create deinit_wait thread fail\n");
			goto err;
	}

	if(get_can_fw_ver) {
		#ifdef FARO_CAN_SDK_DEBUG
			fprintf(stdout, "spawning do_get_can_fw_ver thread...\n");
		#endif

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

	if(mode_active) {
		#ifdef FARO_CAN_SDK_DEBUG
			fprintf(stdout, "spawning do_mode_active thread...\n");
		#endif

		if((ret = pthread_create(&do_mode_active_thread, NULL, do_mode_active, NULL)) != 0) {
			fprintf(stderr, "create do_mode_active thread fail\n");
			goto err;
		}

		ret = pthread_join(do_mode_active_thread, NULL);
		if(ret) {
			fprintf(stderr, "do_mode_active_thread error: %s\n", strerror(ret));
			goto err;
		}

		goto normal_exit;
	}

	if(launch_receiver) {
		#ifdef FARO_CAN_SDK_DEBUG
			fprintf(stdout, "spawning do_launch_receiver thread...\n");
		#endif

		if((ret = pthread_create(&do_launch_receiver_thread, NULL, do_launch_receiver, NULL)) != 0) {
			fprintf(stderr, "create do_launch_receiver thread fail\n");
			goto err;
		}

		ret = pthread_join(do_launch_receiver_thread, NULL);
		if(ret) {
			fprintf(stderr, "do_launch_receiver_thread error: %s\n", strerror(ret));
			goto err;
		}

		goto normal_exit;
	}

	if(sensor_test) {
		#ifdef FARO_CAN_SDK_DEBUG
			fprintf(stdout, "spawn do_sensor_test thread...\n");
		#endif

		if((ret = pthread_create(&do_sensor_test_thread, NULL, do_sensor_test, NULL)) != 0) {
			fprintf(stderr, "create do_sensor_test thread fail\n");
			goto err;
		}

		ret = pthread_join(do_sensor_test_thread, NULL);
		if(ret) {
			fprintf(stderr, "do_sensor_test error: %s\n", strerror(ret));
			goto err;
		}

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
	mode_active = 0;
	launch_receiver = 0;
	sensor_test = 0;

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
