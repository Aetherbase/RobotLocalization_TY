/***********************************************************************
 *
 *  Copyright (c) 2017 - 2018 ANTZER TECH CO., LTD.
 *  All Rights Reserved
 *
 ************************************************************************/

/**
 * header file for FARO CAN SDK
 */
#ifndef _FARO_CAN_SDK_H
#define _FARO_CAN_SDK_H

#include <stdint.h>

#define FARO_CAN_SDK_MSGQ_NODE									"/tmp/faro_can_sdk_msg_queue"
#define FARO_CAN_SDK_MSGQ_QUEUE_ID								'F'

extern uint8_t g_quiet_print_flag;	//0: Normal,1: Quiet


struct can_msg_payload_t {
	long mtype;
	uint8_t data[22];
};

struct msgq_node_info_t {
	char *node;
	char queue_id;
};

struct can_fw_ver_t {
	uint8_t main;
	uint8_t minor;
	uint8_t type;

	uint8_t sub1;
	uint8_t sub2;
	uint8_t rc;
};

enum can_speed {
	CAN_SPEED_1M = 0,
	CAN_SPEED_800K = 1,
	CAN_SPEED_500K = 2,
	CAN_SPEED_250K = 3,
	CAN_SPEED_200K = 4,
	CAN_SPEED_125K = 5,
};

// port = 0 or 1
// speed = CAN_SPEED_1M to CAN_SPEED_125K
struct can_config_t {
	uint8_t port;
	enum can_speed speed;
};

struct can_message_t {
	uint8_t port;
	uint32_t id;
	uint8_t mode; // CAN 2.0A/B (STD/Extended)
	uint8_t rtr; // Remote request
	uint8_t dlc;
	uint8_t data[8];
};

struct can_filter_config_t {
	uint8_t type; // 0 - ID+Mask, 1 - ID List, 2 - Remove, 3 - Reset All
	uint8_t port;
	uint8_t bank;
	uint8_t mode;
	uint32_t filterId;
	uint32_t filterMask;
};

enum start_consume_pkt_t {
	start_consume_pkt_off = 0,
	start_consume_pkt_on = 1,
};

struct J1939_message_t {
	uint32_t id;
	uint8_t p;			// priority
	uint8_t dp;			// data page
	uint8_t pf;			// PDU format
	uint8_t ps;			// PDU Specifc Field
	uint8_t sa;			// Source Address
	uint32_t pgn;		// Parametr Group Number
	uint8_t dlc;
	uint8_t data[8];
};

struct J1939_send_msg_t {
	uint8_t port;
	uint8_t p;			// priority
	uint8_t sa;			// Source Address
	uint32_t pgn;		// Parametr Group Number
	uint8_t dlc;
	uint8_t data[8];	// data
};

struct J1708_message_t {
	uint8_t mid;		// 0-255
	uint8_t p;			// priority (1-8)
	uint8_t dlc;		// 0-18
	uint8_t data[19];
};

struct sensor_ctrl_format_t {
	uint8_t cmd;
	uint8_t dlc_cfg;	// DLC or configuration
	uint8_t ctrl1;
	uint8_t ctrl2;
	uint8_t ctrl3;
	uint8_t ctrl4;
	uint8_t ctrl5;
	uint8_t ctrl6;
	uint8_t ctrl7;
	uint8_t reserved[8];
};

struct sensor_Gyro_format_t {
	uint8_t cmd;
	uint8_t dlc_cfg;	// DLC or configuration
	uint8_t dcrm;
	uint8_t xx_H;
	uint8_t xx_L;
	uint8_t yy_H;
	uint8_t yy_L;
	uint8_t zz_H;
	uint8_t zz_L;
	uint8_t reserved[8];
};

struct sensor_GyroDUR_format_t {
	uint8_t cmd;
	uint8_t dlc_cfg;	// DLC or configuration
	uint8_t wait;
	uint8_t durData;
	uint8_t reserved[13];
};

extern uint8_t g_quiet_print_flag;	//0: Normal,1: Quite

/**
 * @brief Get CAN bus module serial port name and amount
 * @param lpszCommPort all of serial port name string and separated character is ‘\n’, e.g. “/dev/ttyUSB0\n/dev/ttyUSB1\n…"
 * @param portCnt amount of serial port
 * @return Success – ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_GetCommPort(char *lpszCommPort, int *portCnt);

/**
 * @brief Serial port open, initialization & configuration
 * @param lpszDevice serial port name string
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_Init(char *lpszDevice);

/*!
 * @breif Serial port close & de-initialization
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_DeInit(void);

/**
 * @breif Get Faro CAN bus module firmware version
 * @parm canfwver 3 bytes, 1st byte is major version, 2nd byte is minor version and 3rd byte is module type (f1 or f4), e.g. 1.0.f1
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_GetFWVer(struct can_fw_ver_t *canfwver);

/**
 * @brief Set CAN bus configurations
 * @param canCFG canCFG.port = 0 or 1, canCFG.speed = 0 ~ 5
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_CAN_Config(struct can_config_t canCFG);

/**
 * @brief The SDK support message queue method to read CAN bus data and set wait time //byTony/19.09.05
 * @param candata total 16 bytes
 *                        1st byte is CAN bus port 0 or 1
 *                        2nd byte is raw CAN data ID
 *                        3rd byte is CAN 2.0A (=0, standard) or B (=1, extended) mode
 *                        4th byte is RTR (Remote Request) mode (=1 RTR, =0 None)
 *                        5th byte is raw CAN data length DLC
 *                        the last 8 bytes are raw CAN data.
 * @param dwread actual reading bytes
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_CAN_Read_wait(struct can_message_t *candata, uint32_t *dwread, uint16_t wait_ms_time);

/**
 * @brief The SDK support message queue method to read CAN bus data
 * @param candata total 16 bytes
 *                        1st byte is CAN bus port 0 or 1
 *                        2nd byte is raw CAN data ID
 *                        3rd byte is CAN 2.0A (=0, standard) or B (=1, extended) mode
 *                        4th byte is RTR (Remote Request) mode (=1 RTR, =0 None)
 *                        5th byte is raw CAN data length DLC
 *                        the last 8 bytes are raw CAN data.
 * @param dwread actual reading bytes
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_CAN_Read(struct can_message_t *candata, uint32_t *dwread);

/**
 * @brief Write raw CAN data to CAN bus controller
 * @param candata candata.port = 0 or 1
 *                candata.id = 0xXXXXXXXX
 *                candata.mode = 0 or 1
 *                candata.rtr = 0 or 1
 *                candata.dlc = 1 ~ 8
 *                candata.data[0 ~ 7] = the raw CAN data would be transmitted
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_CAN_Write(struct can_message_t candata);

/**
 * @brief Set CAN bus controller filters configuration
 * @param canFIlterCFG canFilterCFG.type = 0 ~ 3
 *                     canFilterCFG.port = 0 or 1
 *                     canFilterCFG.bank = 0 ~ 13 (F1), 0 ~ 27 (F4)
 *                     canFilterCFG.mode = 0 (CAN 2.0A) or 1 (CAN 2.0B)
 *                     canFilterCFG.filterId = CAN bus controller ID filter configuration value
 *                     canFilterCFG.filterMask = CAN bus controller ID filter mask configuration value
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_CAN_Filter_Config(struct can_filter_config_t canFilterCFG);

/**
 * @brief Retrieve OBDII or J1939 supporting status
 * @param mode 0x01 retrieve OBDII mode, 0x02 retrieve J1939 mode
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_ModeActive(uint8_t mode);

/**
 * @brief Get CAN bus controller filters configuration
 * @param canFilterCFG canFilterCFG.type = 0 ~ 3
 *                     canFilterCFG.port = 0 or 1
 *                     canFilterCFG.bank = 0 ~ 13 (F1), 0 ~ 27 (F4)
 *                     canFilterCFG.mode = 0 (CAN 2.0A) or 1 (CAN 2.0B)
 *                     canFilterCFG.filterId = CAN bus controller ID filter configuration value
 *                     canFilterCFG.filterMask = CAN bus controller ID filter mask configuration value
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 */
int AZ_VC_CAN_Filter_Get(struct can_filter_config_t *canFilterCFG);

/**
 * @brief start packet consumer, for debug purpose
 * @param consume_pkt
 */
void AZ_VC_CAN_Start_Pkt_Consumer(enum start_consume_pkt_t consume_pkt);

/**
 * @brief dump received packages, for debug purpose
 */
void AZ_VC_CAN_Dump_Pkt_Cnt(void);

/**
 * @brief read J1939 data
 * @param J1939data (output) – total 22 bytes
 *		1st ~ 4th byte is J1939 ID
 *		5th byte is J1939 data priority (0 ~ 7)
 *		6th byte is J1939 data page (0 or 1)
 *		7th byte is PDU format (PDU1 = 0 ~ 239, PDU3 = 240 ~ 255)
 *		8th byte is PDU specific field
 *		9th byte is source address (unique address)
 *		10th ~ 13th bytes are Parameter group number (total 3 bytes, consist of dp, pf and ps)
 *		14th byte is raw CAN data length DLC, the last 8 bytes are raw CAN data,
 *			refer to J1939 standard SAE J1939-21 for more detail information.
 * @param dwread (output) - actual reading bytes
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1939_Read(struct J1939_message_t *j1939data, uint32_t *dwread);

/**
 * @brief Write J1939 data to CAN bus controller
 * @param J1939data (output) – total 22 bytes
 *		1st ~ 4th byte is J1939 ID
 *		5th byte is J1939 data priority (0 ~ 7)
 *		6th byte is J1939 data page (0 or 1)
 *		7th byte is PDU format (PDU1 = 0 ~ 239, PDU3 = 240 ~ 255)
 *		8th byte is PDU specific field
 *		9th byte is source address (unique address)
 *		10th ~ 13th bytes are Parameter group number (total 3 bytes, consist of dp, pf and ps)
 *		14th byte is raw CAN data length DLC, the last 8 bytes are raw CAN data,
 *			refer to J1939 standard SAE J1939-21 for more detail information.
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1939_Write(struct J1939_send_msg_t j1939data);

/**
 * @brief Configuration J1939 transmission port
 * @param port - 0 or 1 CAN bus communication port
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1939Port_Config(uint8_t port);

/**
 * @brief Configure J1939 filter settings
 * @param type - 0 - (ID + Mask), 1 - ID List, 2 - Remove one filter, 3 - Reset all filters
 * @param port
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1939Filter_Config(uint8_t type, uint8_t port, uint32_t pgn);

/**
 * @brief Get J1939 current filter amount
 * @param port - 0 or 1 CAN bus communication port
 * @param count - J1939 current filter amount
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1939_get_filter_count(uint8_t port, uint16_t *count);

/**
 * @brief Get J1939 current filter
 * @param port - 0 or 1 CAN bus communication port
 * @param filter - [0 ~ total_count] = PGN number
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1939Filter_Get(uint8_t port, uint32_t *filter);

/**
 * @brief Reset module
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_Reset_Module(void);

/**
 * @brief read J1708 data
 * @param data - buffer to read
 * @param dwread - actual reading bytes
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1708_Read(struct J1708_message_t *data, uint32_t *dwread);

/**
 * @brief write J1708 data to J1708 controller
 * @param data - buffer to write
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1708_Write(struct J1708_message_t data);

/**
 * @brief configure J1708 filter setting
 * @param type - type
 * @param mid - mid
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1708_FilterConfig(uint8_t type, uint8_t mid);

/**
 * @brief get filter count
 * @param count - J1708 current filter amount
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1708_GetFilterCount(uint16_t *count);

/**
 * @brief get J1708 current J1708 filter amount. It must call AZ_VC_J1708_GetFilterCount API first before calling this.
 * @param count - J1708 current filter amount
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_J1708_FilterGet(uint32_t *filter);

/**
 * @brief read ACC or Gyro sensor register data from Faro module
 * @param senData - buffer to read
 * @param dwread - actual reading bytes
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_Sensor_Read(struct sensor_ctrl_format_t *senData, uint32_t *dwread);

/**
 * @brief write ACC or Gyro sensor register data from Faro module
 * @param event type
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_Sensor_Write(struct sensor_ctrl_format_t senData);

/**
 * @brief Get ACC sensor data from Faro module
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_GetACCData(void);

/**
 * @brief Get ACC sensor interrupt data from Faro module
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_GetACCINTData(void);

/**
 * @brief Set ACC sensor interrupt configuration data from Faro module
 * @param ACCINTData - config data
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_SetACCINTData(struct sensor_ctrl_format_t ACCINTData);

/**
 * @brief Ask Faro module ACC sensor to do calibration
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_ACCCalibration(void);

/**
 * @brief Configure and enable ACC sensor interrupt
 * @param ACCINTData - config data
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_ConfigACCINT(struct sensor_ctrl_format_t ACCINTCtrl);

/**
 * @brief Get Gyro sensor data from Faro module
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_GetGyroData(void);

/**
 * @brief Get Gyro sensor threshold setting from Faro module
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_GetGyroTHS(void);

/**
 * @brief Get Gyro sensor duration settings from FARO module
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_GetGyroDUR(void);

/**
 * @brief Ask Faro module Gyro sensor to do calibration
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_GyroCalibration(void);

/**
 * @brief Set Gyro sensor threshold data to Faro module
 * @param GyroData - config data
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_SetGyroTHS(struct sensor_Gyro_format_t GyroData);

/**
 * @brief Set Gyro sensor duration data to Faro module
 * @param GyroData - config data
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_SetGyroDUR(struct sensor_GyroDUR_format_t GyroData);

/**
 * @brief Configure and enable Gyro sensor interrupt
 * @param GyroCtrl - config data
 * @return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
int AZ_VC_ConfigGyroINT(struct sensor_ctrl_format_t GyroCtrl);

/**
 * @//////////brief Configure and enable Gyro sensor interrupt
 * @//////////param GyroCtrl - config data
 * @//////////return ERROR_SUCCESS (=0), Failure – ERROR_ACCESS_DENIED and others (!=0)
 **/
void AZ_VC_quiet_mode(uint8_t quiet);

#endif
