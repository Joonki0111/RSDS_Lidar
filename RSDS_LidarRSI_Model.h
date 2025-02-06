/**
 * Description : Lidar RSI Sensor
 *
 * COPYRIGHT 2023, IPG Automotive Korea.
 */
#ifndef LIDARSIMMODEL_H_
#define LIDARSIMMODEL_H_

#include <stdint.h>

#include <CarMaker.h>
#include <Car/Car.h>
#include <Vehicle/Sensor_LidarRSI.h>
#include "rsds-client.h"
#include <stdbool.h> //HJK_250116
#include <limits.h> //HJK_250116
#include <unistd.h> //HJK_250116

#define LIDAR_MAX_SCANPOINT_SIZE 270000

#define VLP_16_LIDAR_DATAPACKET_SIZE 12
#define VLP_16_LIDAR_DATABLOCK_SIZE 32
#define VLP_16_LIDAR_SENDPOINT_SIZE 384
#define VLP_16_NUMBER_OF_POINTS 28800
#define VLP_16_NUMBER_OF_PACKET 75
#define VLP_16_NUMBER_OF_SCAN 10
#define VLP_16_POINTS_OF_SCAN 2880
#define VLP_16_Distance_Res 2
#define VLP_16_Angle_Res 40 // 0.4 deg

#define VLP_32_LIDAR_DATAPACKET_SIZE 12
#define VLP_32_LIDAR_DATABLOCK_SIZE 32
#define VLP_32_LIDAR_SENDPOINT_SIZE 384
#define VLP_32_NUMBER_OF_POINTS 57600
#define VLP_32_NUMBER_OF_PACKET 150
#define VLP_32_NUMBER_OF_SCAN 10
#define VLP_32_POINTS_OF_SCAN 5760
#define VLP_32_Distance_Res 4
#define VLP_32_Angle_Res 20 // 0.2 deg

#define VLP_64_LIDAR_DATAPACKET_SIZE 12
#define VLP_64_LIDAR_DATABLOCK_SIZE 32
#define VLP_64_LIDAR_SENDPOINT_SIZE 384
#define VLP_64_NUMBER_OF_POINTS 115200
#define VLP_64_NUMBER_OF_PACKET 600
#define VLP_64_Distance_Res 4
#define VLP_64_Angle_Res 20 // 0.2 deg

#define VLS_128_LIDAR_DATAPACKET_SIZE 12
#define VLS_128_LIDAR_DATABLOCK_SIZE 32
#define VLS_128_LIDAR_SENDPOINT_SIZE 384
#define VLS_128_NUMBER_OF_POINTS 230400
#define VLS_128_NUMBER_OF_PACKET 600
#define VLS_128_Distance_Res 4
#define VLS_128_Angle_Res 20 // 0.2 deg

#define OS1_128_NUMBER_OF_POINTS 131072 //HJK_250116

#pragma pack(push, 1)
typedef struct Data_point {  
    unsigned short 	Range;	// [mm] / n, VLP-16(n = 2), VLP-128(n = 4)
    unsigned char	Ref; 	// Reflectivity [-] 0~100 diffuse, 100~255 retroreflective
}tData_point;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Data_block {
	unsigned short Flag; 	// Constant
	unsigned short Azimuth; // [deg*100]
    tData_point Data_points[VLP_16_LIDAR_DATABLOCK_SIZE];
}tData_block;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct LidarRSI_SendData {
    tData_block Data_blocks[VLP_16_LIDAR_DATAPACKET_SIZE];
	unsigned int Timestamp; // [us]
	unsigned short Factory; // Constant
}tLidarRSI_SendData;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Data_block_32 {
	unsigned short Flag; 	// Constant
	unsigned short Azimuth; // [deg*100]
    tData_point Data_points[VLP_32_LIDAR_DATABLOCK_SIZE];
}tData_block_32;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct LidarRSI_SendData_32 {
    tData_block_32 Data_blocks[VLP_32_LIDAR_DATAPACKET_SIZE];
	unsigned int Timestamp; // [us]
	unsigned short Factory; // Constant
}tLidarRSI_SendData_32;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Data_block_64 {
	unsigned short Flag; 	// Constant
	unsigned short Azimuth; // [deg*100]
    tData_point Data_points[VLP_64_LIDAR_DATABLOCK_SIZE];
}tData_block_64;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct LidarRSI_SendData_64 {
    tData_block_64 Data_blocks[VLP_64_LIDAR_DATAPACKET_SIZE];
	unsigned int Timestamp; // [us]
	unsigned short Factory; // Constant
}tLidarRSI_SendData_64;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Data_block_128 {
	unsigned short Flag; 	// Constant
	unsigned short Azimuth; // [deg*100]
    tData_point Data_points[VLS_128_LIDAR_DATABLOCK_SIZE];
}tData_block_128;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct LidarRSI_SendData_128 {
    tData_block_128 Data_blocks[VLS_128_LIDAR_DATAPACKET_SIZE];
	unsigned int Timestamp; // [us]
	unsigned short Factory; // Constant
}tLidarRSI_SendData_128;
#pragma pack(pop)

typedef struct LidarRSI_ScanData {
    tData_point Data_points[LIDAR_MAX_SCANPOINT_SIZE];
}tLidarRSI_ScanData;

//HJK_250116 {
typedef struct OS1_data_block 
{
    unsigned int range;
    unsigned short reflectivity;
    unsigned short signal;
    unsigned short nir;
    unsigned short reserved;
}tOS1_data_block;

#pragma pack(push, 1)
typedef struct OS1_column
{
    unsigned long long timestamp;
    unsigned short measurement_id;
    unsigned short frame_id;
    unsigned int encoder_count;
    tOS1_data_block data_block[128];
    unsigned int block_status;
}tOS1_column;
#pragma pack(pop)

typedef struct OS1_packet 
{
	tOS1_column column[16];
}tOS1_packet;
//HJK_250116 }

typedef struct LidarRSI_Param {
    tInfos *Inf_Sensor;
    char SensorBeamFile[255];
    int	sensorCycleTime;
    int sensorCycleOffset;
	int nTotal;
	int nRays;
    int nRaysH;
    int nRaysV;
	double rayHorPerM;
	double rayVertPerM;
    double rayAreaPerM;
    double Beam_VertDiv; // opening angle of beam vertical
    double Beam_HorDiv;  // opening angle of beam horizontal
    double transmitPower;
    double receptionArea;
    double wavelength;
    double env_visRangeInFog;
    double env_alpha;
    double max_intensity;
	
	double roll;
	double pitch;
	double yaw;
	
	double **Lidar_Beams;
}tLidarRSI_Param;

typedef struct LidarRSI_SensorUnit {
	char SensorKind[20];
	char SensorValid;
    int socketOut_Lidar;
	int Num_Scan;
	unsigned short ID_Frame;

	//HJK_250116 {
	unsigned short os1_frame_count;
	unsigned short os1_measurement_id_count;
	unsigned int os1_encoder_count;
	unsigned int os1_point_count;
	//HJK_250116 }

    tLidarRSI_Param SensorParam;
	tLidarRSI_ScanData Lidar_ScanData;
	tLidarRSI_SendData Lidar_SendData[VLP_16_NUMBER_OF_PACKET];
	tLidarRSI_SendData_32 Lidar_SendData_32[VLP_32_NUMBER_OF_PACKET];
	tLidarRSI_SendData_64 Lidar_SendData_64[VLP_64_NUMBER_OF_PACKET];
	tLidarRSI_SendData_128 Lidar_SendData_128[VLS_128_NUMBER_OF_PACKET];
	tOS1_packet os1_packet[64]; //HJK_250116
	tLidarRSI *CM_LidarSens;
	tLidarRSI *ptr;
}tLidarRSI_SensorUnit;

typedef struct LidarRSI_Config {
    int nSensors;
	int Sens_Num[20];
	int Sens_Param[20];
	int Sens_Cluster[20];
    tLidarRSI_SensorUnit* LidarSensor;
} tLidarRSI_Config;

extern tLidarRSI_Config lidarConfig;

int LidarRSI_Config_TestRun_Start_atEnd(void);
int LidarRSI_Config_Out(const unsigned CycleNo);
int LidarRSI_Config_TestRun_End(void);
int LidarRSI_Config_Cleanup (void);

unsigned char LidarRSI_IntensityCalc_VLP(tLidarRSI_SensorUnit *sens, double intensity, double dist);
unsigned char LidarRSI_IntensityCalc_OS1(tLidarRSI_SensorUnit *sens, double intensity, double dist);

void RSDS_LidarRSI_ReadSpec_SensorInfoFile (char* SensorBeamFile, int idx_current, char SensorKind[20]);
int  RSDS_LidarRSI_ReadSpec_VehicleInfoFile(char* VehicleFileName, int MoviePort);
void RSDS_LidarRSI_Init_IntensityCalc (void);
int  RSDS_LidarRSI_Valid (void);
int  RSDS_LidarRSI_WriteSensorData (tLidarRSI_SensorUnit *sens, tRSIResMsg_Lidar *RSIResMsg_Lidar);
void RSDS_LidarRSI_Send_UDP_DataPackets (tLidarRSI_SensorUnit *sens);
int  RSDS_LidarRSI_Config_Out (tRSIResMsg_Lidar *RSIResMsg_Lidar);
int  RSDS_LidarRSI_Config_Cleanup (void);

#endif /* LIDARSIMMODEL_H_ */
