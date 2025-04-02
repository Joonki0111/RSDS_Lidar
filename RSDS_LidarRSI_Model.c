/*
 * RSDS_LidarRSI_Model.c
 *
 *  updated at: 07.20.2023
 *      Author: dhk
 *     Summary: set RSDS client for Virtual Velodyne Sensor Model
 *
 */

#include "RSDS_LidarRSI_Model.h"

tLidarRSI_Config lidarConfig = {0};

void RSDS_LidarRSI_ReadSpec_SensorInfoFile(char* SensorBeamFile, int idx_current, char SensorKind[20])
{
	// prefix
    tLidarRSI_SensorUnit *Pf_LidarSens = &lidarConfig.LidarSensor[idx_current];
	
	tErrorMsg 	*err_Sensor;
	int i = 0;
	int j = 0;
	int nRows = 0;
	int Table_Index_total = 0;
    double *Table_Data_Total = NULL;	//variable for all the data in the table
	
	// //Window Address
	// TCHAR programpath[_MAX_PATH]; 
	// GetModuleFileName( NULL, programpath, _MAX_PATH);
	
	//Linux Address
	char programpath[2000]; 
    char *res = realpath(".", programpath);
    
	// Read beam-file
    char LidarInfoFile[3000];
    // sprintf(LidarInfoFile, "%s/../../Data/Sensor/%s", programpath, Pf_LidarSens->SensorParam.SensorBeamFile);
	sprintf(LidarInfoFile, "%s/Data/Sensor/%s", programpath, Pf_LidarSens->SensorParam.SensorBeamFile); //test_dhk
	Pf_LidarSens->SensorParam.Inf_Sensor = InfoNew();
    iRead2(&err_Sensor, Pf_LidarSens->SensorParam.Inf_Sensor, LidarInfoFile, "");
    Log("BeamFile[%d]: %s\n", idx_current, Pf_LidarSens->SensorParam.SensorBeamFile);
	
	// Read Beams.N
	if      (strcmp(SensorKind, "VLP-16-360deg_50ms") == 0)		Pf_LidarSens->SensorParam.nTotal = VLP_16_POINTS_OF_SCAN * 5;
	else if (strcmp(SensorKind, "VLP-16-360deg_100ms") == 0)	Pf_LidarSens->SensorParam.nTotal = VLP_16_POINTS_OF_SCAN * 10;
	else if (strcmp(SensorKind, "VLP-32-360deg_50ms") == 0)		Pf_LidarSens->SensorParam.nTotal = VLP_32_POINTS_OF_SCAN * 5;
	else if (strcmp(SensorKind, "VLP-64") == 0)					Pf_LidarSens->SensorParam.nTotal = VLP_64_NUMBER_OF_POINTS;
	else if (strcmp(SensorKind, "VLS-128") == 0)				Pf_LidarSens->SensorParam.nTotal = VLS_128_NUMBER_OF_POINTS;
	else if (strcmp(SensorKind, "OS1-128") == 0)				Pf_LidarSens->SensorParam.nTotal = OS1_128_NUMBER_OF_POINTS; //HJK_250116
	else 														Pf_LidarSens->SensorParam.nTotal = 0;
	
	// Read h/v resolution of sensors
    double *Beam_Width = iGetFixedTable2(Pf_LidarSens->SensorParam.Inf_Sensor, "Beam.Width", 2 , 1);
    Pf_LidarSens->SensorParam.Beam_HorDiv = Beam_Width[0];
    Pf_LidarSens->SensorParam.Beam_VertDiv = Beam_Width[1];
    double *nRays_Beam = iGetFixedTable2(Pf_LidarSens->SensorParam.Inf_Sensor, "Beam.Rays", 2, 1);
    Pf_LidarSens->SensorParam.nRaysH = nRays_Beam[0];
    Pf_LidarSens->SensorParam.nRaysV = nRays_Beam[1];
    Pf_LidarSens->SensorParam.nRays = Pf_LidarSens->SensorParam.nRaysH * Pf_LidarSens->SensorParam.nRaysV;
	Pf_LidarSens->SensorParam.env_visRangeInFog = 10000.0; // m
		
	// Read-in beam table of Sensor-BeamFiles
    // There are 6 columns for Beams
    Table_Index_total = Pf_LidarSens->SensorParam.nTotal * 6;
    /* Allocate memory to load the whole table data for Beams, Table_Index_total * 1.  */
    Table_Data_Total = calloc(Table_Index_total, sizeof(double));
	
	// Allocate memory for array to save the data from (Table_Index_total*1 --> Beams_total *6).
	// Lidar_Beams will be initialized before every TestRun.
    Pf_LidarSens->SensorParam.Lidar_Beams = calloc (Pf_LidarSens->SensorParam.nTotal, sizeof(double));
    for (i = 0; i < Pf_LidarSens->SensorParam.nTotal; i++) {
		Pf_LidarSens->SensorParam.Lidar_Beams[i] = calloc (6, sizeof(double));
    }
	
	// Load data from InfoFile with the key "Beams".
    iGetTableOpt (Pf_LidarSens->SensorParam.Inf_Sensor, "Beams", Table_Data_Total, Table_Index_total, 6, &nRows);
	
    // Transform the array form 1D to 2D
    for (j = 0; j < 6; j++) {
		for (i = 0; i < Pf_LidarSens->SensorParam.nTotal; i++) {
			Pf_LidarSens->SensorParam.Lidar_Beams[i][j] = Table_Data_Total[j*Pf_LidarSens->SensorParam.nTotal + i];
		}
    }

	if(Pf_LidarSens->SensorParam.nTotal != nRows) LogErrStr(EC_Init, "Check the beam length about \'NR_OF_BEAMS\'\n");

	free(Table_Data_Total);
	
}

void RSDS_LidarRSI_Init_IntensityCalc(void)
{
	int i = 0;
	double q = 0.585;
	
	for (i=0; i < lidarConfig.nSensors; i++)
    {
		// Prefix
		tLidarRSI_SensorUnit *Pf_LidarSens = &lidarConfig.LidarSensor[i];
	
		Pf_LidarSens->SensorParam.rayHorPerM = tan(Pf_LidarSens->SensorParam.Beam_HorDiv * M_PI / 180.0) / Pf_LidarSens->SensorParam.nRaysH;
		Pf_LidarSens->SensorParam.rayVertPerM = tan(Pf_LidarSens->SensorParam.Beam_VertDiv * M_PI / 180.0) / Pf_LidarSens->SensorParam.nRaysV;
		Pf_LidarSens->SensorParam.rayAreaPerM = Pf_LidarSens->SensorParam.rayHorPerM * Pf_LidarSens->SensorParam.rayVertPerM;
		
		// Calculation of alpha for environment damping
		
		if (Pf_LidarSens->SensorParam.env_visRangeInFog < 6000.0)
		{
			q = 0.585 * pow(Pf_LidarSens->SensorParam.env_visRangeInFog, 1.0/3.0);
		}
		else if (Pf_LidarSens->SensorParam.env_visRangeInFog < 50000.0)
		{
			q = 1.3;
		}
		else
		{
			q = 1.6;
		}
		
		Pf_LidarSens->SensorParam.env_alpha = 3.912 / Pf_LidarSens->SensorParam.env_visRangeInFog * pow((Pf_LidarSens->SensorParam.wavelength / 550.0), -1.0 * q); // dB/m
		
		Pf_LidarSens->SensorParam.max_intensity = Pf_LidarSens->SensorParam.transmitPower * 1000000000.0 / Pf_LidarSens->SensorParam.nRays;       // nW
	}
}

int RSDS_LidarRSI_ReadSpec_VehicleInfoFile(char* VehicleFileName, int MoviePort)
{
	char *name;
	char key[64];
    int i; 
	int port;
    int ret = 0; // Return value;
	int Num_Sens, Ref_Param, Active_Sens;
	int tmp = 0;
	
	// Set Vehicle-InfoFile
	tLidarRSI_SensorUnit *Pf_LidarSens = &lidarConfig.LidarSensor[0];
	tErrorMsg 	*err_Vehicle;
	
	// //Window Address
	// TCHAR programpath[_MAX_PATH]; 
	// GetModuleFileName( NULL, programpath, _MAX_PATH);
	
	//Linux Address
	char programpath[2000]; 
    char *res = realpath(".", programpath);
    
	// Read beam-file
	char VehicleInfoFile[3000];
	// sprintf(VehicleInfoFile, "%s/../../Data/Vehicle/%s", programpath, VehicleFileName);
	sprintf(VehicleInfoFile, "%s/Data/Vehicle/%s", programpath, VehicleFileName); //test_dhk
    tInfos *VehicleInfo = InfoNew();
	iRead2(&err_Vehicle, VehicleInfo, VehicleInfoFile, "");
    Log("Vehicle File: %s\n", VehicleFileName);
	
	// Amount of Lidar RSI Sensors
	sprintf(key, "Sensor.N");
	Num_Sens = iGetInt(VehicleInfo, key);
	lidarConfig.nSensors = 0;
	
	for (i=0; i<Num_Sens ; i++)
	{
		sprintf(key, "Sensor.%d.Active", i);
		Active_Sens = iGetInt(VehicleInfo, key);
		if(Active_Sens == 0) continue;
		
		sprintf(key, "Sensor.%d.Ref.Param", i);
		Ref_Param = iGetInt(VehicleInfo, key);
		
		sprintf(key, "Sensor.Param.%d.Type", Ref_Param);
		name = iGetStrOpt(VehicleInfo, key, NULL);
		
		if(strcmp(name, "LidarRSI") == 0)	
		{
			sprintf(key, "Sensor.%d.Ref.Cluster", i);
			int Sens_ClusterNum = iGetInt(VehicleInfo, key);
			
			sprintf(key, "SensorCluster.%d.ClientType", Sens_ClusterNum);
			char *Sens_ClientType = iGetStr(VehicleInfo, key);
			int Sens_Socket = 0;
			if(strcmp(Sens_ClientType, "UserDefined") == 0){
				sprintf(key, "SensorCluster.%d.Socket", Sens_ClusterNum);
				Sens_Socket = iGetInt(VehicleInfo, key);
			}
			
			if(Sens_Socket == MoviePort){
				lidarConfig.nSensors++;
				lidarConfig.Sens_Num[tmp] = i;
				lidarConfig.Sens_Param[tmp] = Ref_Param;
				sprintf(key, "Sensor.%d.Ref.Cluster", i);
				lidarConfig.Sens_Cluster[tmp] = iGetInt(VehicleInfo, key);
				tmp++;
			}
		}
	}
    
    if(lidarConfig.nSensors < 1){
		Log ("No LidarRSI detected!\n");
    } else {
		Log("\nLidar module:\n");
		Log("Amount of LidarRSI sensors: %d\n", lidarConfig.nSensors);
    }
	
	// Initialization of sensor configuration
    lidarConfig.LidarSensor = malloc(lidarConfig.nSensors * sizeof(tLidarRSI_SensorUnit));
	
	for (i=0; i < lidarConfig.nSensors; i++)
    {
		// Prefix
		tLidarRSI_SensorUnit *Pf_LidarSens = &lidarConfig.LidarSensor[i];
		
		// Read sensor name
		sprintf(key, "Sensor.%d.Ref.Param", lidarConfig.Sens_Num[i]);
		int param_num = iGetIntOpt(VehicleInfo, key, NULL);
		sprintf(key, "Sensor.Param.%d.Name", param_num);
    	sprintf(Pf_LidarSens->SensorKind, iGetStrOpt(VehicleInfo, key, NULL));
    	Log("Sensor Kind[%d]: %s\n", i, Pf_LidarSens->SensorKind);
		
		// Read IP & Port
		sprintf(key, "Sensor.LidarRSI.IP");
    	name = iGetStrOpt(VehicleInfo, key, "localhost");
    	Log("IP: %s\n",name);
    	port = MoviePort * 10 + i; //22100(if Default) = 2210(Default Movie Port) * 10 + i(Sensor Number of connected GPUSensor)
    	Log("Port: %d\n", port);
    	if ((Pf_LidarSens->socketOut_Lidar = UDP_New_OutSocket(name, port)) < 0) {
    	    LogErrF(EC_Init, "Can't open output port '%s:%d' for sensor '%d'", name, port, i);
    	    ret = -1;
    	    break;
    	}
		
		// Read Rotation of sensors
		sprintf(key, "Sensor.%d.rot", lidarConfig.Sens_Num[i]);
		double *Rot = iGetFixedTable2(VehicleInfo, key, 3 , 1);
		Pf_LidarSens->SensorParam.roll = Rot[0] / 180 * M_PI;
		Pf_LidarSens->SensorParam.pitch = Rot[1] / 180 * M_PI;
		Pf_LidarSens->SensorParam.yaw = Rot[2] / 180 * M_PI;
		
		// Read CycleTimes of sensors from Vehicle-InfoFile
		sprintf(key, "SensorCluster.%d.CycleTime", lidarConfig.Sens_Cluster[i]);
		Pf_LidarSens->SensorParam.sensorCycleTime = (int) iGetDbl(VehicleInfo, key); // ms
		
		// Read CycleOffsets of sensors from Vehicle-InfoFile
		sprintf(key, "SensorCluster.%d.CycleOffset", lidarConfig.Sens_Cluster[i]);
		Pf_LidarSens->SensorParam.sensorCycleOffset = (int) iGetDbl(VehicleInfo, key); // ms
		
		// Read BeamFile-Names of sensors from Vehicle-InfoFile
		// sprintf(key, "Sensor.Param.%d.BeamsFName", lidarConfig.Sens_Param[i]); //Under CM-10
		sprintf(key, "Sensor.Param.%d.Beams.FName", lidarConfig.Sens_Param[i]); //Over CM-11
		sprintf(Pf_LidarSens->SensorParam.SensorBeamFile, iGetStr(VehicleInfo, key));
		
		// Read Transmit Power of sensor
		sprintf(key, "Sensor.Param.%d.TransmitPower", lidarConfig.Sens_Param[i]);
		Pf_LidarSens->SensorParam.transmitPower = iGetDbl(VehicleInfo, key); // W
		
		// Read Reception Area of sensor
		sprintf(key, "Sensor.Param.%d.ReceptionArea", lidarConfig.Sens_Param[i]);
		Pf_LidarSens->SensorParam.receptionArea = iGetDbl(VehicleInfo, key)/100/100; // m^2
		
		// Read Wavelength of sensor
		sprintf(key, "Sensor.Param.%d.Wavelength", lidarConfig.Sens_Param[i]);
		Pf_LidarSens->SensorParam.wavelength = iGetDbl(VehicleInfo, key); // nm
		
		RSDS_LidarRSI_ReadSpec_SensorInfoFile(Pf_LidarSens->SensorParam.SensorBeamFile, i, Pf_LidarSens->SensorKind);
		
		// Initialization Param
		Pf_LidarSens->Num_Scan = 0;

	}
	
	for (i=0; i<lidarConfig.nSensors; i++){
		InfoDelete(lidarConfig.LidarSensor[i].SensorParam.Inf_Sensor);
    }
	return ret;
	
}

int RSDS_LidarRSI_Valid(void)
{
	int i;
	
	for (i=0; i < lidarConfig.nSensors; i++)
    {
		// Prefix
		tLidarRSI_SensorUnit *Pf_LidarSens = &lidarConfig.LidarSensor[i];
		
		if(strcmp(Pf_LidarSens->SensorKind, "VLP-16-360deg_50ms") == 0)
		{

			if(Pf_LidarSens->SensorParam.nTotal == VLP_16_POINTS_OF_SCAN * 5)
			{
				Pf_LidarSens->SensorValid = 1;
			}
			else 
			{
				Pf_LidarSens->SensorValid = 0;
				Log("Check the VLP-16-360deg_50ms Sensor Beam file\n");
			}
		}
		else if(strcmp(Pf_LidarSens->SensorKind, "VLP-16-360deg_100ms") == 0)
		{
			if(Pf_LidarSens->SensorParam.nTotal == VLP_16_POINTS_OF_SCAN * 10)
			{
				Pf_LidarSens->SensorValid = 1;
			}
			else 
			{
				Pf_LidarSens->SensorValid = 0;
				Log("Check the VLP-16-360deg_100ms Sensor Beam file\n");
			}
		}
		else if(strcmp(Pf_LidarSens->SensorKind, "VLP-32-360deg_50ms") == 0)
		{

			if(Pf_LidarSens->SensorParam.nTotal == VLP_32_POINTS_OF_SCAN * 5)
			{
				Pf_LidarSens->SensorValid = 1;
			}
			else 
			{
				Pf_LidarSens->SensorValid = 0;
				Log("Check the VLP-32-360deg_50ms Sensor Beam file\n");
			}
		}
		else if(strcmp(Pf_LidarSens->SensorKind, "VLP-64") == 0)
		{
			if(Pf_LidarSens->SensorParam.nTotal == VLP_64_NUMBER_OF_POINTS)
			{
				Pf_LidarSens->SensorValid = 1;
			}
			else 
			{
				Pf_LidarSens->SensorValid = 0;
				Log("Check the VLP-64 Sensor Beam file\n");
			}
		}
		else if(strcmp(Pf_LidarSens->SensorKind, "VLS-128") == 0)
		{

			if(Pf_LidarSens->SensorParam.nTotal == VLS_128_NUMBER_OF_POINTS)
			{
				Pf_LidarSens->SensorValid = 1;
			}
			else 
			{
				Pf_LidarSens->SensorValid = 0;
				Log("Check the VLS-128 Sensor Beam file\n");
			}
		}
		//HJK_250116 {
		else if(strcmp(Pf_LidarSens->SensorKind, "OS1-128") == 0)
		{

			if(Pf_LidarSens->SensorParam.nTotal == OS1_128_NUMBER_OF_POINTS)
			{
				Pf_LidarSens->SensorValid = 1;
			}
			else 
			{
				Pf_LidarSens->SensorValid = 0;
				Log("Check the OS1-128 Sensor Beam file\n");
			}
		}
		//HJK_250116 }
		else {
			Pf_LidarSens->SensorValid = 0;
			Log("Check the Sensor %d kind\n",i);
		}
		
	}
	
	return 0;
}

unsigned char LidarRSI_IntensityCalc_VLP(tLidarRSI_SensorUnit *sens, double intensity, double dist)
{
	unsigned char reflectivity = 0;
	double FogDmp;			// Fog damping
	double Area_ray;		// Area of ray
	double Area_receive; 	// Area of receiver
	double reflectivity_temp;
	double HorWidthBeam;
	double VertWidthBeam;
	double lengthRecvMax;

	FogDmp = exp(-sens->SensorParam.env_alpha * dist);
	
	// Diffuse
	Area_ray = 2.0*M_PI*dist*dist/4.0; // 2*pi*(d/2)^2
	reflectivity_temp = intensity / sens->SensorParam.max_intensity / FogDmp / sens->SensorParam.receptionArea * Area_ray;
	
	if( reflectivity_temp <= 1.0)
	{
		reflectivity = (unsigned char) (reflectivity_temp * 100.0); // Range 0~100
		return reflectivity;
	}
	
	// Retroreflective
	HorWidthBeam = sens->SensorParam.rayHorPerM * dist;
    VertWidthBeam = sens->SensorParam.rayVertPerM * dist;

    lengthRecvMax = sqrt(sens->SensorParam.receptionArea);
	
	if(HorWidthBeam > lengthRecvMax) HorWidthBeam = lengthRecvMax;
	if(VertWidthBeam > lengthRecvMax) VertWidthBeam = lengthRecvMax;
	
	Area_receive = HorWidthBeam * VertWidthBeam;
	Area_ray = sens->SensorParam.rayAreaPerM*dist*dist;
	
	reflectivity_temp = intensity / sens->SensorParam.max_intensity / FogDmp / Area_receive * Area_ray;
	
	if( reflectivity_temp >= 1.0)
	{
		reflectivity_temp = 1.0;
	}
	
	reflectivity = (unsigned char) (reflectivity_temp * 155.0 + 100.0); // Range 100~255

	return reflectivity;
}

unsigned char LidarRSI_IntensityCalc_OS1(tLidarRSI_SensorUnit *sens, double intensity, double dist)
{
	unsigned char reflectivity = 0;
	double FogDmp;			// Fog damping
	double Area_ray;		// Area of ray
	double Area_receive; 	// Area of receiver
	double reflectivity_temp;
	double HorWidthBeam;
	double VertWidthBeam;
	double lengthRecvMax;

	FogDmp = exp(-sens->SensorParam.env_alpha * dist);
	
	// Diffuse
	Area_ray = 2.0*M_PI*dist*dist/4.0; // 2*pi*(d/2)^2
	reflectivity_temp = intensity / sens->SensorParam.max_intensity / FogDmp / sens->SensorParam.receptionArea * Area_ray;
	
	if( reflectivity_temp <= 1.0)
	{
		reflectivity = (unsigned char) (reflectivity_temp * 127.0); // Range 0~100
		return reflectivity;
	}
	
	// Retroreflective
	HorWidthBeam = sens->SensorParam.rayHorPerM * dist;
    VertWidthBeam = sens->SensorParam.rayVertPerM * dist;

    lengthRecvMax = sqrt(sens->SensorParam.receptionArea);
	
	if(HorWidthBeam > lengthRecvMax) HorWidthBeam = lengthRecvMax;
	if(VertWidthBeam > lengthRecvMax) VertWidthBeam = lengthRecvMax;
	
	Area_receive = HorWidthBeam * VertWidthBeam;
	Area_ray = sens->SensorParam.rayAreaPerM*dist*dist;
	
	reflectivity_temp = intensity / sens->SensorParam.max_intensity / FogDmp / Area_receive * Area_ray;
	
	if( reflectivity_temp >= 1.0)
	{
		reflectivity_temp = 1.0;
	}
	
	reflectivity = (unsigned char) (reflectivity_temp * 128.0 + 127.0); // Range 100~255

	return reflectivity;
}

int RSDS_LidarRSI_WriteSensorData(tLidarRSI_SensorUnit *sens, tRSIResMsg_Lidar *RSIResMsg_Lidar)
{
	int i = 0;
	int count = 0;
	double temp;
	unsigned char temp_c;
	unsigned int temp_i;
	
	if(strcmp(sens->SensorKind, "VLP-16-360deg_50ms") == 0)
	{	
		for(i = 0; i < VLP_16_POINTS_OF_SCAN * 5; i++){
			sens->Lidar_ScanData.Data_points[i].Range = 0;
			sens->Lidar_ScanData.Data_points[i].Ref = 0;
		}
		for(i = 0; i < RSIResMsg_Lidar->Header.nScanPoints; i++){
			temp =  RSIResMsg_Lidar->SP[i].LengthOF * 1000 / 2 / VLP_16_Distance_Res; // [mm] - VLP_16_Distance_Res : 2, VLP_32_Distance_Res: 4, VLP_64_Distance_Res: 4
			if(temp > 60000) temp = 60000;
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Range = (unsigned short) temp;

			temp_c = LidarRSI_IntensityCalc_VLP(sens,  RSIResMsg_Lidar->SP[i].Intensity,  RSIResMsg_Lidar->SP[i].LengthOF);
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Ref = temp_c;
			
		}

	}

	else if(strcmp(sens->SensorKind, "VLP-16-360deg_100ms") == 0)
	{	
		for(i = 0; i < VLP_16_POINTS_OF_SCAN * 10; i++){
			sens->Lidar_ScanData.Data_points[i].Range = 0;
			sens->Lidar_ScanData.Data_points[i].Ref = 0;
		}
		for(i = 0; i < RSIResMsg_Lidar->Header.nScanPoints; i++){
			temp =  RSIResMsg_Lidar->SP[i].LengthOF * 1000 / 2 / VLP_16_Distance_Res; // [mm] - VLP_16_Distance_Res : 2, VLP_32_Distance_Res: 4, VLP_64_Distance_Res: 4
			if(temp > 60000) temp = 60000;
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Range = (unsigned short) temp;

			temp_c = LidarRSI_IntensityCalc_VLP(sens,  RSIResMsg_Lidar->SP[i].Intensity,  RSIResMsg_Lidar->SP[i].LengthOF);
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Ref = temp_c;
			
		}

	}

	else if(strcmp(sens->SensorKind, "VLP-32-360deg_50ms") == 0)
	{	
		for(i = 0; i < VLP_32_POINTS_OF_SCAN * 10; i++){
			sens->Lidar_ScanData.Data_points[i].Range = 0;
			sens->Lidar_ScanData.Data_points[i].Ref = 0;
		}
		for(i = 0; i < RSIResMsg_Lidar->Header.nScanPoints; i++){
			temp =  RSIResMsg_Lidar->SP[i].LengthOF * 1000 / 2 / VLP_32_Distance_Res; // [mm] - VLP_16_Distance_Res : 2, VLP_32_Distance_Res: 4, VLP_64_Distance_Res: 4
			if(temp > 60000) temp = 60000;
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Range = (unsigned short) temp;

			temp_c = LidarRSI_IntensityCalc_VLP(sens,  RSIResMsg_Lidar->SP[i].Intensity,  RSIResMsg_Lidar->SP[i].LengthOF);
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Ref = temp_c;
			
		}

	}

	
	else if(strcmp(sens->SensorKind, "VLP-64") == 0)
	{	
		for(i = 0; i < VLP_64_NUMBER_OF_POINTS*2; i++){
			sens->Lidar_ScanData.Data_points[i].Range = 0;
			sens->Lidar_ScanData.Data_points[i].Ref = 0;
		}
		for(i = 0; i < RSIResMsg_Lidar->Header.nScanPoints; i++){
			temp =  RSIResMsg_Lidar->SP[i].LengthOF * 1000 / 2 / VLP_64_Distance_Res; // [mm] - VLP_64_Distance_Res: 4
			if(temp > 60000) temp = 60000;
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID*2].Range = (unsigned short) temp;

			temp_c = LidarRSI_IntensityCalc_VLP(sens,  RSIResMsg_Lidar->SP[i].Intensity,  RSIResMsg_Lidar->SP[i].LengthOF);
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID*2].Ref = temp_c;
			
		}

	}

	else if(strcmp(sens->SensorKind, "VLS-128") == 0)
	{	
		for(i = 0; i < VLS_128_NUMBER_OF_POINTS; i++){
			sens->Lidar_ScanData.Data_points[i].Range = 0;
			sens->Lidar_ScanData.Data_points[i].Ref = 0;
		}
		for(i = 0; i < RSIResMsg_Lidar->Header.nScanPoints; i++){
			temp =  RSIResMsg_Lidar->SP[i].LengthOF * 1000 / 2 / VLS_128_Distance_Res; // [mm] - VLS_128_Distance_Res: 4
			if(temp > 60000) temp = 60000;
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Range = (unsigned short) temp;

			temp_c = LidarRSI_IntensityCalc_VLP(sens,  RSIResMsg_Lidar->SP[i].Intensity,  RSIResMsg_Lidar->SP[i].LengthOF);
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Ref = temp_c;
			
		}

	}

	//HJK_250116 {
	else if(strcmp(sens->SensorKind, "OS1-128") == 0)
	{	
		for(i = 0; i < OS1_128_NUMBER_OF_POINTS; i++){
			sens->Lidar_ScanData.Data_points[i].Range = 0;
			sens->Lidar_ScanData.Data_points[i].Ref = 0;
		}
		for(i = 0; i < RSIResMsg_Lidar->Header.nScanPoints; i++){
			temp =  RSIResMsg_Lidar->SP[i].LengthOF * 500; //HJK
			if(temp > 524287.0) temp = 524287.0; //HJK
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Range = (unsigned int) temp;

			temp_c = LidarRSI_IntensityCalc_VLP(sens,  RSIResMsg_Lidar->SP[i].Intensity,  RSIResMsg_Lidar->SP[i].LengthOF);
			sens->Lidar_ScanData.Data_points[RSIResMsg_Lidar->SP[i].BeamID].Ref = temp_c;
			
		}
	
	}
	//HJK_250116 }
	return 0;
}

void RSDS_LidarRSI_Send_UDP_DataPackets(tLidarRSI_SensorUnit *sens)
{
	int i,j,k;
	double temp;

	// Set DataPackets
	if(strcmp(sens->SensorKind, "VLP-16-360deg_50ms") == 0)
	{	
		for(int i=0; i < VLP_16_NUMBER_OF_PACKET; i++) //VLP_16_NUMBER_OF_PACKET 75
		{
			for(int j=0; j < VLP_16_LIDAR_DATAPACKET_SIZE; j++)
			{
				sens->Lidar_SendData[i].Data_blocks[j].Flag = 0xEEFF;
				int azitmp = 40*2*(i*VLP_16_LIDAR_DATAPACKET_SIZE + j);
				if(azitmp > 65535) azitmp = 65535;
				sens->Lidar_SendData[i].Data_blocks[j].Azimuth = azitmp;
				
				for(int k=0; k < VLP_16_LIDAR_DATABLOCK_SIZE; k++)
				{
					memcpy(&sens->Lidar_SendData[i].Data_blocks[j].Data_points[k], &sens->Lidar_ScanData.Data_points[i*VLP_16_LIDAR_SENDPOINT_SIZE + VLP_16_LIDAR_DATABLOCK_SIZE*j + k], sizeof(tData_point));
				}	
			}
		}
		
		struct timeval time_now;
		gettimeofday(&time_now, NULL);
		long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);
		while( UDP_time > 3600000000 )
		{
			UDP_time -= 3600000000;
		}
		for(int i=0; i < VLP_16_NUMBER_OF_PACKET; i++){ //VLP_16_NUMBER_OF_PACKET 75
			sens->Lidar_SendData[i].Timestamp = UDP_time;
			sens->Lidar_SendData[i].Factory = 0x3722;
		}
		
		for(int i=0; i < VLP_16_NUMBER_OF_PACKET; i++){ //VLP_16_NUMBER_OF_PACKET 75
			UDP_SendMsg(sens->socketOut_Lidar, &sens->Lidar_SendData[i], sizeof(tLidarRSI_SendData));
		}
		
	}
	
	else if(strcmp(sens->SensorKind, "VLP-16-360deg_100ms") == 0)
	{	
		for(int i=0; i < VLP_16_NUMBER_OF_PACKET; i++) //VLP_16_NUMBER_OF_PACKET 75
		{
			for(int j=0; j < VLP_16_LIDAR_DATAPACKET_SIZE; j++)
			{
				sens->Lidar_SendData[i].Data_blocks[j].Flag = 0xEEFF;
				int azitmp = 40*1*(i*VLP_16_LIDAR_DATAPACKET_SIZE + j);
				if(azitmp > 65535) azitmp = 65535;
				sens->Lidar_SendData[i].Data_blocks[j].Azimuth = azitmp;
				
				for(int k=0; k < VLP_16_LIDAR_DATABLOCK_SIZE; k++)
				{
					memcpy(&sens->Lidar_SendData[i].Data_blocks[j].Data_points[k], &sens->Lidar_ScanData.Data_points[i*VLP_16_LIDAR_SENDPOINT_SIZE + VLP_16_LIDAR_DATABLOCK_SIZE*j + k], sizeof(tData_point));
				}	
			}
		}
		
		struct timeval time_now;
		gettimeofday(&time_now, NULL);
		long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);
		while( UDP_time > 3600000000 )
		{
			UDP_time -= 3600000000;
		}
		for(int i=0; i < VLP_16_NUMBER_OF_PACKET; i++){ //VLP_16_NUMBER_OF_PACKET 75
			sens->Lidar_SendData[i].Timestamp = UDP_time;
			sens->Lidar_SendData[i].Factory = 0x3722;
		}
		
		for(int i=0; i < VLP_16_NUMBER_OF_PACKET; i++){ //VLP_16_NUMBER_OF_PACKET 75
			UDP_SendMsg(sens->socketOut_Lidar, &sens->Lidar_SendData[i], sizeof(tLidarRSI_SendData));
		}
		
	}

	else if(strcmp(sens->SensorKind, "VLP-32-360deg_50ms") == 0)
	{	
		for(int i=0; i < VLP_32_NUMBER_OF_PACKET; i++)
		{
			for(int j=0; j < VLP_32_LIDAR_DATAPACKET_SIZE; j++)
			{
				sens->Lidar_SendData_32[i].Data_blocks[j].Flag = 0xEEFF;
				int azitmp = VLP_32_Angle_Res*2*(i*VLP_32_LIDAR_DATAPACKET_SIZE + j);
				if(azitmp > 65535) azitmp = 65535;
				sens->Lidar_SendData_32[i].Data_blocks[j].Azimuth = azitmp;
				
				for(int k=0; k < VLP_32_LIDAR_DATABLOCK_SIZE; k++)
				{
					memcpy(&sens->Lidar_SendData_32[i].Data_blocks[j].Data_points[k], &sens->Lidar_ScanData.Data_points[i*VLP_32_LIDAR_SENDPOINT_SIZE + VLP_32_LIDAR_DATABLOCK_SIZE*j + k], sizeof(tData_point));
				}	
			}
		}
		
		struct timeval time_now;
		gettimeofday(&time_now, NULL);
		long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);
		while( UDP_time > 3600000000 )
		{
			UDP_time -= 3600000000;
		}
		for(int i=0; i < VLP_32_NUMBER_OF_PACKET; i++){
			sens->Lidar_SendData_32[i].Timestamp = UDP_time;
			sens->Lidar_SendData_32[i].Factory = 0x3728;
		}
		
		for(int i=0; i < VLP_32_NUMBER_OF_PACKET; i++){
			UDP_SendMsg(sens->socketOut_Lidar, &sens->Lidar_SendData_32[i], sizeof(tLidarRSI_SendData_32));
		}
		
	}

	else if(strcmp(sens->SensorKind, "VLP-64") == 0)
	{	
		for(i = 0; i < VLP_64_NUMBER_OF_PACKET; i++)
		{
			for(j = 0; j < VLP_64_LIDAR_DATAPACKET_SIZE; j++)
			{
				if(j ==  0) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xEEFF;
				if(j ==  1) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xDDFF;
				if(j ==  2) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xCCFF;
				if(j ==  3) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xBBFF;
				if(j ==  4) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xEEFF;
				if(j ==  5) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xDDFF;
				if(j ==  6) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xCCFF;
				if(j ==  7) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xBBFF;
				if(j ==  8) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xEEFF;
				if(j ==  9) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xDDFF;
				if(j == 10) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xCCFF;
				if(j == 11) sens->Lidar_SendData_64[i].Data_blocks[j].Flag = 0xBBFF;
				int azitmp = VLP_64_Angle_Res*(i*3 + floor(j/3));
				azitmp = azitmp % 36000;
				sens->Lidar_SendData_64[i].Data_blocks[j].Azimuth = azitmp;
				
				for(int k=0; k < VLP_64_LIDAR_DATABLOCK_SIZE; k++)
				{
					memcpy(&sens->Lidar_SendData_64[i].Data_blocks[j].Data_points[k], &sens->Lidar_ScanData.Data_points[i*384 + 32*j + k], sizeof(tData_point));
				}	
			}
		}

		struct timeval time_now;
		gettimeofday(&time_now, NULL);
		long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);
		while( UDP_time > 3600000000 )
		{
			UDP_time -= 3600000000;
		}
		for(i = 0; i < VLP_64_NUMBER_OF_PACKET; i++){
			sens->Lidar_SendData_64[i].Timestamp = (unsigned int) UDP_time;
			sens->Lidar_SendData_64[i].Factory = 0x37A1;
		}

		for(i = 0; i < VLP_64_NUMBER_OF_PACKET; i++){
			
			UDP_SendMsg(sens->socketOut_Lidar, &sens->Lidar_SendData_64[i], sizeof(tLidarRSI_SendData_64));
		}
		
	}

	else if(strcmp(sens->SensorKind, "VLS-128") == 0)
	{	
		for(i = 0; i < VLS_128_NUMBER_OF_PACKET; i++)
		{
			for(j = 0; j < VLS_128_LIDAR_DATAPACKET_SIZE; j++)
			{
				if(j ==  0) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xEEFF;
				if(j ==  1) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xDDFF;
				if(j ==  2) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xCCFF;
				if(j ==  3) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xBBFF;
				if(j ==  4) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xEEFF;
				if(j ==  5) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xDDFF;
				if(j ==  6) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xCCFF;
				if(j ==  7) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xBBFF;
				if(j ==  8) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xEEFF;
				if(j ==  9) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xDDFF;
				if(j == 10) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xCCFF;
				if(j == 11) sens->Lidar_SendData_128[i].Data_blocks[j].Flag = 0xBBFF;
				int azitmp = VLS_128_Angle_Res*(i*3 + floor(j/3));
				azitmp = azitmp % 36000;
				sens->Lidar_SendData_128[i].Data_blocks[j].Azimuth = azitmp;
				
				for(int k=0; k < VLS_128_LIDAR_DATABLOCK_SIZE; k++)
				{
					memcpy(&sens->Lidar_SendData_128[i].Data_blocks[j].Data_points[k], &sens->Lidar_ScanData.Data_points[i*384 + 32*j + k], sizeof(tData_point));
				}	
			}
		}

		struct timeval time_now;
		gettimeofday(&time_now, NULL);
		long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);
		while( UDP_time > 3600000000 )
		{
			UDP_time -= 3600000000;
		}
		for(i = 0; i < VLS_128_NUMBER_OF_PACKET; i++){
			sens->Lidar_SendData_128[i].Timestamp = (unsigned int) UDP_time;
			sens->Lidar_SendData_128[i].Factory = 0x37A1;
		}

		for(i = 0; i < VLS_128_NUMBER_OF_PACKET; i++){
			UDP_SendMsg(sens->socketOut_Lidar, &sens->Lidar_SendData_128[i], sizeof(tLidarRSI_SendData_128));
		}
		
	}
	//HJK_250116 {
	else if(strcmp(sens->SensorKind, "OS1-128") == 0)
	{
		for(int i = 0; i < 64; i++) //send packet 64 times
		{
			for(int j = 0; j < 16; j++) //this is one packet
			{
				struct timeval time_now;
				gettimeofday(&time_now, NULL);
				long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);

				sens->os1_packet[i].column[j].timestamp = UDP_time;
				sens->os1_packet[i].column[j].measurement_id = sens->os1_measurement_id_count;
				sens->os1_packet[i].column[j].frame_id = sens->os1_frame_count; //TODO
				sens->os1_packet[i].column[j].encoder_count = sens->os1_encoder_count;
				for(int k = 0; k < 128; k++) //this is one column
				{
					sens->os1_packet[i].column[j].data_block[k].range = sens->Lidar_ScanData.Data_points[sens->os1_point_count].Range;
					sens->os1_packet[i].column[j].data_block[k].reflectivity = sens->Lidar_ScanData.Data_points[sens->os1_point_count].Ref;
					sens->os1_packet[i].column[j].data_block[k].signal = 0;
					sens->os1_packet[i].column[j].data_block[k].nir = 0;
					sens->os1_packet[i].column[j].data_block[k].reserved = 0;
					sens->os1_point_count++;
					if(sens->os1_point_count == OS1_128_NUMBER_OF_POINTS)
					{
						sens->os1_point_count = 0;
					}
				}
				sens->os1_packet[i].column[j].block_status =0xffffffff;

				sens->os1_measurement_id_count++;
				sens->os1_encoder_count += 88; //1024 = 88
				if(sens->os1_measurement_id_count == 1024)
				{
					sens->os1_measurement_id_count = 0;
				}
				if(sens->os1_encoder_count >= 90111)
				{
					sens->os1_encoder_count = 0;
				}
			}
			UDP_SendMsg(sens->socketOut_Lidar, &sens->os1_packet[i], sizeof(tOS1_packet));
		}

		if(sens->os1_frame_count == USHRT_MAX)
		{
			sens->os1_frame_count = 0;
		};
		sens->os1_frame_count++;
	}
	//HJK_250116 }

	return;
}

int RSDS_LidarRSI_Config_Out(tRSIResMsg_Lidar *RSIResMsg_Lidar)
{
	int i,j;
	
	struct timeval time_now;
	gettimeofday(&time_now, NULL);
	long long UDP_time = (time_now.tv_sec * 1000LL + time_now.tv_usec / 1000);
	// Log("UDP_time:%lld\n", UDP_time);
	
	double Rot_yaw;
	double Rot_rad[3];
	
	for (i=0; i < lidarConfig.nSensors; i++)
	{
		// Sensor valid check
		if(lidarConfig.LidarSensor[i].SensorValid == 0)	continue;
		
		// Sensor data update
		RSDS_LidarRSI_WriteSensorData(&lidarConfig.LidarSensor[i], RSIResMsg_Lidar);
		
		if(strcmp(lidarConfig.LidarSensor[i].SensorKind, "VLP-16-360deg_50ms") == 0)
		{	
			// Sensor data send
			RSDS_LidarRSI_Send_UDP_DataPackets(&lidarConfig.LidarSensor[i]);
		}
		else if(strcmp(lidarConfig.LidarSensor[i].SensorKind, "VLP-16-360deg_100ms") == 0)
		{	
			// Sensor data send
			RSDS_LidarRSI_Send_UDP_DataPackets(&lidarConfig.LidarSensor[i]);
		}
		else if(strcmp(lidarConfig.LidarSensor[i].SensorKind, "VLP-32-360deg_50ms") == 0)
		{	
			// Sensor data send
			RSDS_LidarRSI_Send_UDP_DataPackets(&lidarConfig.LidarSensor[i]);
		}
		else if(strcmp(lidarConfig.LidarSensor[i].SensorKind, "VLP-64") == 0)
		{	
			// Sensor data send
			RSDS_LidarRSI_Send_UDP_DataPackets(&lidarConfig.LidarSensor[i]);
		}
		else if(strcmp(lidarConfig.LidarSensor[i].SensorKind, "VLS-128") == 0)
		{	
			// Sensor data send
			RSDS_LidarRSI_Send_UDP_DataPackets(&lidarConfig.LidarSensor[i]);
		}
		//HJK_250116 {
		else if(strcmp(lidarConfig.LidarSensor[i].SensorKind, "OS1-128") == 0)
		{	
			// Sensor data send
			RSDS_LidarRSI_Send_UDP_DataPackets(&lidarConfig.LidarSensor[i]);
		}
		//HJK_250116 }
	}
	
	return 0;
}

int RSDS_LidarRSI_Config_Cleanup(void)
{
	for(int i = 0; i < lidarConfig.nSensors; i++){
		for(int j = 0; j < lidarConfig.LidarSensor[i].SensorParam.nTotal; j++){
			free(lidarConfig.LidarSensor[i].SensorParam.Lidar_Beams[j]);
		}
		free(lidarConfig.LidarSensor[i].SensorParam.Lidar_Beams);
		
		free(lidarConfig.LidarSensor[i].ptr);
	}

	free(lidarConfig.LidarSensor);
	//UDP_Delete();
	
	return 0;
}

int LidarRSI_Config_Cleanup (void)
{
	//UDP_Cleanup();
	
	return 0;
}


