#pragma once
#include "NIDAQmx.h"

class DataAcquisition {
public:
	 DataAcquisition();
	 ~DataAcquisition();
	void AcquisiteTorqueData(double torquedata[2]);
	void AcquisiteTorqueOffset();
	void AcquisitePullSensorData();
	void AcquisiteSixDemensionData(double output_buf[6]);
	//这里尝试下把肩肘的数据采集放在一起，感觉这样性能可以提升
	void AcquisiteTensionData(double tension_output[2]);

	bool StartTask();
	bool StopTask();

	double ShoulderForwardPull();
	double ShoulderBackwardPull();
	double ElbowForwardPull();
	double ElbowBackwardPull();

public:
	double torque_data[20] { 0 };
	double pull_sensor_data[2] {0};


private:
	TaskHandle m_task_handle;
	TaskHandle s_task_handle;
	const char *torque_channel = "dev2/ai4:5";
	const char *pull_sensor_channel = "Dev2/ai0:1";
	const char *six_dimension_force_channel = "Dev1/ai0:5";
	const char *kPressureForceChannel = "Dev3/ai5:6";

	static const double kRawToReal;
};