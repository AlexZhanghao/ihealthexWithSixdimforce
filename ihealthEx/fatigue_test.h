#pragma once
#include "control_card.h"
#include "data_acquisition.h"
#include "file_writer.h"
#include "FTWrapper.h"
#include"sensordata_processing.h"
#include"boundarydetection.h"

#define CCHART_UPDATE (2050)

class FatigueTest {
public:
	FatigueTest();
	~FatigueTest();

	void Initial(HWND hWnd);
	bool IsInitialed();

	void StartTest();
	void StartAbsoulteMove();
	void StartActiveMove();
	void AbsoluteMove();
	void ActMove();
	//������ά��
	void StartMove();
	//ATI
	void timerAcquisit();
	//ѹ���������ɼ�
	void PressureSensorAcquisit();
	//���ش������ɼ�
	void TorqueAcquisit();
	//���ش������˶�
	void TorqueMove();
	void PositionReset();
	void StopMove();
	bool IsErrorHappened();
	void AcquisiteData();
	void SetZero();
	//�������������ؽڻ��㵽���йؽ�
	void ActiveTorqueToAllTorque(double torque[2], double alltorque[5]);
	//�ȴ��߳�ֹͣ
	void ExitTorqueThread();

	//�������txt�ļ�
	void ExportForceData();
	//������ش��������ݵ�txt
	void ExportTorqueData();
	//���ѹ������������õ��������ݵ�txt
	void ExportMomentData();
	//�����ά�����ݵ�txt
	void ExportSixDimensionData();

public:
	HWND m_hWnd;
	double elbow_angle_error[500] { 0 };
	double shoulder_angle_error[500] { 0 };
	double elbow_angle_curve[500] { 0 };
	double shoulder_angle_curve[500] { 0 };
	double pull_force_curve1[500] { 0 };
	double pull_force_curve2[500] { 0 };
	double pull_force_curve3[500] { 0 };
	double pull_force_curve4[500] { 0 };
	double elbow_torque_curve[500] { 0 };
	double shoulder_torque_curve[500] { 0 };
	double x_axis[500] { 0 };

	bool m_stop = true;
	bool torque_collecting;
	bool torque_moving;

	double two_arm_offset[8];

public:
	
	double moments[6];
	double shoulder_torque; 
	double elbow_torque;
	double shoulder_tau;
	double elbow_tau;
	double shoulder_difference;
	double elbow_difference;
	double shoulder_fusion;
	double elbow_fusion;


private:
	void WriteDataToFile(int index);
	void UpdataDataArray();
	void UpdataDataArray2(double sensordata[8]);

	//��ԭʼֵ��������任
	void Raw2Trans(double RAWData[6], double DistData[6]);
	//��ת�����ֵ�����˲�-���װ�����˹��ͨ�˲���
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	//������ѹ�����������ݽ����˲�
	void Trans2FilterForPressure(double TransData[2], double FiltedData[2]);
	void FiltedVolt2Vel(double FiltedData[6]);
	void MomentCalculation(double ForceVector,double &vel);
	//����ά��ת�Ƶ��ؽڿռ�
	void SixDimensionForceRotation(double sixdimensionforce[6]);
	//�������������ݴ����������άʸ��������ʸ��ֻ�����������������ã��������4�����ݡ�����Ҫ��֪���������İ�װλ��
	void SensorDataToForceVector( double elbowsensordata[2], double ForceVector);

private:
	bool is_initialed = false;
	bool in_test_move = false;
	bool is_testing = false;
	bool is_moving = false;

	FTWrapper *mFTWrapper;
	DataAcquisition *m_pDataAcquisition = nullptr;
	ControlCard *m_pControlCard = nullptr;
	FileWriter *m_pFileWriter = nullptr;
	sensorprocess m_psensorprocess;
	boundaryDetection m_boundarydetect;
	HANDLE test_thread = nullptr;
	HANDLE ATI_thread = nullptr;
	HANDLE acquisition_thread = nullptr;

	double m_shoulder_vel;
	double m_elbow_vel;

	double m_shoulder_moment;
	double m_elbow_moment;
	static double six_dimforce[6];

	I32 option = 0x1000;//ptp�˶�ģʽ����
};