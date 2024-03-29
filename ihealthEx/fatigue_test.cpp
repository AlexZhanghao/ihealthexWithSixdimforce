#include "fatigue_test.h"
#include<vector>
#include <process.h>
#include <string>
#include <iostream>
#include<fstream>
#include "Matrix.h"

#define BOYDET_TIME 0.1

double Force_Fc = 0.3;
double Force_a = 0.3;
double Force_b = 1;
double shoulder_offset ;
double elbow_offset ;
double moment1[8] { 0.0 };
double torque_vel[8];
double pressure_data1;

double Ud_Arm = 0;//力控模式算出手臂的命令速度
double Ud_Shoul = 0;//力控模式算出肩部的命令速度

const double shoulder_moment_variance = 4.3769 / 10000;
const double elbow_moment_variance = 1.2576 / 10000;
const double shoulder_torque_variance = 1.4634 / 100000;
const double elbow_torque_variance = 2.2896 / 100000;
const double shoulder_pos = 40;
const double elbow_pos = 40;
const double sixdim = 0.015;

	
vector<double> force_collection[4];
vector<double> torque_collection[2];
vector<double> moment_collection[2];
vector<double> fatigue_collection[6];
vector<double> sixdimension_collection;

double FatigueTest::six_dimforce[6] { 0 };

unsigned int __stdcall TestThread(PVOID pParam);
unsigned int __stdcall SixdimThread(PVOID pParam);
unsigned int __stdcall AcquisitionThread(PVOID pParam);
unsigned int __stdcall FatigueTestThread(PVOID pParam);

FatigueTest::FatigueTest() {
	mFTWrapper = new FTWrapper();

	shoulder_torque = 0;
	elbow_torque = 0;
	shoulder_fusion = 0;
	elbow_fusion = 0;
	is_moving = false;
	torque_collecting = false;
	torque_moving = false;
}

FatigueTest::~FatigueTest() {
	if (mFTWrapper != NULL) {
		delete mFTWrapper;
	}

	//if (m_pDataAcquisition != NULL) {
	//	delete m_pDataAcquisition;
	//}
}

void FatigueTest::Initial(HWND hWnd) {
	m_hWnd = hWnd;
	m_pControlCard = new ControlCard();
	m_pControlCard->Initial();
	m_boundarydetect.startBydetect();
	m_pDataAcquisition = new DataAcquisition();
	m_pFileWriter = new FileWriter();

	for (int i = 0; i < 500; ++i) {
		x_axis[i] = i;
	}
	is_initialed = true;
}

bool FatigueTest::IsInitialed() {
	return is_initialed;
}

void FatigueTest::StartTest() {

	//ATI六维力传感器使用
	//mFTWrapper->LoadCalFile();
	//mFTWrapper->BiasCurrentLoad(true);
	//mFTWrapper->setFUnit();
	//mFTWrapper->setTUnit();

	is_testing = true;
	torque_collecting = true;

	//主动运动线程
	test_thread = (HANDLE)_beginthreadex(NULL, 0, TestThread, this, 0, NULL);
	//六维力线程启动，这里要小心线程冲突
	Sixdim_thread = (HANDLE)_beginthreadex(NULL, 0, SixdimThread, this, 0, NULL);
	//传感器开启线程
	//acquisition_thread = (HANDLE)_beginthreadex(NULL, 0, AcquisitionThread, this, 0, NULL);
}

void FatigueTest::StartAbsoulteMove() {
	m_pControlCard->SetMotor(MOTOR_ON);
	m_pControlCard->SetClutch(CLUTCH_ON);

	is_moving = true;

	AbsoluteMove();
}

void FatigueTest::StartActiveMove() {
	//ATI六维力传感器使用
	mFTWrapper->LoadCalFile();
	mFTWrapper->BiasCurrentLoad(true);
	mFTWrapper->setFUnit();
	mFTWrapper->setTUnit();

	m_pControlCard->SetMotor(MOTOR_ON);
	m_pControlCard->SetClutch(CLUTCH_ON);

	is_testing = true;
	is_moving = true;
	torque_moving = true;

	//主动运动线程
	test_thread = (HANDLE)_beginthreadex(NULL, 0, TestThread, this, 0, NULL);
	//六维力线程启动，这里要小心线程冲突
	Sixdim_thread = (HANDLE)_beginthreadex(NULL, 0, SixdimThread, this, 0, NULL);
	////传感器开启线程
	//acquisition_thread = (HANDLE)_beginthreadex(NULL, 0, AcquisitionThread, this, 0, NULL);
}

void FatigueTest::AbsoluteMove() {
	if (!IsInitialed()) {
		return;
	}
	I32 Axis[2] = { SHOULDER_AXIS_ID,ELBOW_AXIS_ID };

	APS_ptp_v(Axis[0], option, shoulder_pos / VEL_TO_PALSE, 3 /  VEL_TO_PALSE, NULL);
	APS_ptp_v(Axis[1], option, elbow_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE, NULL);
	//APS_absolute_move(Axis[0], shoulder_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE);
	//APS_absolute_move(Axis[1], elbow_pos / VEL_TO_PALSE, 3 / VEL_TO_PALSE);
}

void FatigueTest::ActMove() {
	m_pControlCard->VelocityMove(SHOULDER_AXIS_ID, Ud_Shoul);
	m_pControlCard->VelocityMove(ELBOW_AXIS_ID, Ud_Arm);
}

void FatigueTest::ATIAcquisit() {
	if (!IsInitialed()) { 
		return;
	}

	double torquedata[2]{ 0 };

	double readings[7] = { 0 };
	double distData[6] = { 0 };
	double filtedData[6] = { 0 };

	while (is_testing == true) {
		//readings的值单位就是力的单位N
		mFTWrapper->GetForcesAndTorques(readings);

		for (int i = 0; i < 6; ++i) {
			distData[i] = readings[i];
		}

		Trans2Filter(distData, filtedData);

		for (int i = 0; i < 6; ++i) {
			six_dimforce[i] = filtedData[i];
		}

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//std::cout << "f0: " << readings[0] << " f1: " << readings[1] <<
		//	" f2: " << readings[2] << " f3: " << readings[3] <<
		//	" f4: " << readings[4] << " f5: " << readings[5] << std::endl;

		//Raw2Trans(readings, distData);
		//Trans2Filter(distData, filtedData);
		//FiltedVolt2Vel(filtedData);
		
		//SixDimensionForceRotation(readings);

		//UpdataDataArray2(readings);
		//PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
}

void FatigueTest::SixDimensionForceRotation(double sixdimensionforce[6]) {
	VectorXd sixdim(6);
	Vector3d sixdim_to_coordinate3 = Vector3d(d3 - shouler_installationsite_to_coordinate4, sixdim_shoulder, 0);

	for (int i = 0; i < 6; ++i) {
		sixdim(i) = sixdimensionforce[i];			
	}
	
	MatrixXd sixdim_transfer(6,6);
	Matrix3d sixdim_rotation;
	Matrix3d to_zero;
	Matrix3d Sixdim_To_Coordinate3;

	VectorToMatrix(sixdim_to_coordinate3, Sixdim_To_Coordinate3);
	to_zero.setZero();

	sixdim_rotation <<
		1, 0, 0,
		0, -1, 0,
		0, 0, -1;

	sixdim_transfer <<
		sixdim_rotation, to_zero,
		Sixdim_To_Coordinate3*sixdim_rotation, sixdim_rotation;

	sixdim = sixdim_transfer*sixdim;
	for (int i = 0; i < 6; ++i) {
		sixdimensionforce[i] = sixdim(i);
	}
}

void FatigueTest::SixdimForceAcquisit() {
	if (!IsInitialed()) {
		return;
	}
	in_test_move = true;
	double readings[6];
	double distData[6] = { 0 };
	double filtedData[6] = { 0 };
	double sub_bias[6] = { 0 };

	// 求六维力传感器的偏置
	double sum[6]{ 0.0 };
	double buf[6]{ 0.0 };
	double six_dimension_offset_[6]{ 0 };
	for (int i = 0; i < 10; ++i) {
		m_pDataAcquisition->AcquisiteSixDemensionData(buf);
		for (int j = 0; j < 6; ++j) {
			sum[j] += buf[j];
		}
	}
	for (int i = 0; i < 6; ++i) {
		six_dimension_offset_[i] = sum[i] / 10;
	}

	while (true) {
		if (!in_test_move) {
			break;
		}

		m_pDataAcquisition->AcquisiteSixDemensionData(readings);

		// 求减去偏置之后的六维力，这里对z轴的力和力矩做了一个反向
		for (int i = 0; i < 6; ++i) {
			sub_bias[i] = readings[i] - six_dimension_offset_[i];
		}
		sub_bias[2] = -sub_bias[2];
		sub_bias[5] = -sub_bias[5];

		for (int i = 0; i < 6; ++i) {
			six_dimforce[i] = sub_bias[i];
		}

		//for (int i = 0; i < 6; ++i) {
		//	readings[i] = -readings[i];
		//}

		//Raw2Trans(readings, distData);
		//Trans2Filter(distData, filtedData);
		//FiltedVolt2Vel(filtedData);



		//注意这里我们传入的顺序是fxfyfzMxMyMz,现在传的是裸值，以后如果你要传入
		//经过转换后的值的话，传入的顺序也要是fxfyfzMxMyMz(转换后的值力和力矩的位置是交换了的)
		//UpdataDataArray(moments);
		//PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
}

void FatigueTest::StopMove() {
	in_test_move = false;
	is_testing = false;
	is_moving = false;
	is_fatigue = false;
	torque_collecting = false;
	torque_moving = false;
	//这里不放开离合的原因是为了防止中间位置松开离合导致手臂迅速下坠
	m_pControlCard->SetMotor(MOTOR_OFF);
	ExitTorqueThread();
}

void FatigueTest::PositionReset() {
	if (!IsInitialed()) {
		return;
	}
	m_pControlCard->PositionReset();
}

void FatigueTest::FatigueTestAcquisit() {
	double torquedata[2]{ 0 };
	double angle[2]{ 0 };

	m_pDataAcquisition->AcquisiteTorqueData(torquedata);
	m_pDataAcquisition->AcquisitePullSensorData();
	m_pControlCard->GetEncoderData(angle);

	fatigue_collection[0].push_back(angle[0]);
	fatigue_collection[1].push_back(angle[1]);
	fatigue_collection[2].push_back(m_pDataAcquisition->pull_sensor_data[0]);
	fatigue_collection[3].push_back(m_pDataAcquisition->pull_sensor_data[1]);
	fatigue_collection[4].push_back(m_pDataAcquisition->pull_sensor_data[2]);
	fatigue_collection[5].push_back(m_pDataAcquisition->pull_sensor_data[3]);
}

//序号 肘部编码器 肩部编码器 拉力1 拉力2 拉力3 拉力4 肘部力矩 肩部力矩 "\n";
void FatigueTest::WriteDataToFile(int index) {
	m_pFileWriter->WriteString(std::to_string(index));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->elbow_position_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->shoulder_position_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->elbow_error_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pControlCard->shoulder_error_in_degree));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[0]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[1]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[2]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->pull_sensor_data[3]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->torque_data[0]));
	m_pFileWriter->WriteString(std::to_string(m_pDataAcquisition->torque_data[1]));
	m_pFileWriter->WriteString("\n");
}

void FatigueTest::UpdataDataArray() {
	for (int i = 0; i < 499; i++) {
		elbow_angle_error[i] = 0;
		shoulder_angle_error[i] = 0;
		elbow_angle_curve[i] = 0;
		shoulder_angle_curve[i] = 0;
		pull_force_curve1[i] = 0;
		pull_force_curve2[i] = 0;
		pull_force_curve3[i] = 0;
		pull_force_curve4[i] = 0;
	}
}

void FatigueTest::UpdataDataArray2(double sensordata[8]) {
	for (int i = 0; i < 499; i++) {
		elbow_angle_error[i] = elbow_angle_error[i + 1];
		shoulder_angle_error[i] = shoulder_angle_error[i + 1];
		elbow_angle_curve[i] = elbow_angle_curve[i + 1];
		shoulder_angle_curve[i] = shoulder_angle_curve[i + 1];
		pull_force_curve1[i] = pull_force_curve1[i + 1];
		pull_force_curve2[i] = pull_force_curve2[i + 1];
		pull_force_curve3[i] = pull_force_curve3[i + 1];
		pull_force_curve4[i] = pull_force_curve4[i + 1];
	}
	elbow_angle_error[499] = sensordata[0];
	shoulder_angle_error[499] = sensordata[1];
	elbow_angle_curve[499] = sensordata[2];
	shoulder_angle_curve[499] = sensordata[3];
	pull_force_curve1[499] = sensordata[4];
	pull_force_curve2[499] = sensordata[5];
	pull_force_curve3[499] = sensordata[6];
	pull_force_curve4[499] = sensordata[7];
}

bool FatigueTest::IsErrorHappened() {
	if (m_pControlCard->elbow_position_in_degree < -20.0) {
		return true;
	}
	return false;
}

unsigned int __stdcall TestThread(PVOID pParam) {
	FatigueTest *mTest = static_cast<FatigueTest *>(pParam);
	if (!mTest->IsInitialed()) {
		return 1;
	}

	//国产六维力
	//mTest->StartMove();
	//压力传感器
	mTest->PressureSensorAcquisit();
}

unsigned int __stdcall SixdimThread(PVOID pParam) {
	FatigueTest *sTest = static_cast<FatigueTest *>(pParam);
	if (!sTest->IsInitialed()) {
		return 1;
	}

	////ATI六维力
	//sTest->ATIAcquisit();
	//国产六维力
	sTest->SixdimForceAcquisit();
}

unsigned int __stdcall AcquisitionThread(PVOID pParam) {
	FatigueTest *tTest = static_cast<FatigueTest *>(pParam);
	if (!tTest->IsInitialed()) {
		return 1;
	}

	//力矩传感器采集
	if (tTest->torque_collecting == true) {
		tTest->TorqueAcquisit();
	}

	//力矩传感器运动
	//if (tTest->torque_moving == true) {
	//	tTest->TorqueMove();
	//}

}

unsigned int __stdcall FatigueTestThread(PVOID pParam) {
	FatigueTest *fTest = static_cast<FatigueTest *>(pParam);
	if (!fTest->IsInitialed()) {
		return 1;
	}

	while (fTest->is_fatigue) {
		fTest->FatigueTestAcquisit();
	}

	fTest->ExportFatigueData();
}


void FatigueTest::Raw2Trans(double RAWData[6], double DistData[6]) {
	//这一段就是为了把力从六维力传感器上传到手柄上，这里的A就是总的一个转换矩阵。
	//具体的旋转矩阵我们要根据六维力的安装确定坐标系方向之后然后再确定。
	MatrixXd A(6, 6);
	A.setZero();
	VectorXd Value_Origi(6);
	VectorXd Value_Convers(6);
	Matrix3d rotate_matrix;
	//这里的旋转矩阵要根据六维力坐标系和手柄坐标系来具体得到
	rotate_matrix <<
		cos(M_PI_4), sin(M_PI_4), 0,
		-sin(M_PI_4), cos(M_PI_4), 0,
		0, 0, 1;
	//Vector3d ForcePosition(-0.075,0.035,0);
	//手柄坐标系下手柄坐标系原点到六维力坐标系原点的向量
	Vector3d ForcePosition(0.075, -0.035, 0);
	Matrix3d ForcePositionHat;
	//这里就是这个p，我们可以想象，fx不会产生x方向的力矩，fy产生的看z坐标，fz产生的y坐标。
	//这里做的就是把力矩弄过去。这个相对坐标都是六维力坐标在手柄坐标系下的位置。
	//比如fx在y方向上有一个力臂，就会产生一个z方向上的力矩。这个力矩的方向和相对位置无关。
	//所以这个地方我们不用改这个ForcePositionHat，只用改ForcePosition这个相对位置就可以了
	ForcePositionHat <<
		0, -ForcePosition[2], ForcePosition[1],
		ForcePosition[2], 0, -ForcePosition[0],
		-ForcePosition[1], ForcePosition[0], 0;
	A.block(0, 0, 3, 3) = rotate_matrix;
	A.block(0, 3, 3, 3) = ForcePositionHat * rotate_matrix;
	A.block(3, 3, 3, 3) = rotate_matrix;


	//之前是fxfyfzMxMyMz,现在变成MxMyMzfxfyfz
	for (int i = 0; i < 6; i++) {
		if (i<3) {
			Value_Origi(i) = RAWData[i + 3];
		} else {
			Value_Origi(i) = RAWData[i - 3];
		}
	}

	//这里计算后就是换算到手柄坐标系上之后的六维力的值，MxMyMzfxfyfz
	Value_Convers = A * Value_Origi;
	for (int m = 0; m<6; m++) {
		DistData[m] = Value_Convers(m);
	}
}

void FatigueTest::Trans2Filter(double TransData[6], double FiltedData[6]) {
	double Wc = 5;
	double Ts = 0.1;
	static int i = 0;
	static double Last_Buffer[6] = { 0 };
	static double Last2_Buffer[6] = { 0 };
	static double Force_Buffer[6] = { 0 };
	static double Last_FT[6] = { 0 };
	static double Last2_FT[6] = { 0 };
	for (int m = 0; m < 6; m++) {
		if (i == 0) {
			Last2_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		} else if (i == 1) {
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		} else {
			//二阶巴特沃斯低通滤波器
			Force_Buffer[m] = TransData[m];
			FiltedData[m] = (1 / (Wc*Wc + 2 * 1.414*Wc / Ts + 4 / (Ts*Ts)))*((Wc*Wc)*Force_Buffer[m]
																			 + (2 * Wc*Wc)*Last_Buffer[m]
																			 + (Wc*Wc)*Last2_Buffer[m]
																			 - (2 * Wc*Wc - 8 / (Ts*Ts))*Last_FT[m]
																			 - (Wc*Wc - 2 * 1.414*Wc / Ts + 4 / (Ts*Ts))*Last2_FT[m]);

			Last2_FT[m] = Last_FT[m];
			Last_FT[m] = FiltedData[m];
			Last2_Buffer[m] = Last_Buffer[m];
			Last_Buffer[m] = Force_Buffer[m];
		}
	}
}

void FatigueTest::FiltedVolt2Vel(double FiltedData[6]) {
	MatrixXd Vel(2, 1);
	MatrixXd Pos(2, 1);
	MatrixXd A(6, 6);
	VectorXd Six_Sensor_Convert(6);
	VectorXd moment(5);
	double angle[2];
	m_pControlCard->GetEncoderData(angle);
	Pos(0, 0) = angle[0];
	Pos(1, 0) = angle[1];

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("elbow angle: %lf\n", angle[1]);
	//printf("shoulder angle: %lf\n", angle[0]);
	//printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", FiltedData[3], FiltedData[4], FiltedData[5], FiltedData[0], FiltedData[1], FiltedData[2]);

	for (int i = 0; i < 6; i++) {
		Six_Sensor_Convert(i) = FiltedData[i];
	}
	damping_control(Six_Sensor_Convert, Pos, Vel, Force_Fc, Force_a, Force_b);

	//计算tau值并输出
	TauExport(Pos, Six_Sensor_Convert, moment);
	shoulder_tau = moment(0);
	elbow_tau = moment(2);

	//shoulder_difference = shoulder_tau - shoulder_moment;
	//elbow_difference = elbow_tau - elbow_moment;

	//moments[0] = shoulder_moment;
	//moments[1] = elbow_moment;
	//moments[2] = shoulder_tau;
	//moments[3] = elbow_tau;
	//moments[4] = shoulder_difference;
	//moments[5] = elbow_difference;
	//
	//UpdataDataArray(moments);
    //PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

	//m_shoulder_vel = Vel(0, 0);
	//m_elbow_vel = Vel(1, 0);

	//printf("肩部速度: %lf\n", Ud_Shoul);
	//printf("肘部速度: %lf\n", Ud_Arm);
}

void FatigueTest::TorqueAcquisit() {
	if (!IsInitialed()) {
		return;
	}

	torque_collecting = false;

	elbow_offset = 0;
	shoulder_offset = 0;
	double shouleder_torque_sum_data = 0;
	double elbow_torque_sum_data = 0;
	double torquedata[2]{ 0 };

	//这里采到到的值会出现都是0的情况，所以加个检查
	while(elbow_torque_sum_data==0){
		m_pDataAcquisition->AcquisiteTorqueOffset();
			for (int j = 0; j < 5; ++j) {
				elbow_torque_sum_data += m_pDataAcquisition->torque_data[j + 5];
				shouleder_torque_sum_data += m_pDataAcquisition->torque_data[15 + j];
			}
	}

	elbow_offset = elbow_torque_sum_data / 5;
	shoulder_offset = shouleder_torque_sum_data / 5;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("elbow_offset:%lf  shoulder_offset:%lf\n", elbow_offset, shoulder_offset);

	m_pDataAcquisition->StopTask();
	m_pDataAcquisition->StartTask();

	while (is_testing == true) {
		m_pDataAcquisition->AcquisiteTorqueData(torquedata);
		shoulder_torque = 2 * (torquedata[1] - shoulder_offset);
		elbow_torque = -2 * (torquedata[0] - elbow_offset);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("elbow_offset:%lf    shoulder_offset:%lf  \n  shoulder_torque:%lf    elbow_torque:%lf  \n", elbow_offset, shoulder_offset, 2 * m_pDataAcquisition->torque_data[1], 2 * m_pDataAcquisition->torque_data[0]);

	}
}

void FatigueTest::TorqueMove() {

	torque_moving = false;

	double torque_subdata[2]{ 0 };
	double torque_alldata[5]{ 0 };

	Vector2d vel;
	VectorXd torque(5);

	while (is_moving == true) {
		torque_subdata[0] = shoulder_torque;
		torque_subdata[1] = elbow_torque;

		ActiveTorqueToAllTorque(torque_subdata, torque_alldata);

		for (int i = 0; i < 5; ++i) {
			torque(i) = torque_alldata[i];
		}

		AdmittanceControl(torque, vel);

		//Ud_Shoul = vel(0);
		//Ud_Arm = vel(1);
		Ud_Shoul = 1.5*shoulder_torque;
		Ud_Arm = 2*elbow_torque;

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("Ud_Shoul:%lf  Ud_Arm:%lf\n", Ud_Shoul, Ud_Arm);

		for (int i = 0; i < 5; ++i) {
			torque_vel[i] = torque_alldata[i];
		}
		torque_vel[6] = Ud_Shoul;
		torque_vel[7] = Ud_Arm;

		if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5)) {
			Ud_Arm = 0;
		}
		if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5)) {
			Ud_Shoul = 0;
		}
		if (Ud_Arm > 3) {
			Ud_Arm = 3;
		}
		else if (Ud_Arm < -3) {
			Ud_Arm = -3;
		}
		if (Ud_Shoul > 3) {
			Ud_Shoul = 3;
		}
		else if (Ud_Shoul < -3) {
			Ud_Shoul = -3;
		}

		ActMove();
	}
}

void FatigueTest::ExitTorqueThread() {
	if (acquisition_thread != 0) {
		::WaitForSingleObject(acquisition_thread, INFINITE);
		acquisition_thread = 0;
	}
}

void FatigueTest::ActiveTorqueToAllTorque(double torque[2], double alltorque[5]) {
	alltorque[0] = torque[0];
	alltorque[1] = torque[0] * 3 / 2;
	alltorque[2] = torque[1];
	alltorque[3] = torque[1] * 56 / 50;
	alltorque[4] = torque[1] * 74 / 50;
}

void FatigueTest::PressureSensorAcquisit() {
	if (!IsInitialed()) {
		return;
	}

	double pressure_data[2] = { 0 };
	double elbow_suboffset[2] = { 0 };
	double elbow_smooth[2] = { 0 };
	double force_vector = 0;
	double force_smoothed[2] = { 0 };
	double vel = 0;
	double force_change[2]{ 0 };
	
	//将肩肘部合在一起
	double two_arm_sum[2]{ 0.0 };
	double two_arm_buf[2]{ 0.0 };
	double two_arm_suboffset[2]{ 0.0 };

	//换成出的和传感器测得的力矩,这里只有6个数据，但是测压力传感器数据时有八个数据，
	//所以这里多加两个和那个匹配，只是最后两个是恒为0的
	double output_moment[8]{ 0.0 };

	for (int i = 0; i < 10; ++i) {
		m_pDataAcquisition->AcquisiteTensionData(two_arm_buf);
		for (int j = 0; j < 2; ++j) {
			two_arm_sum[j] += two_arm_buf[j];
		}
	}

	for (int i = 0; i < 2; ++i) {
		two_arm_offset[i] = two_arm_sum[i] / 10;
	}

	m_pDataAcquisition->StopTask();
	m_pDataAcquisition->StartTask();


	while (is_testing==true){
		m_pDataAcquisition->AcquisiteTensionData(pressure_data);


		for (int i = 0; i < 2; ++i) {
			two_arm_suboffset[i] = pressure_data[i] - two_arm_offset[i];
		}

		//把单位从电压转换成力
		for (int j = 0; j < 2; ++j) {
			elbow_suboffset[j] = 20 * two_arm_suboffset[j];
		}

		//将传感器获取的数据滤波
		Trans2FilterForPressure(elbow_suboffset, elbow_smooth);

		//将传感器数据转成力矢量
		//SensorDataToForceVector(elbow_smooth, force_vector);
		force_vector = elbow_smooth[0] - elbow_smooth[1];

		//将压力转化成关节力矩
		MomentCalculation(force_vector,vel);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("force_vector1:%lf    force_vector2:%lf    force_vector3:%lf	   force_vector4:%lf   \n", force_vector[0], force_vector[1], force_vector[2], force_vector[3]);

		if (Ud_Shoul > 0) {
			Ud_Shoul = 3 * vel;
		}
		else {
			Ud_Shoul = 2* vel;
		}
		if (Ud_Arm > 0) {
			Ud_Arm = 3 * force_vector;
		}
		else {
			Ud_Arm = 2 * force_vector;
		}

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//printf("Ud_Shoul:%lf  Ud_Arm:%lf\n", Ud_Shoul, Ud_Arm);

		//if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5)) {
		//	Ud_Arm = 0;
		//}
		//if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5)) {
		//	Ud_Shoul = 0;
		//}
		if (Ud_Arm > 3) {
			Ud_Arm = 3;
		}
		else if (Ud_Arm < -3) {
			Ud_Arm = -3;
		}
		if (Ud_Shoul > 3) {
			Ud_Shoul = 3;
		}
		else if (Ud_Shoul < -3) {
			Ud_Shoul = -3;
		}

		ActMove();

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//cout <<"m_shoulder_moment:\n"<< m_shoulder_moment << "\n" << "m_elbow_moment:\n"<< m_elbow_moment << endl;
		//cout << "shoulder_suboffset1:" << shoulder_suboffset[0] << "   " << "shoulder_suboffset2:" << shoulder_suboffset[1] << "   " << "shoulder_suboffset3:" << shoulder_suboffset[2] << "   " << "shoulder_suboffset4:" << shoulder_suboffset[3] << endl;
		//cout << "elbow_suboffset1:" << elbow_suboffset[0] << "   " << "elbow_suboffset2:" << elbow_suboffset[1] << "   " << "elbow_suboffset3:" << elbow_suboffset[2] << "   " << "elbow_suboffset4:" << elbow_suboffset[3] << endl;

		//shoulder_difference = m_shoulder_moment - shoulder_moment;
		//elbow_difference = m_elbow_moment - elbow_moment;

		//shoulder_fusion = m_psensorprocess.DataFusion(m_shoulder_moment, shoulder_torque, shoulder_moment_variance, shoulder_torque_variance);
		//elbow_fusion = m_psensorprocess.DataFusion(m_elbow_moment, elbow_torque, elbow_moment_variance, elbow_torque_variance);

		output_moment[0] = m_shoulder_moment;
		output_moment[1] = m_elbow_moment;
		output_moment[2] = 0;
		output_moment[3] = 0;
		output_moment[4] = vel;
		output_moment[5] = force_vector;
		output_moment[6] = Ud_Shoul;
		output_moment[7] = Ud_Arm;



		////采集力矩传感器数据
		//torque_collection[0].push_back(shoulder_torque);
		//torque_collection[1].push_back(elbow_torque);
		////采集六维力数据
		//sixdimension_collection.push_back(sixdim_elbow_moment);
		////采集压力传感器计算出的数据
		//moment_collection[0].push_back(m_shoulder_moment);
		//moment_collection[1].push_back(m_elbow_moment);

		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);
		//cout << "m_shoulder_moment:\n" << m_shoulder_moment << "\n" << "m_elbow_moment:\n" << m_elbow_moment << endl;
		
		UpdataDataArray2(output_moment);
		PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);

		Sleep(100);
	}
	//ExportMomentData();
	//ExportTorqueData();
	//ExportSixDimensionData();
	//ExportForceData();
}


void FatigueTest::Trans2FilterForPressure(double TransData[2], double FiltedData[2]) {
	double Wc = 3;
	double Ts = 0.1;
	static int i = 0;
	static double Last_Buffer[2] = { 0 };
	static double Last2_Buffer[2] = { 0 };
	static double Force_Buffer[2] = { 0 };
	static double Last_FT[2] = { 0 };
	static double Last2_FT[2] = { 0 };
	for (int m = 0; m < 2; m++)
	{
		if (i == 0)
		{
			Last2_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else if (i == 1)
		{
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else
		{
			//二阶巴特沃斯低通滤波器
			Force_Buffer[m] = TransData[m];
			FiltedData[m] = (1 / (Wc*Wc + 2 * 1.414*Wc / Ts + 4 / (Ts*Ts)))*((Wc*Wc)*Force_Buffer[m]
				+ (2 * Wc*Wc)*Last_Buffer[m]
				+ (Wc*Wc)*Last2_Buffer[m]
				- (2 * Wc*Wc - 8 / (Ts*Ts))*Last_FT[m]
				- (Wc*Wc - 2 * 1.414*Wc / Ts + 4 / (Ts*Ts))*Last2_FT[m]);

			Last2_FT[m] = Last_FT[m];
			Last_FT[m] = FiltedData[m];
			Last2_Buffer[m] = Last_Buffer[m];
			Last_Buffer[m] = Force_Buffer[m];
		}
	}
}

void FatigueTest::MomentCalculation(double ForceVector,double& vel) {
	MatrixXd m_vel(1,1);
	MatrixXd pos(2, 1);

	VectorXd shoulder_force_moment_vector(6);
	VectorXd elbow_force_moment_vector(6);
	VectorXd six_dimensional_force_simulation(6);
	VectorXd v_moment(5);

	double angle[2];
	double moment[5];

	m_pControlCard->GetEncoderData(angle);

	pos(0, 0) = angle[0];
	pos(1, 0) = angle[1];

	shoulder_force_moment_vector(0) = six_dimforce[0];
	shoulder_force_moment_vector(1) = six_dimforce[1];
	shoulder_force_moment_vector(2) = six_dimforce[2];
	shoulder_force_moment_vector(3) = six_dimforce[3];
	shoulder_force_moment_vector(4) = six_dimforce[4];
	shoulder_force_moment_vector(5) = six_dimforce[5];
	elbow_force_moment_vector(0) = 0;
	elbow_force_moment_vector(1) = 0;
	elbow_force_moment_vector(2) = 0;
	elbow_force_moment_vector(3) = 0;
	elbow_force_moment_vector(4) = 0;
	elbow_force_moment_vector(5) = 0;

	MomentBalance(shoulder_force_moment_vector, elbow_force_moment_vector, angle, moment);

	for (int i = 0; i < 5; ++i) {
		moment1[i] = moment[i];
	}

	//moment1[6] = moment[0] / moment[2];
	//moment1[7] = moment[1] / moment[2];

	//if (moment1[6] > 5 || moment1[6] < -5) {
	//	moment1[6] = 0;
	//}
	//if (moment1[7] > 5 || moment1[7] < -5) {
	//	moment1[7] = 0;
	//}

	for (int i = 0; i < 5; ++i) {
		v_moment(i) = moment[i];
	}

	Vector2d shoulder_moment;
	shoulder_moment(0) = moment[0];
	shoulder_moment(1) = moment[1];

	m_shoulder_moment = moment[0];
	m_elbow_moment = moment[2];

	AdmittanceControl(shoulder_moment, m_vel);

	vel = moment[0];

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("moment1:%lf      moment2:%lf      moment3:%lf      moment4:%lf      moment5:%lf  \n", moment[0], moment[1], moment[2], moment[3], moment[4]);

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("moment1:%lf      moment2:%lf      moment3:%lf      moment4:%lf      moment5:%lf  \n", shoulder_force_moment_vector(0), shoulder_force_moment_vector(1), shoulder_force_moment_vector(2), shoulder_force_moment_vector(3), shoulder_force_moment_vector(4));

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout << "m_shoulder_moment:\n" << m_shoulder_moment << "\n" << "m_elbow_moment:\n" << m_elbow_moment << endl;
	//cout << "shoulderX:" << ForceVector[0] << "		" << "shoudlerY:" << ForceVector[1] << endl;
	//cout << "elbowX:" << ForceVector[2] << "		" << "elbowY:" << ForceVector[3] << endl;
}

void FatigueTest::SensorDataToForceVector( double elbowsensordata[2], double ForceVector) {
	double elbowdata = elbowsensordata[0] - elbowsensordata[1];

	//合成的力矢量
	//Vector2d shoulderforce;
	//Vector2d elbowforce;
	//shoulderforce << shoulderdataY, shoulderdataZ;
	//elbowforce << elbowdataY, elbowdataZ;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout <<"shoulderforce:\n"<< shoulderforce << "\n" << "elbowforce:\n"<<elbowforce << endl;

	ForceVector = elbowdata;
}

void FatigueTest::SetZero() {
	UpdataDataArray();
	PostMessage(m_hWnd, CCHART_UPDATE, NULL, (LPARAM)this);
}

void FatigueTest::ExportForceData() {
		ofstream dataFile1;
		dataFile1.open("farocedata.txt", ofstream::app);
		dataFile1 << "shoulderX" << "        " << "shoulderY"<<"        "<<"elbowX"<<"        "<<"elbowY" << endl;
		for (int i = 0; i < force_collection[0].size(); ++i) {
			dataFile1 << force_collection[0][i] << "        " << force_collection[1][i] << "        "<< force_collection[1][i]<<"        "<< force_collection[1][i]<<endl;
		}
		dataFile1.close();
}

void FatigueTest::ExportTorqueData() {
	ofstream dataFile2;
	dataFile2.open("torquedata.txt", ofstream::app);
	for (int i = 0; i < torque_collection[0].size(); ++i) {
		dataFile2 << torque_collection[0][i] << "        " << torque_collection[1][i] <<  endl;
	}
	dataFile2.close();
}

void FatigueTest::ExportMomentData() {
	ofstream dataFile3;
	dataFile3.open("momentdata.txt", ofstream::app);
	for (int i = 0; i < moment_collection[0].size(); ++i) {
		dataFile3 << moment_collection[0][i] << "        " << moment_collection[1][i] << endl;
	}
	dataFile3.close();
}

void FatigueTest::ExportSixDimensionData() {
	ofstream dataFile4;
	dataFile4.open("sixdimensiondata.txt", ofstream::app);
	for (int i = 0; i < sixdimension_collection.size(); ++i) {
		dataFile4 << sixdimension_collection[i] << endl;
	}
	dataFile4.close();
}

void FatigueTest::ExportFatigueData() {
	ofstream dataFile1;
	dataFile1.open("pulldata.txt", ofstream::app);
	for (int i = 0; i < fatigue_collection[0].size(); ++i) {
		dataFile1 << fatigue_collection[0][i] << "        " << fatigue_collection[1][i]
			<< "        " << fatigue_collection[2][i] << "        " << fatigue_collection[3][i] << "        " << fatigue_collection[4][i]
			<< "        " << fatigue_collection[5][i] << endl;
	}
	dataFile1.close();
}