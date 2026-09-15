/**
* @file Mechanization.cpp
* @brief 惯导解算函数实现.
* @author duwenjie. Email:2023302143033@whu.edu.cn.
* @date 2025-12-20
* @version 1.0
* @note 包含惯导速度、位置、姿态更新及零速更新等函数的实现.
*/
#include"INS_Structs.h"

using namespace std;

/**
* @brief 速度更新函数.
* @param[in,out] epk INS历元数据结构体，包含位置、速度等信息.
* @note 速度更新基于加速度积分，采用中间时刻外推位置和速度计算地理参数,考虑重力和哥氏力影响.
*/
void VelUpdate(INS_Epoch& epk)
{
	int i;
	double vk[3] = { 0.0 }, vk1[3] = { 0.0 }, vk2[3] = { 0.0 }, 
		dvf[3] = { 0.0 }, dvf_n[3] = { 0.0 }, dvg[3] = { 0.0 },
		pk[3] = { 0.0 }, pk1[3] = { 0.0 }, pk2[3] = { 0.0 },
		vkk[3] = {0.0}, pkk[3] = { 0.0 };
	double temp[3] = { 0.0 };
	double Gp[3] = {0.0}, w_ie[3] = { 0.0 }, w_en[3] = { 0.0 };

	// 赋值位置、速度、角度增量
		// 上一时刻速度
	vk1[0] = epk.vel_Last.vn;
	vk1[1] = epk.vel_Last.ve;
	vk1[2] = epk.vel_Last.vd;
	vk2[0] = epk.vel_Last2.vn;
	vk2[1] = epk.vel_Last2.ve;
	vk2[2] = epk.vel_Last2.vd;
		// 上一时刻位置
	pk1[0] = epk.pos_Last.lat;
	pk1[1] = epk.pos_Last.lon;
	pk1[2] = epk.pos_Last.h;
	pk2[0] = epk.pos_Last2.lat;
	pk2[1] = epk.pos_Last2.lon;
	pk2[2] = epk.pos_Last2.h;

	// 外推中间时刻位置和速度
	for(i=0;i<3;i++)
	{
		vkk[i] = 1.5 * vk1[i] - 0.5 * vk2[i];
		pkk[i] = 1.5 * pk1[i] - 0.5 * pk2[i];
	}
	// 中间时刻计算地理参数
	double Rn = RN(pkk[0]);
	double Rm = RM(pkk[0]);

	Gp[2] = gravity(pkk[0], pkk[2]);

	w_ie[0] = WGS84_OMEGA * cos(pkk[0]);
	w_ie[1] = 0.0;
	w_ie[2] = -WGS84_OMEGA * sin(pkk[0]);

	w_en[0] = vkk[1] / (Rn + pkk[2]);
	w_en[1] = -vkk[0] / (Rm + pkk[2]);
	w_en[2] = -vkk[1] * tan(pkk[0]) / (Rn + pkk[2]);

	double wn[3] = { 0.0 };
	for (i = 0; i < 3; i++)
	{
		wn[i] = 2 * w_ie[i] + w_en[i];
	}
	Mat3xMat3(wn, vkk, temp);
	// 重力哥氏积分项
	for (i = 0; i < 3; i++)
	{
		dvg[i] = (Gp[i] - temp[i]) * 1.0/IMU_FREQ;
	}
	// 比力积分项
	{
		double temp1[3] = { 0.0 }, temp2[3] = { 0.0 }, temp3[3] = { 0.0 }, temp4[3] = { 0.0 }, temp5[3] = { 0.0 },
			phi[3] = { 0.0 };
		Mat3xMat3(epk.sita,epk.dVel,temp1);
		Mat3xMat3(epk.sita0, epk.dVel, temp2);
		Mat3xMat3(epk.dVel0, epk.sita, temp3);
		for (i = 0; i < 3; i++)
		{
			dvf[i] = epk.dVel[i] + 0.5 * temp1[i] + (temp2[i] + temp3[i])/12.0;
		}
		double Cbn[9] = { 0.0 };
		QuaNormalize(epk.Qbn_Last);
		Qua2DCM(epk.Qbn_Last, Cbn);
		// 3x3矩阵与3x1矩阵相乘
		for (i = 0; i < 3; i++)
		{
			temp4[i] = Cbn[i * 3 + 0] * dvf[0] + Cbn[i * 3 + 1] * dvf[1] + Cbn[i * 3 + 2] * dvf[2];
		}
		for (i = 0; i < 3; i++)
		{
			phi[i] = (w_ie[i] + w_en[i]) * 1.0 / IMU_FREQ;
		}
		Mat3xMat3(phi, temp4, temp5);
		for (i = 0; i < 3; i++)
		{
			dvf_n[i] = temp4[i] - 0.5 * temp5[i];
		}
	}
	// 更新速度
	for (i = 0; i < 3; i++)
	{
		vk[i] = vk1[i] + dvf_n[i] + dvg[i];
	}
		// 当前历元速度
	epk.vel_Upd.vn = vk[0];
	epk.vel_Upd.ve = vk[1];
	epk.vel_Upd.vd = vk[2];

	return;
}

/**
* @brief 位置更新函数.
* @param[in,out] epk INS历元数据结构体，包含位置、速度等信息.
* @note 位置更新基于速度积分，采用中间时刻外推速度进行积分.
*/
void PosUpdate(INS_Epoch& epk)
{
	double hkk = 0.0, phi_kk = 0.0, vkk[3] = { 0.0 };
	// 外推中间时刻速度
	vkk[0] = 0.5 * (epk.vel_Last.vn + epk.vel_Upd.vn);
	vkk[1] = 0.5 * (epk.vel_Last.ve + epk.vel_Upd.ve);
	vkk[2] = 0.5 * (epk.vel_Last.vd + epk.vel_Upd.vd);

	// 更新位置
	epk.pos_Upd.h = epk.pos_Last.h - vkk[2] * 1.0 / IMU_FREQ;
	hkk = 0.5 * (epk.pos_Last.h + epk.pos_Upd.h);

	double Rm = RM(epk.pos_Last.lat);
	epk.pos_Upd.lat = epk.pos_Last.lat + (vkk[0] * 1.0 / IMU_FREQ) / (Rm + hkk);
	phi_kk = 0.5 * (epk.pos_Last.lat + epk.pos_Upd.lat);

	double Rn = RN(phi_kk);
	epk.pos_Upd.lon = epk.pos_Last.lon + (vkk[1] * 1.0 / IMU_FREQ) / ((Rn + hkk) * cos(phi_kk));
	
	return;
}

/**
* @brief 姿态更新函数.
* @param[in,out] epk INS历元数据结构体，包含姿态、位置、速度等信息.
* @note 姿态更新基于角度增量积分，采用中间时刻外推位置和速度计算地理参数.
*/
void AttUpdate(INS_Epoch& epk)
{
	double vkk[3] = { 0.0 }, pkk[3] = { 0.0 };
	// 外推中间时刻位置、速度
	vkk[0] = 0.5 * (epk.vel_Last.vn + epk.vel_Upd.vn);
	vkk[1] = 0.5 * (epk.vel_Last.ve + epk.vel_Upd.ve);
	vkk[2] = 0.5 * (epk.vel_Last.vd + epk.vel_Upd.vd);

	pkk[0] = 0.5 * (epk.pos_Last.lat + epk.pos_Upd.lat);
	pkk[1] = 0.5 * (epk.pos_Last.lon + epk.pos_Upd.lon);
	pkk[2] = 0.5 * (epk.pos_Last.h + epk.pos_Upd.h);
	// 计算地理参数
	double Rn = RN(pkk[0]);
	double Rm = RM(pkk[0]);
	double w_en[3] = { 0.0 };
	w_en[0] = vkk[1] / (Rn + pkk[2]);
	w_en[1] = -vkk[0] / (Rm + pkk[2]);
	w_en[2] = -vkk[1] * tan(pkk[0]) / (Rn + pkk[2]);
	double w_ie[3] = { 0.0 };
	w_ie[0] = WGS84_OMEGA * cos(pkk[0]);
	w_ie[1] = 0.0;
	w_ie[2] = -WGS84_OMEGA * sin(pkk[0]);
	double Gp[3] = { 0.0 };
	Gp[2] = gravity(pkk[0], pkk[2]);
	
	double phi[3] = { 0.0 }, temp[3] = { 0.0 },
		yita[3] = { 0.0 }, temp1[3] = { 0.0 };
	Mat3xMat3(epk.sita0, epk.sita, temp);
	for (int i = 0; i < 3; i++)
	{
		phi[i] = epk.sita[i] + temp[i] / 12.0;
	}
	for (int i = 0; i < 3; i++)
	{
		yita[i] = -1.0 * (w_ie[i] + w_en[i]) * 1.0 / IMU_FREQ;
	}

	Quaternion qb, qn, temp_q0, temp_q1;
	RotVec2Qua(phi, qb);
	QuaNormalize(qb);
	RotVec2Qua(yita, qn);
	QuaNormalize(qn);
	// 更新姿态四元数
	QuaMultiply(epk.Qbn_Last, qb, temp_q0);
	QuaNormalize(temp_q0);
	QuaMultiply(qn, temp_q0, temp_q1);
	QuaNormalize(temp_q1);
	double euler[3] = { 0.0 };
	Qua2Euler(temp_q1, euler);
	epk.Qb_Upd = temp_q1;
	epk.euler[0] = euler[0];    // roll
	epk.euler[1] = euler[1];    // pitch
	epk.euler[2] = euler[2];    // yaw

	return;
}

/**
* @brief INS状态更新函数.
* @param[in] data 输入INS原始数据.
* @param[out] epk 输出INS历元数据.
* @note 包含时间推进、角度增量和速度增量更新、位置和速度更新等步骤.
*/
void INS_Update(const INS_RawData& data, INS_Epoch& epk)
{
	// 时间戳向前推进
		// 更新时间
	epk.time.sec = data.time.sec;
		// 角度、速度增量
	epk.sita0[0] = epk.sita[0];
	epk.sita0[1] = epk.sita[1];
	epk.sita0[2] = epk.sita[2];

	epk.dVel0[0] = epk.dVel[0];
	epk.dVel0[1] = epk.dVel[1];
	epk.dVel0[2] = epk.dVel[2];
		// 获取当前时刻角度增量和速度增量
	epk.sita[0] = data.gyro.x * 1.0 / IMU_FREQ;
	epk.sita[1] = data.gyro.y * 1.0 / IMU_FREQ;
	epk.sita[2] = data.gyro.z * 1.0 / IMU_FREQ;

	epk.dVel[0] = data.acc.x * 1.0 / IMU_FREQ;
	epk.dVel[1] = data.acc.y * 1.0 / IMU_FREQ;
	epk.dVel[2] = data.acc.z * 1.0 / IMU_FREQ;

		// 四元数
	epk.Qbn_Last = epk.Qb_Upd;
		// 位置
	epk.pos_Last2 = epk.pos_Last;
	epk.pos_Last = epk.pos_Upd;
		// 速度
	epk.vel_Last2 = epk.vel_Last;
	epk.vel_Last = epk.vel_Upd;

	return;
}

/**
* @brief 零速更新函数.
* @param[in,out] epk INS历元数据结构体，包含位置、速度等信息.
* @note 当速度和角度增量均低于设定阈值时，将速度更新结果置零.
*/
void ZeroVelocityUpdate(INS_Epoch& epk)
{
	double dVel_Norm = 0.0,
		Vel_threshold = 9.85165 / 100.0;             // 速度阈值，单位：m/s
	double dSita_Norm = 0.0,
		Sita_threshold = 0.0006 / 100.0;             // 角度阈值，单位：rad

	dVel_Norm = sqrt(epk.dVel[0] * epk.dVel[0] + epk.dVel[1] * epk.dVel[1] + epk.dVel[2] * epk.dVel[2]);
	dSita_Norm = sqrt(epk.sita[0] * epk.sita[0] + epk.sita[1] * epk.sita[1] + epk.sita[2] * epk.sita[2]);

	if (dVel_Norm < Vel_threshold && dSita_Norm < Sita_threshold)
	{
		epk.vel_Upd.vn = 0.0;
		epk.vel_Upd.ve = 0.0;
		epk.vel_Upd.vd = 0.0;
	}
	else;

	return;
}

/**
* @brief IMU均值滤波函数.
* @param[in] DataBag 输入INS原始数据集合.
* @param[out] data 输出滤波后的INS原始数据.
* @param[out] epk 输出INS历元数据.
* @note 对加速度和角速度进行滑动窗口均值滤波，窗口大小为10个样本（100ms）.
*/
void IMU_MeanFilter(vector<INS_RawData>& DataBag, INS_RawData& data, INS_Epoch& epk)
{
	// 添加当前观测值到滑动窗口，采样率100Hz
	DataBag.push_back(data);

	// 窗口大小
	if (DataBag.size() > 10)
		DataBag.erase(DataBag.begin());

	// 计算加速度和陀螺仪三轴的均值
	double sum_acc_x = 0.0, sum_acc_y = 0.0, sum_acc_z = 0.0;
	double sum_gyro_x = 0.0, sum_gyro_y = 0.0, sum_gyro_z = 0.0;

	for (size_t i = 0; i < DataBag.size(); ++i)
	{
		sum_acc_x += DataBag[i].acc.x;
		sum_acc_y += DataBag[i].acc.y;
		sum_acc_z += DataBag[i].acc.z;
		sum_gyro_x += DataBag[i].gyro.x;
		sum_gyro_y += DataBag[i].gyro.y;
		sum_gyro_z += DataBag[i].gyro.z;
	}

	size_t N = DataBag.size();
	epk.dVel[0] = sum_acc_x / (N * IMU_FREQ);
	epk.dVel[1] = sum_acc_y / (N * IMU_FREQ);
	epk.dVel[2] = sum_acc_z / (N * IMU_FREQ);
	epk.sita[0] = sum_gyro_x / (N * IMU_FREQ);
	epk.sita[1] = sum_gyro_y / (N * IMU_FREQ);
	epk.sita[2] = sum_gyro_z / (N * IMU_FREQ);
}

/**
* @brief 示例模式函数.
* @param[in] data 输入INS原始数据.
* @param[out] epk 输出INS历元数据.
* @param[out] outfile 输出文件流.
* @note 用于处理示例数据，初始化位置、速度和姿态，并输出结果到文件.
*/
void Mode_Sample(INS_RawData &data, INS_Epoch &epk, ofstream &outfile) 
{
	INS_Update(data, epk);

	// Init
	if (epk.time.sec == 91620.0050)
	{
		Quaternion q;
		q.q0 = 0.78921619966;q.q1 = -0.0114037805;q.q2 = -0.014815463;q.q3 = -0.61383079593;
		epk.Qb_Upd = q;
		epk.Qbn_Last = q;
		epk.pos_Last.lat = 23.1373950708 * DEG2RAD;
		epk.pos_Last.lon = 113.3713651222 * DEG2RAD;
		epk.pos_Last.h = 2.175;
		epk.pos_Last2 = epk.pos_Last;
		epk.vel_Last.vn = 0.0;
		epk.vel_Last.ve = 0.0;
		epk.vel_Last.vd = 0.0;
		epk.vel_Last2 = epk.vel_Last;
	}
	if (epk.time.sec - 91620.0 > 1e-5)
	{

		VelUpdate(epk);
		PosUpdate(epk);
		AttUpdate(epk);
		outfile << fixed << setprecision(10)
			<< epk.time.sec << " "
			// 位置
			<< epk.pos_Upd.lat * RAD2DEG << " " << epk.pos_Upd.lon * RAD2DEG << " " << epk.pos_Upd.h << " "
			// 速度
			<< epk.vel_Upd.vn << " " << epk.vel_Upd.ve << " " << epk.vel_Upd.vd << " "
			// 姿态欧拉角(deg)
			<< epk.euler[0] * RAD2DEG << " " << epk.euler[1] * RAD2DEG << " " << epk.euler[2] * RAD2DEG << std::endl;
	}

	return;
}

/**
* @brief 真实模式函数.
* @param[in] DataBag 输入INS原始数据集合.
* @param[out] data 输出滤波后的INS原始数据.
* @param[out] epk 输出INS历元数据.
* @param[out] outfile 输出文件流.
* @note 用于处理真实数据，初始化位置、速度和姿态，并输出结果到文件.
*/
void Mode_Real(vector<INS_RawData>& DataBag, INS_RawData& data, INS_Epoch& epk, ofstream& outfile)
{
	INS_Update(data, epk);  
	IMU_MeanFilter(DataBag, data, epk);
	
	// Init
	if (epk.time.sec == 98279.010)
	{
		/*w = 0.7076142521
			i = -0.0009126867
			j = 0.0001024015
			k = 0.7065610728*/

		// 30.5279708531 114.3556147484 19.676
		Quaternion q;
		double eu[3] = { -0.078084863398340 * DEG2RAD , 0.125305964927434 * DEG2RAD , 89.292609162662690 * DEG2RAD };

		// 89.2876228493   0.1105292883  -0.0995915239
		EulerToQua(eu, q);
		epk.Qb_Upd = q;
		epk.Qbn_Last = q;
		epk.pos_Last.lat = 30.527970844292670 * DEG2RAD;
		epk.pos_Last.lon = 114.3556147193078 * DEG2RAD;
		epk.pos_Last.h = 19.687915471697682;
		epk.pos_Last2 = epk.pos_Last;
		epk.vel_Last.vn = 0.00;
		epk.vel_Last.ve = 0.00;
		epk.vel_Last.vd = 0.00;
		epk.vel_Last2 = epk.vel_Last;
	}
	if (epk.time.sec - 98279.000 > 1e-5)
	{

		VelUpdate(epk);
		PosUpdate(epk);
		AttUpdate(epk);
		ZeroVelocityUpdate(epk);

		outfile << fixed << setprecision(10)
			<< epk.time.sec << " "
			// 位置
			<< epk.pos_Upd.lat * RAD2DEG << " " << epk.pos_Upd.lon * RAD2DEG << " " << epk.pos_Upd.h << " "
			// 速度
			<< epk.vel_Upd.vn << " " << epk.vel_Upd.ve << " " << epk.vel_Upd.vd << " "
			// 姿态欧拉角(deg)
			<< epk.euler[0] * RAD2DEG << " " << epk.euler[1] * RAD2DEG << " " << epk.euler[2] * RAD2DEG << std::endl;
	}

	return;
}