/**
* @file    INS_Structs.h
* @brief   定义INS相关结构体及函数声明.
* @author  duwenjie. Email:2023302143033@whu.edu.cn
* @date    2025-12-20
* @version 1.0
*/
#pragma once
#include<iostream>
#include<sstream>
#include<string>
#include<fstream>
#include<cmath>
#include<vector>
#include<iomanip>
#include"Eigen/Core"
#include"Eigen/Dense"

using namespace std;

#define PI (3.14159265358979323846)           // 圆周率

#define DEG2RAD (PI/180.0)                    // 度转弧度
#define RAD2DEG (180.0/PI)                    // 弧度转度

// 椭球参数
#define WGS84_a (6378137.0)                   // 长半轴
#define WGS84_b (6356752.3142)                // 短半轴
#define WGS84_e (0.081819190842621)           // 第一偏心率
#define WGS84_e2 (6.69437999014e-3)           // 第一偏心率平方
#define WGS84_GM (3.986004418e14)             // 地球引力常数
#define WGS84_OMEGA (7.292115e-5)             // 地球自转角速度
#define Ga (9.7803267715)                     // 赤道重力加速度
#define Gb (9.8321863685)                     // 极点重力加速度

// IMU参数
#define IMU_FREQ    100.0                     // IMU频率
#define Acc_Scale   IMU_FREQ/(655360.0)       // 加速度比例       
#define Gyro_Scale  IMU_FREQ/(160849.543863)  // 角速度比例
 
/**
* @brief GPS时间结构体.
*	
* GPS时间由周数和周内秒数组成.
*/
struct GPSTime
{
	int week;
	double sec;
	GPSTime()
	{
		week = 0;
		sec = 0.0;
	}
};

/**
* @brief 三维加速度结构体.
* 
* 三维加速度包含x、y、z三个方向的加速度分量.
*/
struct AccData 
{
	double x;
	double y;
	double z;
	AccData()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
};

/**
* @brief 三维角速度结构体.
* 
* 三维角速度包含x、y、z三个方向的角速度分量.
*/
struct GyroData
{
	double x;
	double y;
	double z;
	GyroData()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
};

/**
* @brief 四元数结构体.
*	
* 四元数用于表示旋转和方向.
*/
struct Quaternion
{
	double q0;
	double q1;
	double q2;
	double q3;
	Quaternion()
	{
		q0 = 1.0;
		q1 = 0.0;
		q2 = 0.0;
		q3 = 0.0;
	}
};

/**
* @brief 经纬高结构体.
*	
* 经纬高用于表示地理位置.
*/
struct BLH
{
	double lat;  // 纬度
	double lon;  // 经度
	double h;
	BLH()
	{
		lat = 0.0;
		lon = 0.0;
		h = 0.0;
	}
};

/**
* @brief 北东地速度结构体.
* 
* 北东地速度用于表示在北、东、地三个方向上的速度分量.
*/
struct VelNED
{
	double vn;
	double ve;
	double vd;
	VelNED()
	{
		vn = 0.0;
		ve = 0.0;
		vd = 0.0;
	}
};

/**
* @brief INS原始数据结构体.
* 
* INS原始数据包含时间、加速度和角速度信息.
*/
struct INS_RawData
{
	GPSTime time;
	AccData acc;
	GyroData gyro;
	// VelNED vel;    // 速度增量
};

/**
* @brief INS历元数据结构体.
* 
* INS历元数据包含时间、姿态、位置和速度等信息.
*/
struct INS_Epoch
{
	GPSTime time;
	double euler[3];        // 机体系相对于导航系的欧拉角：roll pitch yaw
	double sita[3], sita0[3];
	double dVel[3], dVel0[3];       
	Quaternion Qb_Upd, Qbn_Last;
	BLH pos_Upd, pos_Last, pos_Last2;
	VelNED vel_Upd, vel_Last, vel_Last2;

    INS_Epoch()  
    { 
       for (int i = 0; i < 3; i++)  
       {  
		   euler[i] = 0.0;
           sita[i] = 0.0;  
           sita0[i] = 0.0;  
           dVel[i] = 0.0;  
           dVel0[i] = 0.0;  
       }  
    }
};

/**
* @brief 读取二进制文件中的双精度浮点数.
* * @param[in] p 指向二进制数据的指针.
* @return 返回读取的双精度浮点数.
*/
double R8(unsigned char* p);

/**
* @brief 读取IMU二进制文件并输出到文本文件.
* * @param[in] filePath 输入二进制文件路径.
* @param[in] outfilepath 输出文本文件路径.
* @return 返回读取是否成功.
*/
bool readIMUBinaryFile(const string& filePath, const string& outfilepath);

/**
* @brief 读取输出二进制文件.
* @param[in] filePath 输入二进制文件路径.
* @param[in] outfilepath 输出文本文件路径.
* @return 返回读取是否成功.
*/
bool readOutBinaryFile(const string& filePath, const string& outfilepath);

/**
* @brief 读取真实数据文件.
* @param[in] infilepath 输入文件路径.
* @param[in] outfilepath 输出文件路径.
* @return 返回读取是否成功.
*/
bool ReadRealData(const string& infilepath, const string& outfilepath);

/**
* @brief 解析一行数据并存储到INS_RawData结构体中.
* @param[in] line 输入数据行.
* @param[out] data 输出INS_RawData结构体.
* @return 返回解析是否成功.
*/
bool ReadLines(const string& line, INS_RawData& data);

/**
* @brief 计算子午圈曲率半径.
* @param[in] latitude 纬度（弧度）.
* @return 返回子午圈曲率半径（米）.
*/
double RM(double latitude);

/**
* @brief 计算卯酉圈曲率半径.
* @param[in] latitude 纬度（弧度）.
* @return 返回卯酉圈曲率半径（米）.
*/
double RN(double latitude);

/**
* @brief 计算重力加速度.
* @param[in] latitude 纬度（弧度）.
* @param[in] h 高度（米）.
* @return 返回重力加速度（米每二次方秒）.
*/
double gravity(double latitude, double h);

/**
* @brief 三维向量点乘.
* @param[in] vec1 第一个三维向量.
* @param[in] vec2 第二个三维向量.
* @param[out] vec 点乘结果.
*/
void Mat3xMat3(double vec1[], double vec2[], double* vec);         

/**
* @brief 四元数归一化.
* @param[in,out] q 输入待归一化的四元数，输出归一化后的四元数.
*/
void QuaNormalize(Quaternion& q);

/**
* @brief 旋转向量转换为四元数.
* @param[in] phi 旋转向量（弧度）.
* @param[out] q 输出四元数.
*/
void RotVec2Qua(double phi[], Quaternion& q);

/**
* @brief 四元数转换为方向余弦矩阵.
* @param[in] q 输入四元数.
* @param[out] Cbn 输出方向余弦矩阵（3x3数组）.
*/
void Qua2DCM(const Quaternion& q, double Cbn[]);

/**
* @brief 欧拉角转换为四元数.
* @param[in] ea 输入欧拉角（弧度）.
* @param[out] q 输出四元数.
*/
void EulerToQua(const double ea[], Quaternion& q);

/**
* @brief 四元数转换为欧拉角.
* @param[in] q 输入四元数.
* @param[out] euler 输出欧拉角（弧度）.
*/
void Qua2Euler(const Quaternion& q, double euler[]);

/**
* @brief 四元数乘法.
* @param[in] p 第一个四元数.
* @param[in] q 第二个四元数.
* @param[out] Q 输出乘积四元数.
*/
void QuaMultiply(const Quaternion& p, const Quaternion& q, Quaternion& Q);



/*机械编排*/

/**
* @brief INS状态更新函数.
* @param[in] data 输入INS原始数据.
* @param[out] epk 输出INS历元数据.
*/
void INS_Update(const INS_RawData& data, INS_Epoch& epk);

/**
* @brief 速度更新函数.
* @param[out] epk 输出INS历元数据.
*/
void VelUpdate(INS_Epoch& epk);

/**
* @brief 位置更新函数.
* @param[out] epk 输出INS历元数据.
*/
void PosUpdate(INS_Epoch& epk);

/**
* @brief 姿态更新函数.
* @param[out] epk 输出INS历元数据.
*/
void AttUpdate(INS_Epoch& epk);

/**
* @brief 零速更新函数.
* @param[out] epk 输出INS历元数据.
*/
void ZeroVelocityUpdate(INS_Epoch& epk);

/**
* @brief 示例模式函数.
* @param[in] data 输入INS原始数据.
* @param[out] epk 输出INS历元数据.
* @param[out] outfile 输出文件流.
*/
void Mode_Sample(INS_RawData& data, INS_Epoch& epk, ofstream& outfile);

/**
* @brief 真实模式函数.
* @param[in] DataBag 输入INS原始数据集合.
* @param[in] data 当前INS原始数据.
* @param[out] epk 输出INS历元数据.
* @param[out] outfile 输出文件流.
*/
void Mode_Real(vector<INS_RawData>& DataBag, INS_RawData& data, INS_Epoch& epk, ofstream& outfile);

/**
* @brief IMU均值滤波函数.
* @param[in] DataBag 输入INS原始数据集合.
* @param[out] data 输出滤波后的INS原始数据.
* @param[out] epk 输出INS历元数据.
*/
void IMU_MeanFilter(vector<INS_RawData>& DataBag, INS_RawData& data, INS_Epoch& epk);