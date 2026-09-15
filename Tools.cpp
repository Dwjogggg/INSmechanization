/**
* @file Tools.cpp
* @brief   实现INS相关辅助函数.
* @author  duwnejie. Email:2023302143033@whu.edu.cn.
* @date    2025-12-20
* @version 1.0
*/
#include"INS_structs.h"

using namespace std;

/**
* @brief 计算子午圈曲率半径.
* @param[in] latitude 纬度（弧度）.
* @return 返回子午圈曲率半径（米）.
* @note 计算公式来源于WGS-84椭球模型.
*/
double RM(double latitude)
{
	return WGS84_a * (1 - WGS84_e2) / pow(1 - WGS84_e2 * sin(latitude) * sin(latitude), 1.5);
}

/**
* @brief 计算卯酉圈曲率半径.
* @param[in] latitude 纬度（弧度）.
* @return 返回卯酉圈曲率半径（米）.
* @note 计算公式来源于WGS-84椭球模型.
*/
double RN(double latitude)
{
	return WGS84_a / sqrt(1 - WGS84_e2 * sin(latitude) * sin(latitude));
}

/**
* @brief 计算重力加速度.
* @param[in] latitude 纬度（弧度）.
* @param[in] h 高度（米）.
* @return 返回重力加速度（米每二次方秒）.
* @note 采用国际重力公式计算重力加速度.
*/
double gravity(double latitude, double h)
{
	double sinLat = sin(latitude);
	double G0 = Ga * (1 + 0.0052790414 * pow(sinLat, 2) + 0.0000232718 * pow(sinLat, 4));
	return G0 - (3.0877e-6 - 4.397731e-9 * pow(sinLat, 2)) * h + 0.72e-12 * h * h;
}

/**
* @brief 三维向量叉乘.
* @param[in] vec1 第一个三维向量.
* @param[in] vec2 第二个三维向量.
* @param[out] vec 叉乘结果.
* @note 结果vec = vec1 x vec2.
*/
void Mat3xMat3(double vec1[], double vec2[], double* vec)
{
	// 调用eigen库实现三维向量叉乘
	Eigen::MatrixXd matA(3, 3);
	Eigen::VectorXd matB(3);
	matA.setZero();
	matB.setZero();
	// 构造反对称矩阵
	matA(0, 1) = -vec1[2]; matA(0, 2) = vec1[1];
	matA(1, 0) = vec1[2];  matA(1, 2) = -vec1[0];
	matA(2, 0) = -vec1[1]; matA(2, 1) = vec1[0];
	for (int i = 0; i < 3; i++) 
	{
		matB(i) = vec2[i];
	}
	Eigen::VectorXd matC = matA * matB;
	for (int j = 0; j < 3; j++)
	{
		vec[j] = matC(j);
	}

	return;
}

/**
* @brief 四元数转换为方向余弦矩阵.
* @param[in] q 输入四元数.
* @param[out] Cbn 输出方向余弦矩阵（3x3数组）.
* @note Cbn为从导航系到机体系的方向余弦矩阵.
*/
void Qua2DCM(const Quaternion& q, double Cbn[])
{
	Cbn[0] = q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3;
	Cbn[1] = 2 * (q.q1 * q.q2 - q.q0 * q.q3);
	Cbn[2] = 2 * (q.q1 * q.q3 + q.q0 * q.q2);
	Cbn[3] = 2 * (q.q1 * q.q2 + q.q0 * q.q3);
	Cbn[4] = q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3;
	Cbn[5] = 2 * (q.q2 * q.q3 - q.q0 * q.q1);
	Cbn[6] = 2 * (q.q1 * q.q3 - q.q0 * q.q2);
	Cbn[7] = 2 * (q.q2 * q.q3 + q.q0 * q.q1);
	Cbn[8] = q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3;
	return;
}

/**
* @brief 旋转向量转换为四元数.
* @param[in] phi 旋转向量（弧度）.
* @param[out] q 输出四元数.
* @note 旋转向量的模表示旋转角度，方向表示旋转轴.
*/
void RotVec2Qua(double phi[], Quaternion& q)
{
	double mol_phi = sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2]);
	double half_mol_phi = mol_phi / 2.0;

	if (mol_phi < 1e-10)
	{
		q.q0 = 1.0;
		q.q1 = q.q2 = q.q3 = 0.0;
	}
	else
	{
		q.q0 = cos(half_mol_phi);
		q.q1 = (phi[0] / mol_phi) * sin(half_mol_phi);
		q.q2 = (phi[1] / mol_phi) * sin(half_mol_phi);
		q.q3 = (phi[2] / mol_phi) * sin(half_mol_phi);
	}

	QuaNormalize(q);

	return;
}

/**
* @brief 欧拉角转换为四元数.
* @param[in] ea 输入欧拉角（弧度）.
* @param[out] q 输出四元数.
* @note 欧拉角顺序为滚转-俯仰-偏航（roll-pitch-yaw.
*/
void EulerToQua(const double ea[], Quaternion &q)
{
	double cos_r = cos(ea[0] / 2.0);
	double sin_r = sin(ea[0] / 2.0);
	double cos_p = cos(ea[1] / 2.0);
	double sin_p = sin(ea[1] / 2.0);
	double cos_y = cos(ea[2] / 2.0);
	double sin_y = sin(ea[2] / 2.0);

	q.q0 = cos_y * cos_p * cos_r + sin_y * sin_p * sin_r;
	q.q1 = cos_y * cos_p * sin_r - sin_y * sin_p * cos_r;
	q.q2 = cos_y * sin_p * cos_r + sin_y * cos_p * sin_r;
	q.q3 = sin_y * cos_p * cos_r - cos_y * sin_p * sin_r;
	QuaNormalize(q);
}

/**
* @brief 四元数乘法.
* @param[in] p 第一个四元数.
* @param[in] q 第二个四元数.
* @param[out] Q 输出乘积四元数.
* @note 四元数乘法表示两个旋转的组合.
*/
void QuaMultiply(const Quaternion& p, const Quaternion& q, Quaternion& Q)
{
	Q.q0 = p.q0 * q.q0 - p.q1 * q.q1 - p.q2 * q.q2 - p.q3 * q.q3;
	Q.q1 = p.q0 * q.q1 + p.q1 * q.q0 + p.q2 * q.q3 - p.q3 * q.q2;
	Q.q2 = p.q0 * q.q2 - p.q1 * q.q3 + p.q2 * q.q0 + p.q3 * q.q1;
	Q.q3 = p.q0 * q.q3 + p.q1 * q.q2 - p.q2 * q.q1 + p.q3 * q.q0;

	QuaNormalize(Q);

	return;
}

/**
* @brief 四元数归一化.
* @param[in,out] q 输入待归一化的四元数，输出归一化后的四元数.
*/
void QuaNormalize(Quaternion& q)
{
	double norm = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
	q.q0 /= norm;
	q.q1 /= norm;
	q.q2 /= norm;
	q.q3 /= norm;
	return;
}

/**
* @brief 四元数转换为欧拉角.
* @param[in] q 输入四元数.
* @param[out] euler 输出欧拉角（弧度）.
* @note 欧拉角顺序为滚转-俯仰-偏航（roll-pitch-yaw）.
*/
void Qua2Euler(const Quaternion& q, double euler[])
{
	euler[0] = atan2(2.0 * (q.q0 * q.q1 + q.q2 * q.q3), 1 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)); // roll
	euler[1] = asin(2.0 * (q.q0 * q.q2 - q.q3 * q.q1));                                         // pitch
	euler[2] = atan2(2.0 * (q.q0 * q.q3 + q.q1 * q.q2), 1 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3)); // yaw

	return;
}


