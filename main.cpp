/**
* @file main.cpp
* @brief 惯导主程序入口.
* @author duwenjie. Email:2023302143033@whu.edu.cn.
* @date 2025-12-20
* @version 1.0
* @note 包含惯导解算的主循环逻辑，根据不同模式处理INS数据.
*/
#include"INS_Structs.h"

using namespace std;

int mode = 1; // 1-示例数据模式 2-真实数据模式

int main()
{
	// 转换文件格式
	/*string RawfilePath = "D:\\大三\\惯导课程\\纯惯导程序调试示例数据\\01 示例数据\\IMU.bin",
		PureINS = "D:\\大三\\惯导课程\\纯惯导程序调试示例数据\\01 示例数据\\PureINS.bin",
		RealRawData = "D:\\大三\\惯导课程\\group3333.ASC";
	string outfilepath = "IMU_data.txt",
		PureINS_outfilepath = "PureINS.txt",
		Real_outfilepath = "RealData.txt";

	readIMUBinaryFile(RawfilePath, outfilepath);
	readOutBinaryFile(PureINS, PureINS_outfilepath);
	ReadRealData(RealRawData, Real_outfilepath);*/

	string filePath = "D:\\大三\\惯导课程\\纯惯导程序调试示例数据\\01 示例数据\\IMU_data.txt",
		real_filepath = "D:\\大三\\惯导课程\\纯惯导程序调试示例数据\\01 示例数据\\RealData.txt";
	
	ifstream infile;
	ofstream outfile("outcome.txt");
	ofstream real_outfile("Real_outcome.txt");
	string line;

	// 滤波数据保存
	vector<INS_RawData> DataBag;
	
	cout << "解算模式" << endl;
	cout << "1-示例数据模式 2-真实数据模式" << endl;
	cin >> mode;

	if (mode == 1) infile.open(filePath);
	else if (mode == 2) infile.open(real_filepath);
	else { cout << "无对应模式！" << endl;  return 0; }
	
	INS_RawData data;
	INS_Epoch epk;

	// 主循环
	while (true)
	{
		getline(infile, line);
		if (!ReadLines(line, data))break;
		
		if (mode == 1) Mode_Sample(data, epk, outfile);
		else if (mode == 2) Mode_Real(DataBag, data, epk, real_outfile);
		
	}

	infile.close();
	outfile.close();
	real_outfile.close();

	return 0;
}