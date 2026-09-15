/**
* @file Readfile.cpp
* @brief 读取IMU和导航解算二进制文件，并转换为文本格式的实现文件.
* @author duwenjie. Email:2023302143033@whu.edu.cn.
* @date 2025-12-20
* @version 1.0
*/
#include"INS_Structs.h"

using namespace std;

/**
* @brief 读取二进制文件中的双精度浮点数.
* @param[in] p 指向二进制数据的指针.
* @return 返回读取的双精度浮点数.
* @note 内存拷贝方式读取，避免对齐问题.
*/
double R8(unsigned char* p)
{
    double r;
    memcpy(&r, p, 8);
    return r;
}

/**
* @brief 解码IMU二进制文件，转为.txt文本文件
* @param[in] filePath 输入二进制文件路径.
* @param[in] outfilepath 输出文本文件路径.
* @return 返回读取是否成功.
* @note 陀螺仪和加速度计数据均乘以200缩放系数.
*/
bool readIMUBinaryFile(const string& filePath, const string& outfilepath)
{
    std::ifstream file(filePath, std::ios::binary | std::ios::in);
    if (!file.is_open()) {
        std::cerr << "错误：无法打开文件 -> " << filePath << std::endl;
        return 0;
    }

	ofstream outfile(outfilepath);

    unsigned char byteBuf[56];
    const size_t doubleSize = 8;  // double 8字节

    while (true) {
        
        if (!file.read(reinterpret_cast<char*>(byteBuf), 7 * doubleSize))break;
        // 时间秒数（sec）
        double sec = R8(byteBuf);

        // 陀螺仪X/Y/Z
        double gyro_x = 200.0 * R8(byteBuf + 8);
        double gyro_y = 200.0 * R8(byteBuf + 16);
        double gyro_z = 200.0 * R8(byteBuf + 24);

        // 加速度X/Y/Z
        double acc_x = 200.0 * R8(byteBuf + 32);
        double acc_y = 200.0 * R8(byteBuf + 40);
        double acc_z = 200.0 * R8(byteBuf + 48);

        // 写入解码后的数据
        outfile << fixed << setprecision(10)
            << sec << " "
            << acc_x << " " << acc_y << " " << acc_z << " "
            << gyro_x << " " << gyro_y << " " << gyro_z << std::endl;
    }
	file.close();
	outfile.close();

    return 1;
}

/**
* @brief 解码输出二进制文件，转为.txt文本文件
* @param[in] filePath 输入二进制文件路径.
* @param[in] outfilepath 输出文本文件路径.
* @return 返回读取是否成功.
* @note 输出数据包含时间、位置、速度和姿态角.
*/
bool readOutBinaryFile(const string& filePath, const string& outfilepath)
{
    std::ifstream file(filePath, std::ios::binary | std::ios::in);
    if (!file.is_open()) {
        std::cerr << "错误：无法打开文件 -> " << filePath << std::endl;
        return 0;
    }

    ofstream outfile(outfilepath);

    unsigned char byteBuf[80];
    const size_t doubleSize = 8;  // double 8字节

    while (true) {

        if (!file.read(reinterpret_cast<char*>(byteBuf), 10 * doubleSize))break;
        // 时间秒数（sec）
        double sec = R8(byteBuf);

        // BLH
		double lat = R8(byteBuf + 8);
		double lon = R8(byteBuf + 16);
		double h = R8(byteBuf + 24);

		// 速度NED
		double vn = R8(byteBuf + 32);
		double ve = R8(byteBuf + 40);
		double vd = R8(byteBuf + 48);

        // 姿态角（deg）
		double roll = R8(byteBuf + 56);
		double pitch = R8(byteBuf + 64);
		double yaw = R8(byteBuf + 72);

        // 写入解码后的数据
        outfile << fixed << setprecision(10)
            << sec << " "
			<< lat << " " << lon << " " << h << " "
			<< vn << " " << ve << " " << vd << " "
			<< roll << " " << pitch << " " << yaw << std::endl;
    }
    file.close();
    outfile.close();

    return 1;
}

/**
* @brief 解析一行数据并存储到INS_RawData结构体中.
* @param[in] line 输入数据行.
* @param[out] data 输出INS_RawData结构体.
* @return 返回解析是否成功.
* @note 解析格式：sec acc_x acc_y acc_z gyro_x gyro_y gyro_z.
*/
bool ReadLines(const string& line,INS_RawData& data) 
{
	std::istringstream iss(line);
	if (!(iss >> data.time.sec
		>> data.acc.x >> data.acc.y >> data.acc.z
		>> data.gyro.x >> data.gyro.y >> data.gyro.z))
	{
		return false; // 解析失败
	}
	return true;      // 解析成功
}

/**
* @brief 读取真实数据文件.
* @param[in] infilepath 输入文件路径.
* @param[in] outfilepath 输出文件路径.
* @return 返回读取是否成功.
* @note 仅解析以#RAWIMUA开头的行，其他行跳过.
*/
bool ReadRealData(const std::string& infilepath, const std::string& outfilepath)
{
    // 1. 打开输入输出文件并校验
    std::ifstream file(infilepath);
    std::ofstream outfile(outfilepath);

    // 2. 定义存储变量
    INS_RawData data;
    std::string line;
    const std::string DATA_HEADER = "#RAWIMUA"; // 目标数据头

    // 3. 逐行读取数据
    while (std::getline(file, line))
    {
        // 跳过非目标数据行（仅处理以#RAWIMUA开头的行，其余行（乱码/无关）直接跳过）
        if (line.substr(0, DATA_HEADER.length()) != DATA_HEADER) {
            continue;
        }

        try { 
            std::stringstream ss(line);
            char sharp;
            std::string temp;

            // 解析开头#和RAWIMUA标识符
            ss >> sharp; // 读取#
            std::getline(ss, temp, ','); // 读取RAWIMUA（丢弃）

            // 跳过#后到;前的所有无用字段（COM1,0,113.0,FINE,2391,97979.190,0,21730,14102）
            std::getline(ss, temp, ';');

            // 解析周数和周内秒
            std::getline(ss, temp, ',');
            data.time.week = std::stoi(temp); // week:2391
            std::getline(ss, temp, ',');
            data.time.sec = std::stod(temp); // sec:97979.190

            // 跳过imu state字段（00007377）
            std::getline(ss, temp, ',');

            // 解析加速度计Z/Y/X（Y轴取反，乘以缩放系数）
            std::getline(ss, temp, ',');
            data.acc.z = std::stoi(temp) * Acc_Scale;        // Acc Z:64250
            std::getline(ss, temp, ',');
            data.acc.y = -1.0 * std::stoi(temp) * Acc_Scale;   // Acc Y:70（取反）
            std::getline(ss, temp, ',');
            data.acc.x = std::stoi(temp) * Acc_Scale;        // Acc X:185

            // 解析陀螺仪Z/Y/X（Y轴取反，乘以缩放系数）
            std::getline(ss, temp, ',');
            data.gyro.z = std::stoi(temp) * Gyro_Scale * DEG2RAD;        // Gyro Z:130
            std::getline(ss, temp, ',');
            data.gyro.y = -1.0 * std::stoi(temp) * Gyro_Scale * DEG2RAD; // Gyro Y:-29（取反）
            std::getline(ss, temp, '*');
            data.gyro.x = std::stoi(temp) * Gyro_Scale * DEG2RAD;        // Gyro X:7

			// 轴系调整，右前上系转为前右下系
			double temp_acc = data.acc.x;
			data.acc.x = data.acc.y;
			data.acc.y = temp_acc;
			data.acc.z = -data.acc.z;
			double temp_gyro = data.gyro.x;
			data.gyro.x = data.gyro.y;
			data.gyro.y = temp_gyro;
			data.gyro.z = -data.gyro.z;

            // 输出解析结果（保留10位小数）
            outfile << std::fixed << std::setprecision(10)
                << data.time.sec << " "
                << data.acc.x << " " << data.acc.y << " " << data.acc.z << " "
                << data.gyro.x << " " << data.gyro.y << " " << data.gyro.z << std::endl;
        }
        catch (const std::exception& e) {
            // 解析异常（乱码/非数值）：跳过当前行并打印提示
            std::cerr << "警告：解析行失败（乱码/格式错误），行内容：" << line << std::endl;
            std::cerr << "异常信息：" << e.what() << std::endl;
            continue;
        }
    }

    // 关闭文件
    file.close();
    outfile.close();

    return true;
}
