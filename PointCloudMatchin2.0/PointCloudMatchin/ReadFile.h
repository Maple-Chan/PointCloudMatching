#pragma once
#include <fstream>
#include <string>
#include "PointCloud.h"
#include <vector>
using namespace std;
//const char* CLOUD1 = "0.asc";
//const char* CLOUD2 = "1.asc";
const char* CLOUD1 = "5.asc";
const char* CLOUD2 = "6.asc";
const char* outCLOUD = "output.asc";
const char* Temp = "1standardRt.asc";
/*
先用简单的vector线性数据结构来进行数据存取与处理
任务后期如果有时间再非线性，提高效率的数据结构

*/
class readFile
{
public:
	readFile();//将点云封装好放在成员中
	void readCloud();
	void writeCloud(const vector<PointM>& Cloud3);
	~readFile();
	vector<PointM> cloud1;
	vector<PointM> cloud2;
private:
	vector<double> data; //一个临时存储所有浮点数的变量

	int c1Len;
	int c2Len;
};
readFile::readFile() {

}
void readFile::readCloud()
{
	//打开文件
	cloud1.clear();
	cloud2.clear();

	vector<PointM> Cloud1temp;
	vector<PointM> Cloud2temp;

	ifstream fileCloud1;
	fileCloud1.open(CLOUD1);
	ifstream fileCloud2;
	fileCloud2.open(CLOUD2);
	double x = 0;
	//--下面是点云1的数据读取
	while (fileCloud1)
	{//将数据存入临时的data中
		fileCloud1 >> x;
		data.push_back(x);
	}
	c1Len = data.size();
	PointM temp;
	for (int i = 0; i < c1Len - 6; i+=6)
	{//用 for (int i = 0; i < c1Len/3; i++)是错误的！！！
		temp.x = data[i];
		temp.y = data[i + 1];
		temp.z = data[i + 2];
		temp.direction[0] = data[i + 3];
		temp.direction[1] = data[i + 4];
		temp.direction[2] = data[i + 5];

		Cloud1temp.push_back(temp);
		//将点的数据封装成Point
	}
	data.clear();//将data清空
				//如果不清空的话那么，读进来的cloud2 和 cloud1 就一样了
	//---------------------------
	//--下面是点云2的数据读取
	while (fileCloud2)
	{//将数据存入临时的data中
	 //TODO：测试data 的数据
		fileCloud2 >> x;
		data.push_back(x);
	}
	c2Len = data.size();

	for (int i = 0; i < c2Len - 6; i += 6)
	{
		temp.x = data[i];
		temp.y = data[i + 1];
		temp.z = data[i + 2];

		temp.direction[0] = data[i + 3];
		temp.direction[1] = data[i + 4];
		temp.direction[2] = data[i + 5];
		Cloud2temp.push_back(temp);
		//将点的数据封装成Point
	}
	cloud1 = Cloud1temp;
	cloud2 = Cloud2temp;

	fileCloud1.close();
	fileCloud2.close();
}

readFile::~readFile()
{
}
void readFile::writeCloud(const vector<PointM>& Cloud3)
{
	ofstream outfile;
	outfile.open(outCLOUD);
	//从cloud2中输出到out文件中
	int n = Cloud3.size();
	for (int i = 0; i < n; ++i)
	{
		outfile << Cloud3[i].x << " " << Cloud3[i].y << " " << Cloud3[i].z << endl;
	}
}
