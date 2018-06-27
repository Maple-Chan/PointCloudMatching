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
���ü򵥵�vector�������ݽṹ���������ݴ�ȡ�봦��
������������ʱ���ٷ����ԣ����Ч�ʵ����ݽṹ

*/
class readFile
{
public:
	readFile();//�����Ʒ�װ�÷��ڳ�Ա��
	void readCloud();
	void writeCloud(const vector<PointM>& Cloud3);
	~readFile();
	vector<PointM> cloud1;
	vector<PointM> cloud2;
private:
	vector<double> data; //һ����ʱ�洢���и������ı���

	int c1Len;
	int c2Len;
};
readFile::readFile() {

}
void readFile::readCloud()
{
	//���ļ�
	cloud1.clear();
	cloud2.clear();

	vector<PointM> Cloud1temp;
	vector<PointM> Cloud2temp;

	ifstream fileCloud1;
	fileCloud1.open(CLOUD1);
	ifstream fileCloud2;
	fileCloud2.open(CLOUD2);
	double x = 0;
	//--�����ǵ���1�����ݶ�ȡ
	while (fileCloud1)
	{//�����ݴ�����ʱ��data��
		fileCloud1 >> x;
		data.push_back(x);
	}
	c1Len = data.size();
	PointM temp;
	for (int i = 0; i < c1Len - 6; i+=6)
	{//�� for (int i = 0; i < c1Len/3; i++)�Ǵ���ģ�����
		temp.x = data[i];
		temp.y = data[i + 1];
		temp.z = data[i + 2];
		temp.direction[0] = data[i + 3];
		temp.direction[1] = data[i + 4];
		temp.direction[2] = data[i + 5];

		Cloud1temp.push_back(temp);
		//��������ݷ�װ��Point
	}
	data.clear();//��data���
				//�������յĻ���ô����������cloud2 �� cloud1 ��һ����
	//---------------------------
	//--�����ǵ���2�����ݶ�ȡ
	while (fileCloud2)
	{//�����ݴ�����ʱ��data��
	 //TODO������data ������
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
		//��������ݷ�װ��Point
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
	//��cloud2�������out�ļ���
	int n = Cloud3.size();
	for (int i = 0; i < n; ++i)
	{
		outfile << Cloud3[i].x << " " << Cloud3[i].y << " " << Cloud3[i].z << endl;
	}
}
