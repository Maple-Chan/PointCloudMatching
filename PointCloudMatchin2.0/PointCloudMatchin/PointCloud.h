#pragma once
#include <GL/glut.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// Eigen
#include <Eigen/SVD>
#include <opencv2/highgui/highgui.hpp>
//pcl
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/common/common.h>  
#include <pcl/common/angles.h>  
#include <pcl/common/transforms.h> 
//。。。
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace cv;
using namespace std;
using namespace Eigen;
class PointM {
public:
	double x;
	double y;
	double z;
	unsigned int index;
	double direction[3];
	PointM() :x(0.0), y(0.0), z(0.0), index(0) 
	{	direction[0] = 0.0; 
		direction[1] = 0.0;
		direction[2] = 0.0;
	}
};
class matchitem {
public:
	int index1;
	int index2;
	matchitem() :index1(0), index2(0) {}
	matchitem(int i1,int i2):index1(i1),index2(i2){}
};
class dismatch
{
public:
	matchitem mch;
	double dis;
	dismatch() :mch(),dis(0.0) {}
};
struct functionC {
	int *x;
};
class mPointCloud {
	vector<PointM> cloud1;
	vector<PointM> cloud2;

	
	//functionC C;
	int count;
public:
	PointM Center;
	mPointCloud();
	mPointCloud(const vector<PointM>& c1,const vector<PointM>& c2);
	~mPointCloud() {}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	//在这个类中写算法---针对数组点云
	void ICP(double precision);
	double distance(PointM p1, PointM p2)const;
	void setMatch(vector<matchitem>& match);
	PointM computeGravityCenter();
	void computeRT_Mat(vector<matchitem>& match,Eigen::Matrix3d& R, Eigen::Vector3d& t);  //计算RT转换矩阵
	void computeRT_MatbyPCL(vector<matchitem>& match, Eigen::Matrix4d &transformation_matrix);  //计算RT转换矩阵

	void TransTheCloud2(const Matrix3d& R,const Vector3d& t);
	bool multiMatrix(const Matrix4d& transform,PointM src , PointM& out);
	bool multiMatrix(const Matrix3d& transform, PointM src, PointM& out);

	double leastSqure(vector<matchitem>& match);
	bool isSampleDir(PointM p1, PointM p2);
	double computeAngle(PointM p1, PointM p2);
	//返回变量的接口
	size_t getCloud1Size() { return cloud1.size(); }
	size_t getCloud2Size() { return cloud2.size(); }

	vector<PointM>& getCloud1() { return cloud1; }
	vector<PointM>& getCloud2() { return cloud2; }
	//调试用的辅助函数
	void debugWrite2(int count);
	void debugWrite1(int count);
	void debugWriteLeast(double lic);

	void readMatch();

	void KDTreeBuild();
	void ICP4KDTree(double precision, int times);
	void setMatch4KDTree(vector<matchitem>& match);

};

