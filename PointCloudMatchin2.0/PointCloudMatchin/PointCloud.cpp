#include "stdafx.h"
#include "PointCloud.h"


using namespace cv;


const char* CLOUD3 = "6.asc";
const string filename = "outCloud";
static int leastc = 0;
void sortByDis(vector<dismatch>& closeSet);


mPointCloud::mPointCloud()
{
}
mPointCloud::mPointCloud(const vector<PointM>& c1,const vector<PointM>& c2):count(0)
{
	
	cloud1 = c1;
	cloud2 = c2;
	int nc1 = cloud1.size();
	int nc2 = cloud2.size();  //�����С�Ľ��б����Ļ���ôֻ�ܱ���ǰ��n����
	int n = nc1 > nc2 ? nc2 : nc1;
	Center = computeGravityCenter();
}


void mPointCloud::ICP(double precision)
{

	//��ʼICP�㷨
	//�������
	double least;
	vector<matchitem> match;
	do
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		//1-���öԳƵ㣨�������Ϊ������
		setMatch(match);
		//...END

		//readMatch();//for test ���Rt�ĺ�����
		//2-����仯����
		computeRT_Mat(match,R, t);
		//...END
 
		/*R(0, 0) = 0.903067498016661; R(0, 1) = -0.0114543820083601; R(0, 2) = -0.429345887553077;
		R(1, 0) = 0.0117542776251332; R(1,1) = 0.999929008170884; R(1, 2) = -0.00195334991953774;
		R(2, 0) = 0.429337781919366; R(2, 1) = -0.0032826439349211; R(2, 2) = 0.903138025589308;


		t[0] = -184.889081882023;
		t[1] = 1.55270732820347;
		t[2] = 40.5364071654102;*/
		/***************************************/
		///�Լ�д��Rt�ͺ������������Rt�Ѿ���һ������~
		/***************************************/

		//3-��ñ任��Ľ��
		TransTheCloud2(R, t);

		//4-����ƽ������Ľ��
		least = leastSqure(match);

		debugWriteLeast(least);
	}
	while (least > precision);


}

double mPointCloud::distance(PointM p1, PointM p2)const
{//��������ľ���
	return sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z) );
}

void mPointCloud::setMatch(vector<matchitem>& match)
{
	int nc1 = cloud1.size();
	int nc2 = cloud2.size();  //ȱ��:�����С�Ľ��б����Ļ���ôֻ�ܱ���ǰ��n����
							  //int n = nc1 > nc2 ? nc2 : nc1;
	match.clear();
	match.reserve(0);

	double min;
	double dis;
	int y;

	//ɸѡ����1.�ȶԾ������ɸѡ��ѡ�е���С����ĵ㣬�ٽ��и��ݽǶ�ɸѡ
	/*ofstream mindis;
	mindis.open("mindis.txt");*/

	for (int i = 0; i < nc1; ++i)
	{//�Ը����������С�����ƥ��
		min = 9999999.9;
		y = -1;

		for (int j = 0; j < nc2; ++j)
		{
			dis = distance(cloud1[i], cloud2[j]);

			//�Ծ���ĵ����ɸѡ������̫��ľͲ���Ϊ��Ӧ�ĵ�
			/*if (0 == leastc)
			{
				if (dis > 1000)
					continue;
			}
			else if(2==leastc)
			{
				if (dis > 500)
					continue;
			}
			else
			{
				if (dis > 200)
					continue;
			}*/

			//ѡ�������С�ģ����Ҽ�¼���Ƕ�Ӧ���±�
			if (dis < min)
			{
				min = dis;
				y = j;
			}
		}
		if (!isSampleDir(cloud1[i], cloud2[y]))
			continue;

		if (-1 != y)//���������� && ���·�����ͬ������Ϊ��Ӧ��
		{
			matchitem item = matchitem(i,y);
			match.push_back(item);
			//mindis << item.index1 << " " << item.index2 << endl;
		}
	}//����ӳ���ϵ����ENDL*/
	//��ӳ���ϵת��Ϊ������Ϊ��׼��һһ��Ӧ
	
	//mindis.close();

	//����2.�ҵ�һ���������ĵ㼯���ڸõ㼯���ҽǶ�ƫ����С�ġ�
	/*size_t setNum = 3;
	
	vector<dismatch> closeSet;
	vector<PointM> c1 = cloud1, c2 = cloud2;
	dismatch item;
	matchitem mitem(0,0);
	for (int i = 0; i < nc1; ++i)
	{//�Ը����������С�����ƥ��
		min = 9999999.9;
		y = -1;
		closeSet.clear();
		for (int j = 0; j < nc2; ++j)
		{
			dis = distance(cloud1[i], cloud2[j]);
			mitem.index1 = i, mitem.index2 = j;
			item.dis = dis;
			item.mch = mitem;
			if(closeSet.size()<setNum)
			{//�����С������㼯�����Ļ���ֱ�Ӵ���
				closeSet.push_back(item);
			}
			else
			{//������ڵ�������㼯����
				//���򣬰����Ļ���
				sortByDis(closeSet);
				if (closeSet[0].dis > item.dis) 
				{
					closeSet[0] = item;//����¼����disС����ô�������Ǹ��滻��
				}
			}
		}
		//����һ�������С�㼯�Ѿ��������
		//����С�㼯���ҽǶ���С��;
		double minAngle=9999.9;

		for (size_t k = 0; k < setNum; ++k)
		{
			double an=computeAngle(cloud1[closeSet[k].mch.index1], cloud2[closeSet[k].mch.index2]);
			if (an < minAngle)
			{
				minAngle = an;
				y = closeSet[k].mch.index2;
			}
		}

		if (-1 != y)//�������Ͻ� && �Ƕ�ƫ����С
		{
			matchitem item = matchitem(i, y);
			match.push_back(item);
		}
	}
	//����ӳ���ϵ����ENDL*/
}

void sortByDis(vector<dismatch>& closeSet)
{
	size_t N = closeSet.size();
	dismatch temp;
	for (size_t i = 0; i < N; ++i)
	{
		bool flag = false;
		for (size_t j = 0; j < N-i-1; ++j)
		{
			if (closeSet[j].dis < closeSet[j + 1].dis)//�����һ����ǰһ����
			{
				temp = closeSet[j];
				closeSet[j] = closeSet[j + 1];
				closeSet[j + 1] = temp;
				flag = true;
			}
		}
		if (!flag)return;//���һ��Ҳû�н�����ô���Ѿ��ź��򣬿�����ǰ����
	}
}

void mPointCloud::readMatch()
{
	for (size_t i = 0; i < cloud1.size(); ++i)
	{//�Ը����������С�����ƥ��
		cloud1[i].index = i;  //f(x) = y ��ӳ���ϵ   yΪcloud2���±�
	}
}

bool mPointCloud::isSampleDir(PointM p1, PointM p2)
{
	double threshold = 5;//�Ƕȵ���ֵ
	double delta;
	
	//�������������ļн�
	double ab = p1.direction[0] * p2.direction[0] + p1.direction[1] * p2.direction[1] + p1.direction[2] * p2.direction[2];
	double _a_ = sqrt(p1.direction[0] *p1.direction[0] + p1.direction[1] *p1.direction[1] + p1.direction[2] *p1.direction[2]);
	double _b_ = sqrt(p2.direction[0] *p2.direction[0] + p2.direction[1] *p2.direction[1] + p2.direction[2] *p2.direction[2]);
	double cosab = ab / (_a_ * _b_);//�����нǵ�cosֵ

	delta = acos(cosab)/3.1415926 * 180;//�����Ǻ�����

	if (delta < threshold)
		return true;
	return false;
}

double mPointCloud::computeAngle(PointM p1, PointM p2)
{
	double delta;
	double threshold =5;//�Ƕȵ���ֵ

	//�������������ļн�
	double ab = p1.direction[0] * p2.direction[0] + p1.direction[1] * p2.direction[1] + p1.direction[2] * p2.direction[2];
	double _a_ = sqrt(p1.direction[0] * p1.direction[0] + p1.direction[1] * p1.direction[1] + p1.direction[2] * p1.direction[2]);
	double _b_ = sqrt(p2.direction[0] * p2.direction[0] + p2.direction[1] * p2.direction[1] + p2.direction[2] * p2.direction[2]);
	double cosab = ab / (_a_ * _b_);//�����нǵ�cosֵ

	delta = acos(cosab) / 3.1415926 * 180;//�����Ǻ�����

	if (delta < threshold)
		return delta;
	return 99999999.9;

}

PointM mPointCloud::computeGravityCenter()
{//������Ƶ�����
	PointM c;
	for (unsigned i = 0; i < cloud1.size(); ++i)
	{
		c.x += cloud1[i].x;
		c.y += cloud1[i].y;
		c.z += cloud1[i].z;
	}
	for (unsigned i = 0; i < cloud2.size(); ++i)
	{
		c.x += cloud2[i].x;
		c.y += cloud2[i].y;
		c.z += cloud2[i].z;
	}
	c.x /= (cloud1.size() + cloud2.size());
	c.y /= (cloud1.size() + cloud2.size());
	c.z /= (cloud1.size() + cloud2.size());

	return c;
}

void mPointCloud::computeRT_Mat(vector<matchitem>& match,Eigen::Matrix3d& R, Eigen::Vector3d& t)
{//����RTת������

	PointM p1 = PointM(), p2 = PointM();
	int nmatch = match.size();
	int N = nmatch;
	for (int i = 0; i < N; ++i)
	{
		
		p1.x += cloud1[match[i].index1].x;    //��Ϊ�� 2 -�� 1 ����Ҫ�� cloud2 �� һ�� ��cloud2 ��Ӧ�������Rt����
		p1.y += cloud1[match[i].index1].y;
		p1.z += cloud1[match[i].index1].z;
		
		p2.x += cloud2[match[i].index2].x;
		p2.y += cloud2[match[i].index2].y;
		p2.z += cloud2[match[i].index2].z;
		///test << "i="<<i<<" index="<<cloud2[i].index << endl;
		
	}
	p1.x /= N;	p1.y /= N;	p1.z /= N;
	p2.x /= N;	p2.y /= N;	p2.z /= N;
	
	//�õ�ȥ�������꣬������ѧϰΪʲôҪ�õ���ȥ�������ꣿ

	vector<PointM> q1(N), q2(N);
	for (int i = 0; i < N; ++i)
	{
		q1[i].x = cloud1[match[i].index1].x - p1.x;
		q1[i].y = cloud1[match[i].index1].y - p1.y;
		q1[i].z = cloud1[match[i].index1].z - p1.z;

		q2[i].x = cloud2[match[i].index2].x - p2.x; //��cloud2 �ж�Ӧcloud2�ĵ����svd
		q2[i].y = cloud2[match[i].index2].y - p2.y;
		q2[i].z = cloud2[match[i].index2].z - p2.z;
	}

	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();

	for (int i = 0; i < N; ++i)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	//��W��������ֽ�
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	//������ת ��ƽ�ƾ��� R  �� t
	Eigen::Matrix3d R_ = U * (V.transpose());
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) 
		- R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
	R = R_;
	t = t_;
	ofstream ofile1;
	string name = "Rt.asc";
	ofile1.open(name, ios::app);
	ofile1 << R << endl;
	ofile1 << t << endl;
	ofile1.close();
	///test.close();
	
}

void mPointCloud::computeRT_MatbyPCL(vector<matchitem>& match, Eigen::Matrix4d &transformation_matrix)
{
	/*���ҵĵ�����ת����PCL�õ�������ת������*/

	/*//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//pcl::computeCovarianceMatrixNormalized();
	const int npts = match.size();
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts);

	for (int i = 0; i < npts; ++i)
	{
		cloud_src(0, i) = cloud1[match[i].index1].x;
		cloud_src(1, i) = cloud1[match[i].index1].y;
		cloud_src(2, i) = cloud1[match[i].index1].z;
		

		cloud_tgt(0, i) = cloud2[match[i].index2].x;
		cloud_tgt(1, i) = cloud2[match[i].index2].y;
		cloud_tgt(2, i) = cloud2[match[i].index2].z;
	}
	transformation_matrix = Eigen::umeyama(cloud_src,cloud_tgt,false);//PCL���е�SVD���Rt����*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

	cloud_in->width = 4;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->resize(match.size());

	cloud_out->width = 4;
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->resize(match.size());

	for (int i = 0; i<match.size(); i++)
	{
		cloud_in->points[i].x = cloud1[match[i].index1].x;
		cloud_in->points[i].y = cloud1[match[i].index1].y;
		cloud_in->points[i].z = cloud1[match[i].index1].z;
	}
	for (int i = 0; i<match.size(); i++)
	{
		cloud_in->points[i].x = cloud2[match[i].index2].x;
		cloud_in->points[i].y = cloud2[match[i].index2].y;
		cloud_in->points[i].z = cloud2[match[i].index2].z;
	}

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
	TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);

}
void mPointCloud::TransTheCloud2(const Matrix3d& R,const Vector3d& t)
{//������ƾ���Rt�任
	Matrix4d transform;

	transform(0, 0) = R(0, 0); transform(0, 1) = R(0, 1); transform(0, 2) = R(0, 2); transform(0, 3) = t[0];
	transform(1, 0) = R(1, 0); transform(1, 1) = R(1, 1); transform(1, 2) = R(1, 2); transform(1, 3) = t[1];
	transform(2, 0) = R(2, 0); transform(2, 1) = R(2, 1); transform(2, 2) = R(2, 2); transform(2, 3) = t[2];
	transform(3, 0) = 0		 ; transform(3, 1) = 0		; transform(3, 2) = 0	   ; transform(3, 3) = 1;

	ofstream ofile1;
	string name = "M4d.asc";
	ofile1.open(name,ios::app);
	ofile1 << transform << endl;
	  //���������4ά����û�����⡣

	//���м���
	for (size_t i = 0; i < cloud2.size(); ++i)
	{
		multiMatrix(transform, cloud2[i], cloud2[i]);//-----�Ե������ϵ����ת��
		//��������������ת��
		multiMatrix(R, cloud2[i], cloud2[i]);
	}
	//cloud1[0]�ĳ˷������Ѿ�����
	ofile1.close();


}

bool mPointCloud::multiMatrix(const Matrix4d& transform, PointM src,PointM& out)
{
	//�ϲ���ת��һ��ת������
	Vector4d vsrc ( src.x, src.y, src.z, 1);
	Vector4d temp (
	transform(0, 0) * vsrc[0]+ transform(0, 1) * vsrc[1]+ transform(0, 2)  * vsrc[2]+ transform(0, 3)  * vsrc[3],
	transform(1, 0) * vsrc[0]+ transform(1, 1) * vsrc[1]+ transform(1, 2)  * vsrc[2]+ transform(1, 3)  * vsrc[3],
	transform(2, 0) * vsrc[0]+ transform(2, 1) * vsrc[1]+ transform(2, 2)  * vsrc[2]+ transform(2, 3)  * vsrc[3],
	transform(3, 0) * vsrc[0]+ transform(3, 1) * vsrc[1]+ transform(3, 2)  * vsrc[2]+ transform(3, 3)  * vsrc[3]
	);


	out.x = temp[0];
	out.y = temp[1];
	out.z = temp[2];
	return true;
}
bool mPointCloud::multiMatrix(const Matrix3d& transform, PointM src, PointM& out)
{
	//�ϲ���ת��һ��ת������
	Vector3d vsrc(src.direction[0], src.direction[1], src.direction[2]);
	Vector3d temp = transform * vsrc;


	out.direction[0] = temp[0];
	out.direction[1] = temp[1];
	out.direction[2] = temp[2];
	return true;
}

double mPointCloud::leastSqure(vector<matchitem>& match)
{
	double result=0.0;
	int n = match.size();

	for (int i = 0; i < n; ++i)
	{
		
		result += distance(cloud1[match[i].index1], cloud2[match[i].index2]);
		//result += distance(cloud1[i], cloud2[C.x[i]]) * distance(cloud1[i], cloud2[C.x[i]]);
	}
	result /= n;

	return result;
}

void mPointCloud::debugWrite2(int count)
{
	ofstream ofile;
	string name = filename + to_string(count)+"_2_0.asc";
	ofile.open(name);

	int n = cloud2.size();
	for (int i = 0; i < n; ++i)
	{
		ofile << cloud2[i].x << " " << cloud2[i].y << " " << cloud2[i].z << endl;
	}
}
void mPointCloud::debugWrite1(int count)
{
	ofstream ofile;
	string name = filename + to_string(count) + "_3_0.asc";
	ofile.open(name);

	int n = cloud1.size();
	for (int i = 0; i < n; ++i)
	{
		ofile << cloud1[i].x << " " << cloud1[i].y << " " << cloud1[i].z << endl;
	}
}
void mPointCloud::debugWriteLeast(double least)
{
	
	ofstream ofile;
	string name = "Least_3_0.asc";
	ofile.open(name, ios::app);
	ofile << least << endl;
	ofile.close();


	ofstream ofileCloud;
	name = filename + to_string(leastc) + "_3_0.asc";
	ofileCloud.open(name);
	leastc++;
	int n = cloud2.size();
	for (int i = 0; i < n; ++i)
	{
		ofileCloud << cloud2[i].x << " " << cloud2[i].y << " " << cloud2[i].z <<" "
			<<cloud2[i].direction[0]<< " " <<cloud2[i].direction[1]<<" "<<cloud2[i].direction[2]<< endl;
	}
}

///===========================================================

void mPointCloud:: ICP4KDTree(double precision, int times)
{
	

	//��ʼICP For KDTree�㷨
	double least;
	vector<matchitem> match;
	do
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		Eigen::Matrix4d trans;
		//1-���öԳƵ㣨�������Ϊ������
		setMatch4KDTree(match);
		//...END
		//readMatch();//for test ���Rt�ĺ���
		//2-����仯����
		computeRT_Mat(match, R,t);
		//...END

		//3-��ñ任��Ľ��
		TransTheCloud2(R, t);



		//4-����ƽ������Ľ��
		least = leastSqure(match);

		debugWriteLeast(least);
	} while (least > precision && leastc < times);
}

void mPointCloud::KDTreeBuild()
{
	const int K = 3;
	//��cloud1 cloud2 ����KDtree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(cloud2.size());//���¸����Ƹ�ֵ

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = cloud2[i].x;
		cloud->points[i].y = cloud2[i].y;
		cloud->points[i].z = cloud2[i].z;
	}
	kdtree.setInputCloud(cloud);//�������
}


void mPointCloud::setMatch4KDTree(vector<matchitem>& match)
{

	//The tree's id is equal to cloud1/2' index;  
	//Can we just mark the id but computing the Rt and Transforming the Cloud2 use the old function?  Yeah of course;
	KDTreeBuild();
	match.clear();
	match.reserve(0);
	
	//ofstream mindis;
	//mindis.open("mindis.txt");
	int K = 1;//����K �ڽ���ѯ
	std::vector<int> pointIdxNKNSearch(K); //
	std::vector<float> pointNKNSquaredDistance(K);

	pcl::PointXYZ searchPoint;
	double minAngle;
	int matchIdx;

	for (int i = 0; i < cloud1.size(); ++i)
	{
		minAngle = 10;//����������cloud2�ĵ�
		matchIdx = -1;
		searchPoint.x = cloud1[i].x;
		searchPoint.y = cloud1[i].y;
		searchPoint.z = cloud1[i].z;
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
			{
				if (isSampleDir(cloud1[i], cloud2[pointIdxNKNSearch[j]]))
				//if (computeAngle(cloud1[i], cloud2[ pointIdxNKNSearch[j] ]) < minAngle)
				{
					matchIdx = pointIdxNKNSearch[j];
				}
			}
		}
		if (matchIdx != -1)
		{
			matchitem item = matchitem(i, matchIdx);
			match.push_back(item);
		}
	}
	//mindis.close();

}
///����KDTree���㷨*/