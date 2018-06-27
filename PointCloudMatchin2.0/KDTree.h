#pragma once
#ifndef KDTREE_H
#define KDTREE_H

#include <cassert>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <cmath>
#include <fstream>
#include <queue>
using ::std::vector;
using ::std::endl;

namespace sx {

	typedef double DataType;
	typedef unsigned int UInt;

	struct Feature {
		vector<DataType> data;//3==data.size();
		DataType dis;//每一次都对Feature进行距离计算，根据这个距离来判断有限队列的排序
		vector<DataType>direction;
		int id;

		Feature() {}
		Feature(const vector<DataType> & d, int i)
			: data(d), id(i) {}
	} /* optional variable list */;

	struct cmp {
		bool operator ()(Feature &a, Feature &b) {//
			return a.dis < b.dis;
		}
	};

	template <UInt K>
	class KDTree {
		
	public:
		std::priority_queue<sx::Feature,std::vector<sx::Feature>,sx::cmp > kclosed;//...如何用优先队列实现k邻近

		KDTree();
		virtual ~KDTree();
		KDTree(const KDTree & rhs);
		const KDTree & operator = (const KDTree & rhs);

		void Clean();
		void Build(const vector<Feature> & matrix_feature);

		int FindNearestFeature(const Feature & target) ;
		int FindNearestFeature(const Feature & target,DataType & min_difference) ;

		void Show() const;

	private:
		
		struct KDNode {
			KDNode * left;
			KDNode * right;
			Feature feature;
			int depth;

			KDNode(const Feature & f, KDNode * lt, KDNode * rt, int d)
				: feature(f), left(lt), right(rt), depth(d) {}
		} /* optional variable list */;

		KDNode * root_;

		struct Comparator {
			int index_comparator;

			Comparator(int ix): index_comparator(ix) {}

			bool operator () (const Feature & lhs, const Feature & rhs) 
			{
				return lhs.data[index_comparator] < rhs.data[index_comparator];
			}
		} /* optional variable list */;

		KDNode * Clone(KDNode * t) ;

		void Clean(KDNode * & t);
		void SortFeature(vector<Feature> & features, int index);
		void Build(const vector<Feature> & matrix_feature,KDNode * & t, int depth);

		DataType Feature2FeatureDifference(const Feature & f1,const Feature & f2) ;
		int FindNearestFeature(const Feature & target,DataType & min_difference, KDNode * t);

		void Show(KDNode * t) ;
	};
	//定义完毕----END
	//下面是函数实现
	template <UInt K>
	KDTree<K>::KDTree(): root_(NULL) {}

	template <UInt K>
	KDTree<K>::~KDTree() 
	{
		Clean();
	}

	template <UInt K>
	KDTree<K>::KDTree(const KDTree & rhs) 
	{
		*this = rhs;
	}

	template <UInt K>
	const KDTree<K> & KDTree<K>::operator = (const KDTree & rhs) 
	{
		if (this != &rhs) {
			Clean();
			root_ = Clone(rhs.root_);
		}
		return *this;
	}

	template <UInt K>
	void KDTree<K>::Clean() 
	{
		Clean(root_);
	}

	template <UInt K>
	void KDTree<K>::Build(const vector<Feature> & matrix_feature) 
	{
		if (matrix_feature.size() != 0) {
			assert(matrix_feature[0].data.size() == K);
		}
		Build(matrix_feature, root_, 0);
	}

	template <UInt K>
	int KDTree<K>::FindNearestFeature(const Feature & target) 
	{
		DataType min_difference;
		return FindNearestFeature(target, min_difference);
	}

	template <UInt K>
	int KDTree<K>::FindNearestFeature(const Feature & target, DataType & min_difference)  {
		min_difference = 10e8;
		return FindNearestFeature(target, min_difference, root_);
	}

	template <UInt K>
	void KDTree<K>::Show() const {
		Show(root_);
		return;
	}


	template <UInt K>
	typename KDTree<K>::KDNode * KDTree<K>::Clone(KDNode * t)  
	{
		if (NULL == t) {
			return NULL;
		}
		return new KDNode(t->feature, t->left, t->right, t->depth);
	}


	template <UInt K>
	void KDTree<K>::Clean(KDNode * & t) 
	{
		if (t != NULL) {
			Clean(t->left);
			Clean(t->right);
			delete t;
		}
		t = NULL;
	}

	template <UInt K>
	void KDTree<K>::SortFeature(vector<Feature> & features, int index) 
	{
		sort(features.begin(), features.end(), Comparator(index));
	}

	template <UInt K>
	void KDTree<K>::Build(const vector<Feature> & matrix_feature, KDNode * & t, int depth) 
	{
		if (matrix_feature.size() == 0) {
			t = NULL;
			return;
		}
		vector<Feature> temp_feature = matrix_feature;
		vector<Feature> left_feature;
		vector<Feature> right_feature;

		SortFeature(temp_feature, depth % K);

		int length = (int)temp_feature.size();
		int middle_position = length / 2;

		t = new KDNode(temp_feature[middle_position], NULL, NULL, depth);

		for (int i = 0; i < middle_position; ++i) {
			left_feature.push_back(temp_feature[i]);
		}

		for (int i = middle_position + 1; i < length; ++i) {
			right_feature.push_back(temp_feature[i]);
		}

		Build(left_feature, t->left, depth + 1);
		Build(right_feature, t->right, depth + 1);

		return;
	}

	template <UInt K>
	DataType KDTree<K>::Feature2FeatureDifference(const Feature & f1, const Feature & f2)  {
		DataType diff = 0.0;
		assert(f1.data.size() == f2.data.size());
		for (int i = 0; i < (int)f1.data.size(); ++i) {
			diff += (f1.data[i] - f2.data[i]) * (f1.data[i] - f2.data[i]);
		}
		return sqrt(diff);
	}

	template <UInt K>
	int KDTree<K>::FindNearestFeature(const Feature & target, DataType & min_difference,KDNode * t)  
	{
		if (NULL == t) {
			return -1;
		}
		Feature temp;
		DataType diff_parent = Feature2FeatureDifference(target, t->feature);
		/*temp.dis = diff_parent;//给当前节点赋距离值
		temp.data = t->feature.data;
		temp.id = t->feature.id;*/

		kclosed.push(temp);//存入优先队列
		//重要的是将所有的target放入优先队列

		DataType diff_left = 10e8;
		DataType diff_right = 10e8;

		int result_parent = -1;
		int result_left = -1;
		int result_right = -1;

		if (diff_parent < min_difference) {
			min_difference = diff_parent;
			result_parent = t->feature.id;
		}

		if (NULL == t->left && NULL == t->right) {
			return result_parent;
		}

		if (NULL == t->left /* && t->right != NULL */) {
			result_right = FindNearestFeature(target, diff_right, t->right);
			if (diff_right < min_difference) {
				min_difference = diff_right;
				result_parent = result_right;
			}
			return result_parent;
		}

		if (NULL == t->right /* && t->left != NULL */) {
			result_left = FindNearestFeature(target, diff_left, t->left);
			if (diff_left < min_difference) {
				min_difference = diff_left;
				result_parent = result_left;
			}
			return result_parent;
		}

		int index_feature = t->depth % K;
		DataType diff_boundary =
			fabs(target.data[index_feature] - t->feature.data[index_feature]);

		if (target.data[index_feature] < t->feature.data[index_feature]) {
			result_left = FindNearestFeature(target, diff_left, t->left);
			if (diff_left < min_difference) {
				min_difference = diff_left;
				result_parent = result_left;
			}
			if (diff_boundary <
				Feature2FeatureDifference(target, t->left->feature)) {
				result_right = FindNearestFeature(target, diff_right, t->right);
				if (diff_right < min_difference) {
					min_difference = diff_right;
					result_parent = result_right;
				}
			}
		}
		else {
			result_right = FindNearestFeature(target, diff_right, t->right);
			if (diff_right < min_difference) {
				min_difference = diff_right;
				result_parent = result_right;
			}
			if (diff_boundary <
				Feature2FeatureDifference(target, t->right->feature)) {
				result_left = FindNearestFeature(target, diff_left, t->left);
				if (diff_left < min_difference) {
					min_difference = diff_left;
					result_parent = result_left;
				}
			}
		}
		return result_parent;
	}

	template <UInt K>
	void KDTree<K>::Show(KDNode * t)  {
		/*需要替换成文件输出*/
		ofstream cout;
		cout.open("KDTreeShow.asc",ios::app);

		cout << "ID: " << t->feature.id << endl;
		cout << "Data: ";
		for (int i = 0; i < (int)t->feature.data.size(); ++i) {
			cout << t->feature.data[i] << " ";
		}
		cout << endl;
		if (t->left != NULL) {
			cout << "Left: " << t->feature.id << " -> " << t->left->feature.id << endl;
			Show(t->left);
		}
		if (t->right != NULL) {
			cout << "Right: " << t->feature.id << " -> " << t->right->feature.id << endl;
			Show(t->right);
		}
		cout.close();
		return;
	}

} /* sx */



#endif /* end of include guard: KDTREE_H */