#ifndef CLUSTERENGINE_H
#define CLUSTERENGINE_H



#include <string>
#include <vector>
#include "kdtree.h"



template<typename PointT>
class ClusterEngine
{

    void clusterHelper(int indice,typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
public:

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize);
};

#endif