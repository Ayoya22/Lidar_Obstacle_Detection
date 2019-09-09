
#include "ClusterEngine.h"



template<typename PointT>
void ClusterEngine<PointT>::clusterHelper(int indice,typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    processed[indice] = true;
    cluster->points.push_back(cloud->points[indice]);

    std::vector<int> nearst = tree->search(cloud->points[indice], distanceTol);

    for (int id : nearst)
    {
        if (!processed[id])
            clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusterEngine<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize)
{
	std::vector<bool> processed(cloud->points.size(), false);
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	int i = 0;
    while (i < cloud->points.size())
    {
        if(processed[i])
        {
            i++;
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        clusterHelper(i, cloud, cluster, processed, tree, clusterTolerance);
        if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize )
            clusters.push_back(cluster);
        i++;
    }


 
	return clusters;

}