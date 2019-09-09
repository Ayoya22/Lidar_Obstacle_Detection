#include "SegmentEngine.h"



template <typename PointT>
std::unordered_set<int> SegmentEngine<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while(maxIterations--)
    {
        std::unordered_set<int> inliers;
        while(inliers.size() < 3)
            inliers.insert(rand()%(cloud->points.size()));
        
        
        pcl::PointXYZ p1, p2, p3;
        auto itr = inliers.begin();
        p1.x = cloud->points[*itr].x;
        p1.y = cloud->points[*itr].y;
        p1.z = cloud->points[*itr].z;
        itr++;
        p2.x = cloud->points[*itr].x;
        p2.y = cloud->points[*itr].y;
        p2.z = cloud->points[*itr].z;
        itr++;
        p3.x = cloud->points[*itr].x;
        p3.y = cloud->points[*itr].y;
        p3.z = cloud->points[*itr].z;

        dVector v1((p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z));
        dVector v2((p3.x - p1.x), (p3.y - p1.y), (p3.z - p1.z));

        dVector n = v1.udtGetCrossProduct(v2);
        float a = n.x;
        float b = n.y;
        float c = n.z;

        float negD = ((p1.x)*a) + ((p1.y)*b) + ((p1.z)*c);

        float nDistance = sqrt(a*a + b*b + c*c);

        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
                continue;

            PointT point = cloud->points[index];

            float d = fabs(a * point.x + b * point.y + c * point.z - negD) / nDistance;

            if (d < distanceTol)
                inliers.insert(index);
        }

        if(inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
    
    return inliersResult;

}