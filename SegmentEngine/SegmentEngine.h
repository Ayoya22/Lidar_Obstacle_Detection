#ifndef SEGMENTENGINE_H
#define SEGMENTENGINE_H

#include "../render/render.h"
#include <unordered_set>

class dVector
{
public:
	float x;
	float y;
	float z;
	dVector(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
	dVector udtGetCrossProduct(dVector other)
	{
		float xOut = (this->y * other.z)-(this->z * other.y);
		float yOut = (this->z * other.x)-(this->x * other.z);
		float zOut = (this->x * other.y)-(this->y * other.x);

		return dVector(xOut, yOut, zOut);
	}
};


template<typename PointT>
class SegmentEngine{

public:
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};


#endif