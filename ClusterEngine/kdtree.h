#ifndef KDTREE_H
#define KDTREE_H

/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> pointXYZ;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	pointXYZ(arr), id(setId), left(NULL), right(NULL)
	{}
};



struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
	template<typename PointT>
    void insertHelper( Node ** node, uint depth, PointT point, int id )
	{
		std::vector<float> pointXYZ {point.x, point.y, point.z};
		if(*node == NULL)
		{	
			*node = new Node(pointXYZ, id);
		}
		else
		{
			// Calculate current dimension
			uint cd = depth%3;

			if(pointXYZ[cd] < ((*node)->pointXYZ[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
		
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if((node->pointXYZ[0] >= (target[0] - distanceTol)) &&
			   (node->pointXYZ[0] <= (target[0] + distanceTol)) &&
			   (node->pointXYZ[1] >= (target[1] - distanceTol)) &&
			   (node->pointXYZ[1] <= (target[1] + distanceTol)) &&
			   (node->pointXYZ[2] >= (target[2] - distanceTol)) &&
			   (node->pointXYZ[2] <= (target[2] + distanceTol)) )
			{
				float distance = sqrt((node->pointXYZ[0] - target[0])*(node->pointXYZ[0] - target[0])
				 + (node->pointXYZ[1] - target[1])*(node->pointXYZ[1] - target[1])
				 + (node->pointXYZ[2] - target[2])*(node->pointXYZ[2] - target[2]));

				if(distance <= distanceTol)
					ids.push_back(node->id);
				
			}

			if((target[depth%3]-distanceTol) < node->pointXYZ[depth%3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((target[depth%3]+distanceTol) > node->pointXYZ[depth%3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	template<typename PointT>
	void insert(PointT point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	template<typename PointT>
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		std::vector<float> pointXYZ {target.x, target.y, target.z};
		searchHelper(pointXYZ, root, 0, distanceTol, ids);
		return ids;
	}
	

};

#endif