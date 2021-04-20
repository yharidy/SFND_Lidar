/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
template<typename PointT1>
struct Node
{
	PointT1 point;
	int id;
	Node* left;
	Node* right;

	Node(PointT1 arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT2>
struct KdTree
{
	Node<PointT2>* root;

	KdTree()
	: root(NULL)
	{}

	template<typename PointT3>
	void insertHelper( Node<PointT3>** node, int depth, PointT3 data, int id)
	{
		if (*node == NULL)
		{
			*node = new Node<PointT3>(data, id);
		}
		else
		{
			int cd = depth %3;
			float value1, value2;
			switch (cd) {
			case 0: 
				value1 = data.x;
				value2 = (*node)->point.x;
				break;       
			case 1:
				value1 = data.y;
				value2 = (*node)->point.y;
				break;  
			case 2:
				value1 = data.z;
				value2 = (*node)->point.z;
				break;
			}

			if (value1 < value2)
			{
				insertHelper(&((*node)->left), depth + 1, data, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth + 1, data, id);
			}
		}

	}
	template<typename PointT4>
	void insert(PointT4 point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}
	template<typename PointT5>
	void searchHelper(Node<PointT5>* node, int depth, PointT5 target, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{		
			if ((target.x + distanceTol >= node->point.x) && (target.x - distanceTol <= node->point.x) && (target.y + distanceTol >= node->point.y) && (target.y - distanceTol <= node->point.y) && (target.z + distanceTol >= node->point.z) && (target.z - distanceTol <= node->point.z))
			{
				float dist = sqrt((target.x - node->point.x) * (target.x - node->point.x) + (target.y - node->point.y) * (target.y - node->point.y) + (target.z - node->point.z) * (target.z - node->point.z));

				if (dist < distanceTol)
					ids.push_back(node->id);
			}
			
			int cd = depth % 3;
			float value1, value2;
			switch (cd) {
				case 0:
					value1 = target.x;
					value2 = node->point.x;
					break;
				case 1:
					value1 = target.y;
					value2 = node->point.y;
					break;
				case 2:
					value1 = target.z;
					value2 = node->point.z;
					break;
			}
			if (value1 - distanceTol < value2)
			{
				searchHelper(node->left, depth + 1, target, distanceTol, ids);
			}
			if (value1 + distanceTol > value2)
			{
				searchHelper(node->right, depth + 1, target, distanceTol, ids);
			}
		}
	}
	// return a list of point ids in the tree that are within distance of target
	template<typename PointT6>
	std::vector<int> search(PointT6 target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}

};




