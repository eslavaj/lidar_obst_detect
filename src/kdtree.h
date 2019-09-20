/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

//#include "../../render/render.h"
#include <iostream>
#include <vector>
#include <string>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int depth = 0;
		Node** current_node = &root;

		while(1)
		{
			if(*current_node==NULL)
			{
				*current_node = new Node(point, id);
				break;
			}
			else
			{
				int comparation_dim = depth % (point.size());
				if( point[comparation_dim] >= (*current_node)->point[comparation_dim])
				{
					current_node = &((*current_node)->right);
				}
				else
				{
					current_node = &((*current_node)->left);
				}
			}
			depth++;
		}
	}

	KdTree(const typename pcl::PointCloud<PointT>::Ptr cloud)
	: root(NULL)
	{
		for (int i=0; i<cloud->points.size(); i++)
		{
			this->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z},i);
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::vector<Node**> candidate_node_vect;
		std::vector<int> depth = {0};
		int inside_box = 0;
		candidate_node_vect.push_back(&root);
		float candidate_coord_tmp;

		for(int i = 0; i< candidate_node_vect.size(); i++)
		{
			auto candidate_node = candidate_node_vect[i];
			if(*candidate_node!=NULL)
			{
				for(int j=0; j<target.size(); j++)
				{
					candidate_coord_tmp = (*candidate_node)->point[j];
					if( fabs(candidate_coord_tmp - target[j]) > distanceTol )
					{
						inside_box = 0;
						break;
					}
					else
					{
						inside_box = 1;
					}
				}

				if(inside_box==1)
				{
					float d = 0;
					for(int k=0; k<target.size(); k++)
					{
						d += powf((*candidate_node)->point[k] - target[k], 2);
					}
					d = sqrt(d);
					if(d<=distanceTol)
					{
						ids.push_back((*candidate_node)->id);;
					}
				}

				int comparation_dim = depth[i] % (target.size());
				if( target[comparation_dim] - distanceTol < (*candidate_node)->point[comparation_dim] )
				{
					candidate_node_vect.push_back( &((*candidate_node)->left) );
					depth.push_back(depth[i] + 1);
				}
				if( target[comparation_dim] + distanceTol > (*candidate_node)->point[comparation_dim] )
				{
					candidate_node_vect.push_back( &((*candidate_node)->right) );
					depth.push_back(depth[i] + 1);
				}
			}
		}
		return ids;
	}

};

#endif


