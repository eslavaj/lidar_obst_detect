/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::vector<Node**> candidate_node_vect;
		int depth = 0;
		int inside_box = 0;
		candidate_node_vect.push_back(&root);

		for(int i = 0; i< candidate_node_vect.size(); i++)
		{
			auto candidate_node = candidate_node_vect[i];
			if(*candidate_node!=NULL)
			{
				for(int i=0; i<target.size(); i++)
				{
					float candidate_coord_tmp = (*candidate_node)->point[i];
					if( fabs(candidate_coord_tmp - target[i]) > distanceTol )
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
					for(int i=0; i<target.size(); i++)
					{
						d += pow((*candidate_node)->point[i] - target[i], 2);
					}
					d = sqrt(d);
					if(d<=distanceTol)
					{
						ids.push_back((*candidate_node)->id);;
					}
				}

				int comparation_dim = depth % (target.size());
				if( target[comparation_dim] - distanceTol < (*candidate_node)->point[comparation_dim] )
				{
					candidate_node_vect.push_back( &((*candidate_node)->left) );
				}
				if( target[comparation_dim] + distanceTol > (*candidate_node)->point[comparation_dim] )
				{
					candidate_node_vect.push_back( &((*candidate_node)->right) );
				}
				depth++;
			}
		}
		return ids;
	}

};




