/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>

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
		if (root == NULL)
		{
			root = new Node(point, id);
		} else {
			insert(&root, new Node(point, id), true);
		}

	}

	void insert(Node **node, Node *insertNode, bool compareX)
	{
		if(*node == NULL)
		{
			*node = insertNode;
		}
		else if(compareX)
		{
			if (insertNode->point[0] < (*node)->point[0])
				insert(&(*node)->left, insertNode, !compareX);
			else
				insert(&(*node)->right, insertNode, !compareX);
		}
		else
		{
			if (insertNode->point[1] < (*node)->point[1])
				insert(&(*node)->left, insertNode, !compareX);
			else
				insert(&(*node)->right, insertNode, !compareX);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, target, ids, distanceTol, true);
		return ids;
	}
	void search(Node *node, std::vector<float> target, std::vector<int> &ids, float distanceTol, bool checkX)
	{
		if (node == NULL)
            return;

        if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
            (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)))
        {
            float distance = std::sqrt(std::pow(node->point[0] - target[0], 2) + std::pow(node->point[1] - target[1], 2));
            if (distance <= distanceTol)
                ids.push_back(node->id);
        }

        // Traverse left or right according to the current dimension (X or Y)
        if (checkX)
        {
            if ((target[0] - distanceTol) < node->point[0])
                search(node->left, target, ids, distanceTol, !checkX);
            if ((target[0] + distanceTol) > node->point[0])
                search(node->right, target, ids, distanceTol, !checkX);
        }
        else
        {
            if ((target[1] - distanceTol) < node->point[1])
                search(node->left, target, ids, distanceTol, !checkX);
            if ((target[1] + distanceTol) > node->point[1])
                search(node->right, target, ids, distanceTol, !checkX);
        }
    }
	

};




