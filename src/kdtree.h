#include <cmath>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (root == NULL)
		{
			root = new Node<PointT>(point, id);
		} else {
			insert(&root, new Node<PointT>(point, id), 0);
		}

	}

	void insert(Node<PointT> **node, Node<PointT> *insertNode, int level)
	{
		if(*node == NULL)
		{
			*node = insertNode;
		}
		else if(level % 3 == 0)
		{
			if (insertNode->point.x < (*node)->point.x)
				insert(&(*node)->left, insertNode, level + 1);
			else
				insert(&(*node)->right, insertNode, level + 1);
		}
		else if (level % 3 == 1)
		{
			if (insertNode->point.y < (*node)->point.y)
				insert(&(*node)->left, insertNode, level + 1);
			else
				insert(&(*node)->right, insertNode, level + 1);
		}
		else // if (level % 3 == 2)
		{
			if (insertNode->point.z < (*node)->point.z)
				insert(&(*node)->left, insertNode, level + 1);
			else
				insert(&(*node)->right, insertNode, level + 1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, target, ids, distanceTol, 0);
		return ids;
	}

	void search(Node<PointT> *node, PointT target, std::vector<int> &ids, float distanceTol, int level)
	{
		if (node == NULL)
            return;

        if ((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) &&
            (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)) &&
			(node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol)))
        {
            float distance = std::sqrt(std::pow(node->point.x - target.x, 2) + std::pow(node->point.y - target.y, 2) + std::pow(node->point.z - target.z, 2));
            if (distance <= distanceTol)
                ids.push_back(node->id);
        }

        // Traverse left or right according to the current dimension (X or Y)
        if (level % 3== 0)
        {
            if ((target.x - distanceTol) < node->point.x)
                search(node->left, target, ids, distanceTol, level+1);
            if ((target.x + distanceTol) > node->point.x)
                search(node->right, target, ids, distanceTol, level+1);
        }
        else if (level % 3== 1)
        {
            if ((target.y - distanceTol) < node->point.y)
                search(node->left, target, ids, distanceTol, level+1);
            if ((target.y + distanceTol) > node->point.y)
                search(node->right, target, ids, distanceTol, level+1);
        }
		else // if (level % 3== 2)
        {
            if ((target.z - distanceTol) < node->point.z)
                search(node->left, target, ids, distanceTol, level+1);
            if ((target.z + distanceTol) > node->point.z)
                search(node->right, target, ids, distanceTol, level+1);
        }
    }
};




