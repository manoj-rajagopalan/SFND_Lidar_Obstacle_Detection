/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


constexpr inline float sqr(const float x) {
	return (x*x);
}

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

class KdTree
{
public:
	Node* root = nullptr;

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Manoj: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		assert(point.size() == 2);
		auto new_node = new Node(point, id);
		if (root) {
			addToTree(new_node, root, 0);
		} else {
			root = new_node;
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(target, distanceTol, root, 0, ids);
		return ids;
	}
	
  private:

	void addToTree(Node *new_node, Node *tree_node, int idx) {
		assert(tree_node);
		if(new_node->point[idx] <= tree_node->point[idx]) {
			if(tree_node->left) {
				addToTree(new_node, tree_node->left, 1-idx);
			} else {
				tree_node->left = new_node;
			}
		} else {
			if(tree_node->right) {
				addToTree(new_node, tree_node->right, 1-idx);
			} else {
				tree_node->right = new_node;
			}
		}
	}

	void search(std::vector<float> const& point,
	            const float distanceTol,
				Node const *const node,
				int idx,
				std::vector<int>& ids)
	{
		if(inBox(point, node->point, distanceTol)) {
			const float d_sqr = sqr(point[0] - node->point[0]) + sqr(point[1] - node->point[1]);
			const float tol_sqr = sqr(distanceTol);
			if(d_sqr <= tol_sqr) {
				ids.push_back(node->id);
			}
		}

		if(node->left && (point[idx] - distanceTol) <= node->point[idx]) {
			search(point, distanceTol, node->left, 1-idx, ids);
		}

		if(node->right && (point[idx] + distanceTol >= node->point[idx])) {
			search(point, distanceTol, node->right, 1-idx, ids);
		}
	}

	bool inBox(const std::vector<float>& target,
	           const std::vector<float>& node_pt,
						 const float distanceTol)
	{
		return ((target[0] - distanceTol) <= node_pt[0]) &&
		       ((target[0] + distanceTol) >= node_pt[0]) &&
			   ((target[1] - distanceTol) <= node_pt[1]) &&
		       ((target[1] + distanceTol) >= node_pt[1]);
	}
};




