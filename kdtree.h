/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "render/render.h"



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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{

		//Tree is empty
		if(*node == NULL)
		*node = new Node(point, id);
		else
		{
			//Looking at current data

			uint cd = depth % 3;//x,y,z axis
			if (point[cd] < (*node)->point[cd])// if the x value is less than new point x value
			insertHelper(&((*node)->left), depth+1, point, id);
			else 
			insertHelper(&((*node)->right), depth+1, point, id);// if the y value is less than new point x value

		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id); //address of root, depth initialized to 0, point value, point id (doing recursively)

	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)//note node pointer and ids adress ref
	{
		uint cd = depth % 3;//3 dimensions // %2 - for 2dimension

		if(node!=NULL)
		{
			float x_dist=fabs(-node->point[0]+target[0]);//get x distance of target point from node (abs value as it cloud be on either side)
			float y_dist=fabs(-node->point[1]+target[1]);//get y distance of target point from node (abs value as it cloud be on either side)

			if((x_dist <= distanceTol) && (y_dist <= distanceTol))//check if both x and y are in tolerance
			{
				float distance = sqrt((x_dist * x_dist) + (y_dist * y_dist)); //if yes calculate distance of the target point from the node point
				if (distance<=distanceTol)//if the calculated distance is < than the tolerance means its within the target point vicinity
				ids.push_back(node->id);//add that point id
			}

			//uint cd = depth % 3;//3 dimensions 
			if((target[cd]-distanceTol)<node->point[cd])//add the target point to the left 
			searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((target[cd]+distanceTol)>node->point[cd])//add the target point to the right 
			searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);// target point, root point, depth 0, distanceTol value, ids (doing recursively)
		return ids;
	}
	

};




