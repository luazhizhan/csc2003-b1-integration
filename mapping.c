#define MINWALLLENGTH 10
#define NODELENGTH 25
#define NUMOFNODESTODECLARE 30 // NUM OF POINTERS FOR THE NODE LIST EXTRA IS DECLARED AS SAFETY
uint64_t frontScannedLength = 0;
uint64_t rightScannedLength = 0;
uint64_t leftScannedLength = 0;
int direction = 0;		 // 0 = n, 1=w , 2=s, 3=e direction the car is currently facing
int currentUsedNode = 0; // counter for next node number to use
bool mappingCompleted = false;
int targetx, targety; // target x and y for Astar
/*f=g+h */
struct Node
{
	int x, y;		   // coordinates of this node
	int status;		   // 0 = unvisited 1 = visited
	struct Node *left; // node neighbours
	struct Node *right;
	struct Node *front;
	struct Node *back;
	struct Node *parent;
	int f; // a* algo f = total cost of node
	int g; // g = dist btwn current node and start node
	int h; // h = heuristic -> est. dist from goal
};

struct Node *createnode()
{
	struct Node *temp;
	temp = (struct Node *)malloc(sizeof(struct Node));
	temp->left = NULL;
	temp->right = NULL;
	temp->front = NULL;
	temp->back = NULL;
	temp->parent = NULL;
	temp->f = 0;
	temp->g = 0;
	temp->h = 0;
	temp->status = 0;
	temp->x = 0;
	temp->y = 0;
	return temp;
}

struct List // LinkedList implementation
{
	struct List *next;
	struct Node *child;
};

struct List *createlistnode()
{
	struct List *temp;
	temp = (struct List *)malloc(sizeof(struct List));
	temp->next = NULL;
	return temp;
}

struct List *addList(struct List *head, struct Node *node) // adds a node to the end of linked list
{
	struct List *temp, *cur;
	temp = createlistnode();
	temp->child = node;
	if (head == NULL) // error prevention
	{
		head = temp;
	}
	else
	{

		cur = head;
		while (cur->next != NULL)
		{
			cur = cur->next;
		}
		cur->next = temp;

		return head;
	}
}

struct List *removeList(struct List *head, struct Node *remove) // remove a node if it exists in a linked list
{
	struct List *temp;
	temp = head;
	if (temp->child == remove)
	{
		head = temp->next;
		// free(temp);
		return head;
	}
	while (temp->next != NULL)
	{
		if (temp->next->child == remove)
		{
			temp->next = temp->next->next;
			return head;
		}
		else
		{
			temp = temp->next;
		}
	}
}

struct List *addFrontList(struct List *head, struct Node *node) // pushes node to front of list
{
	struct List *temp, *cur;
	temp = createlistnode();
	temp->child = node;
	if (head == NULL) // error prevention
	{
		head = temp;
	}
	else
	{
		temp->next = head;
		head = temp;
		return head;
	}
}

struct List *removeFrontList(struct List *head) // Pop front of list
{
	struct List *temp;
	temp = head;
	head = temp->next;
	// free(temp);
	return head;
}

bool existInList(struct List *head, struct Node *find) // checks if a node exists in a list
{
	struct List *temp;
	temp = head;
	while (temp != NULL)
	{
		if (temp->child == find)
		{
			return true;
		}
		else
		{
			temp = temp->next;
		}
	}
	return false;
}

struct List *freeList(struct List *head) // frees an entire list
{
	struct List *temp;
	struct List *temp2;
	temp = head;
	temp2 = head;
	while (temp != NULL)
	{
		// free(temp);
		temp = temp2->next;
		temp2 = temp2->next;
	}
}

struct Node *NodeList[NUMOFNODESTODECLARE]; // declare pointers for the map to use
struct Node *currNode;						// current node the car is at
struct Node *pathfindingNode;				// current node being checked by A*
struct Node *goalNode;						// end goal for A*

struct List *closedList; // nodes that we have "checked"
struct List *openList;	 // neighbouring nodes that have yet to be "checked"
struct List *pathList;	 // path for the car to follow to the found node

uint64_t ScanForward()
{
	return scanForward();
}
uint64_t ScanLeft()
{
	return scanLeft();
}
uint64_t ScanRight()
{
	return scanRight();
}

void turnLeftMove()
{
	uint64_t currScanLength = 0;
	turn_left(90);
	frontScannedLength = ScanForward();
	currScanLength = frontScannedLength;
	move_forward();
	while ((frontScannedLength - currScanLength) < NODELENGTH)
	{
		currScanLength = ScanForward();
	}
	stop_movement();
}
void forwardMove()
{
	float currScanLength = 0;
	frontScannedLength = ScanForward();
	currScanLength = frontScannedLength;
	move_forward();
	while ((frontScannedLength - currScanLength) < NODELENGTH)
	{
		currScanLength = ScanForward();
	}
	stop_movement();
}
void turnRightMove()
{
	float currScanLength = 0;
	turn_right(90);
	frontScannedLength = ScanForward();
	currScanLength = frontScannedLength;
	move_forward();
	while ((frontScannedLength - currScanLength) < NODELENGTH)
	{
		currScanLength = ScanForward();
	}
	stop_movement();
}
void turnBehindMove()
{
	float currScanLength = 0;
	turn_left(90);
	turn_left(90);
	frontScannedLength = ScanForward();
	currScanLength = frontScannedLength;
	move_forward();
	while ((frontScannedLength - currScanLength) < NODELENGTH)
	{
		currScanLength = ScanForward();
	}
	stop_movement();
}

void moveLeft()
{
	if (direction == 0)
	{
		currNode = currNode->left;
		turnLeftMove(); // pid turn car left and move forward [true north] to east
	}
	if (direction == 1)
	{
		currNode = currNode->left;
		forwardMove(); // call pid to move car forward [true east]
	}
	if (direction == 2)
	{
		currNode = currNode->left;
		turnRightMove(); // turn car right and move forward [true south] to east
	}
	if (direction == 3)
	{
		currNode = currNode->left;
		turnBehindMove(); // turn pid 180 to move car [true west] to east
	}
	direction = 1;
}

void moveRight()
{
	if (direction == 0)
	{
		currNode = currNode->right;
		turnRightMove(); // pid turn car right and move forward [true north] to west
	}
	if (direction == 1)
	{
		currNode = currNode->right;
		turnBehindMove(); // pid turn car 180 and move forward [true east] to west
	}
	if (direction == 2)
	{
		currNode = currNode->right;
		turnLeftMove(); // pid turn car left and move forward [true south] to west
	}
	if (direction == 3)
	{
		currNode = currNode->right;
		forwardMove(); // call pid to move car forward [true west]
	}
	direction = 3;
}
void moveBack()
{
	if (direction == 0)
	{
		currNode = currNode->back;
		turnBehindMove(); // pid turn car 180 and move forward [true north] to south
	}
	if (direction == 1)
	{
		currNode = currNode->back;
		turnLeftMove(); // pid turn car left and move forward [true east] to south
	}
	if (direction == 2)
	{
		currNode = currNode->back;
		forwardMove(); // pid car move forward [true south]
	}
	if (direction == 3)
	{
		currNode = currNode->back;
		turnRightMove(); // call pid to move car forward [true west] to south
	}
	direction = 2;
}
void moveFront()
{
	if (direction == 0)
	{
		currNode = currNode->front;
		forwardMove(); // pid car move forward [true north] to north
	}
	if (direction == 1)
	{
		currNode = currNode->front;
		turnRightMove(); // pid turn car right and move forward [true west] to north
	}
	if (direction == 2)
	{
		currNode = currNode->front;
		turnBehindMove(); // pid turn car 180 and ove forward [true south] to north
	}
	if (direction == 3)
	{
		currNode = currNode->front;
		turnLeftMove(); // call pid to move car forward [true east] to north
	}
	direction = 0;
}

bool NodeInNodeList(int x, int y)
{
	for (int i; i < NUMOFNODESTODECLARE; i++)
	{
		if (NodeList[i] != NULL)
		{
			if (NodeList[i]->x == x && NodeList[i]->y == y)
			{
				return true;
			}
		}
	}
	return false;
}

// astar and move function
void Move()
{
	pathfindingNode = currNode;
	freeList(closedList);
	freeList(openList);
	closedList = NULL;
	openList = NULL;
	closedList = addList(closedList, currNode);
	if (currNode->front != NULL)
	{
		openList = addList(openList, currNode->front);
		currNode->front->g = 1;
		currNode->front->h = abs(currNode->front->x - targetx) + abs(currNode->front->y - targety);
		currNode->front->f = currNode->front->g + currNode->front->h;
		currNode->front->parent = currNode;
	}
	if (currNode->back != NULL)
	{
		openList = addList(openList, currNode->back);
		currNode->back->g = 1;
		currNode->back->h = abs(currNode->back->x - targetx) + abs(currNode->back->y - targety);
		currNode->back->f = currNode->back->g + currNode->back->h;
		currNode->back->parent = currNode;
	}
	if (currNode->left != NULL)
	{
		openList = addList(openList, currNode->left);
		currNode->left->g = 1;
		currNode->left->h = abs(currNode->left->x - targetx) + abs(currNode->left->y - targety);
		currNode->left->f = currNode->left->g + currNode->left->h;
		currNode->left->parent = currNode;
	}
	if (currNode->right != NULL)
	{
		openList = addList(openList, currNode->right);
		currNode->right->g = 1;
		currNode->right->h = abs(currNode->right->x - targetx) + abs(currNode->right->y - targety);
		currNode->right->f = currNode->right->g + currNode->right->h;
		currNode->right->parent = currNode;
	}
	struct List *temp;
	while (openList != NULL)
	{
		pathfindingNode = openList->child;
		temp = openList;
		while (temp != NULL) // find lowest f value
		{
			if (temp->child->f < pathfindingNode->f)
			{
				pathfindingNode = temp->child;
			}
			temp = temp->next;
		}

		openList = removeList(openList, pathfindingNode); // add current selected lowest f to closed list
		closedList = addList(closedList, pathfindingNode);

		temp = closedList;
		while (temp != NULL)
		{
			if (temp->child == goalNode)
			{
				// goalNode->parent = pathfindingNode;

				pathList = addFrontList(pathList, goalNode);
				pathList = addFrontList(pathList, pathfindingNode);
				while (pathfindingNode->parent != currNode)
				{
					pathList = addFrontList(pathList, pathfindingNode->parent);
					pathfindingNode = pathfindingNode->parent;
				}

				while (currNode != goalNode)
				{
					if (currNode->front == pathList->child)
					{
						moveFront();
						pathList = pathList->next;
					}
					else if (currNode->left == pathList->child)
					{
						moveLeft();
						pathList = pathList->next;
					}
					else if (currNode->right == pathList->child)
					{
						moveRight();
						pathList = pathList->next;
					}
					else if (currNode->back == pathList->child)
					{
						moveBack();
						pathList = pathList->next;
					}
					else
					{
						printf("im sorry idk how you ended up here the car is hentak kaki from this point onwards");
					}
				}
				goalNode->status = 1;
				printf("\nSuccessful navigated to Node \n");
				return;
			}
			temp = temp->next;
		}
		// adds neighbouring nodes if they exist to the openlist & calculate f=g+h
		if (pathfindingNode->front != NULL)
		{
			if (!existInList(closedList, pathfindingNode->front) && !existInList(openList, pathfindingNode->front))
			{
				openList = addList(openList, pathfindingNode->front);
				pathfindingNode->front->parent = pathfindingNode;
				pathfindingNode->front->g = pathfindingNode->g + 1;
				pathfindingNode->front->h = abs(pathfindingNode->front->x - targetx) + abs(pathfindingNode->front->y - targety);
				pathfindingNode->front->f = pathfindingNode->front->h + pathfindingNode->front->g;
			}
			else
			{
				if (pathfindingNode->front->g > pathfindingNode->g + 1)
				{
					pathfindingNode->front->parent = pathfindingNode;
					pathfindingNode->front->g = pathfindingNode->g + 1;
					pathfindingNode->front->f = pathfindingNode->front->h + pathfindingNode->front->g;
				}
			}
		}
		if (pathfindingNode->back != NULL)
		{
			if (!existInList(closedList, pathfindingNode->back) && !existInList(openList, pathfindingNode->back))
			{
				openList = addList(openList, pathfindingNode->back);
				pathfindingNode->back->parent = pathfindingNode;
				pathfindingNode->back->g = pathfindingNode->g + 1;
				pathfindingNode->back->h = abs(pathfindingNode->back->x - targetx) + abs(pathfindingNode->back->y - targety);
				pathfindingNode->back->f = pathfindingNode->back->h + pathfindingNode->back->g;
			}
			else
			{
				if (pathfindingNode->back->g > pathfindingNode->g + 1)
				{
					pathfindingNode->back->parent = pathfindingNode;
					pathfindingNode->back->g = pathfindingNode->g + 1;
					pathfindingNode->back->f = pathfindingNode->back->h + pathfindingNode->back->g;
				}
			}
		}
		if (pathfindingNode->left != NULL)
		{
			if (!existInList(closedList, pathfindingNode->left) && !existInList(openList, pathfindingNode->left))
			{
				openList = addList(openList, pathfindingNode->left);
				pathfindingNode->left->parent = pathfindingNode;
				pathfindingNode->left->g = pathfindingNode->g + 1;
				pathfindingNode->left->h = abs(pathfindingNode->left->x - targetx) + abs(pathfindingNode->left->y - targety);
				pathfindingNode->left->f = pathfindingNode->left->h + pathfindingNode->left->g;
			}
			else
			{
				if (pathfindingNode->left->g > pathfindingNode->g + 1)
				{
					pathfindingNode->left->parent = pathfindingNode;
					pathfindingNode->left->g = pathfindingNode->g + 1;
					pathfindingNode->left->f = pathfindingNode->left->h + pathfindingNode->left->g;
				}
			}
		}
		if (pathfindingNode->right != NULL)
		{
			if (!existInList(closedList, pathfindingNode->right) && !existInList(openList, pathfindingNode->right))
			{
				openList = addList(openList, pathfindingNode->right);
				pathfindingNode->right->parent = pathfindingNode;
				pathfindingNode->right->g = pathfindingNode->g + 1;
				pathfindingNode->right->h = abs(pathfindingNode->right->x - targetx) + abs(pathfindingNode->right->y - targety);
				pathfindingNode->right->f = pathfindingNode->right->h + pathfindingNode->right->g;
			}
			else
			{
				if (pathfindingNode->right->g > pathfindingNode->g + 1)
				{
					pathfindingNode->right->parent = pathfindingNode;
					pathfindingNode->right->g = pathfindingNode->g + 1;
					pathfindingNode->right->f = pathfindingNode->right->h + pathfindingNode->right->g;
				}
			}
		}
	}
}

void InitMapping() // initialize the map
{
	NodeList[0] = createnode();
	currNode = NodeList[0];
	currNode->status = 1;
	currNode->x = 0;
	currNode->y = 0;

	frontScannedLength = ScanForward();
	rightScannedLength = ScanRight();
	leftScannedLength = ScanLeft();
	currNode->back = NULL;
	if (frontScannedLength < NODELENGTH)
	{
		currNode->front = NULL;
	}
	else
	{
		if (!NodeInNodeList(0, 1))
		{
			currentUsedNode += 1;
			NodeList[currentUsedNode] = createnode();
			currNode->front = NodeList[currentUsedNode];
			currNode->front->back = currNode; // sets the new node to have the current node as its corresponding node neighbour
			currNode->front->status = 0;
		}
	}
	if (rightScannedLength < NODELENGTH)
	{
		currNode->right = NULL;
	}
	else
	{
		if (!NodeInNodeList(1, 0))
		{
			currentUsedNode += 1;
			NodeList[currentUsedNode] = createnode();
			currNode->right = NodeList[currentUsedNode];
			currNode->right->left = currNode; // sets the new node to have the current node as its corresponding node side
			currNode->right->status = 0;
		}
	}
	if (leftScannedLength < NODELENGTH)
	{
		currNode->left = NULL;
	}
	else
	{
		if (!NodeInNodeList(-1, 0))
		{
			currentUsedNode += 1;
			NodeList[currentUsedNode] = createnode();
			currNode->left = NodeList[currentUsedNode];
			currNode->left->right = currNode; // sets the new node to have the current node as its corresponding node side
			currNode->left->status = 0;
		}
	}
}

// finds any unvisited nodes
void FindNode()
{
	for (size_t i = 0; i < NUMOFNODESTODECLARE; i++)
	{
		if (NodeList[i] != NULL)
		{
			if (NodeList[i]->status == 0)
			{
				goalNode = NodeList[i];
				targetx = NodeList[i]->x;
				targety = NodeList[i]->y;
				return;
			}
		}
	}
	mappingCompleted = true;
}

// generate neighbouring nodes around car currently
void GenerateNeighbourNode()
{
	frontScannedLength = ScanForward();
	rightScannedLength = ScanRight();
	leftScannedLength = ScanLeft();

	if (frontScannedLength < MINWALLLENGTH) // check if there is enough space to create a node
	{
		if (direction == 0)
		{
			currNode->front = NULL;
		}
		else if (direction == 1)
		{
			currNode->left = NULL;
		}
		else if (direction == 2)
		{
			currNode->back = NULL;
		}
		else if (direction == 3)
		{
			currNode->right = NULL;
		}
	}
	else
	{
		if (direction == 0)
		{
			if (!NodeInNodeList(currNode->x, currNode->y + 1))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->front = NodeList[currentUsedNode];
				currNode->front->back = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->front->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x && NodeList[i]->y == currNode->y + 1)
						{
							NodeList[i]->back = currNode;
							currNode->front = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 1)
		{
			if (!NodeInNodeList(currNode->x - 1, currNode->y))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->left = NodeList[currentUsedNode];
				currNode->left->right = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->left->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x - 1 && NodeList[i]->y == currNode->y)
						{
							NodeList[i]->right = currNode;
							currNode->left = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 2)
		{
			if (!NodeInNodeList(currNode->x, currNode->y - 1))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->back = NodeList[currentUsedNode];
				currNode->back->front = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->back->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x && NodeList[i]->y == currNode->y - 1)
						{
							NodeList[i]->front = currNode;
							currNode->back = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 3)
		{
			if (!NodeInNodeList(currNode->x + 1, currNode->y))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->right = NodeList[currentUsedNode];
				currNode->right->left = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->right->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x + 1 && NodeList[i]->y == currNode->y)
						{
							NodeList[i]->left = currNode;
							currNode->right = NodeList[i];
						}
					}
				}
			}
		}
	}
	if (rightScannedLength < MINWALLLENGTH) // check if there is enough space to create a node
	{
		if (direction == 0)
		{
			currNode->right = NULL;
		}
		else if (direction == 1)
		{
			currNode->front = NULL;
		}
		else if (direction == 2)
		{
			currNode->left = NULL;
		}
		else if (direction == 3)
		{
			currNode->back = NULL;
		}
	}
	else
	{
		if (direction == 0)
		{
			if (!NodeInNodeList(currNode->x + 1, currNode->y))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->right = NodeList[currentUsedNode];
				currNode->right->left = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->right->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x + 1 && NodeList[i]->y == currNode->y)
						{
							NodeList[i]->left = currNode;
							currNode->right = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 1)
		{
			if (!NodeInNodeList(currNode->x, currNode->y + 1))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->front = NodeList[currentUsedNode];
				currNode->front->back = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->front->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x && NodeList[i]->y == currNode->y + 1)
						{
							NodeList[i]->back = currNode;
							currNode->front = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 2)
		{
			if (!NodeInNodeList(currNode->x - 1, currNode->y))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->left = NodeList[currentUsedNode];
				currNode->left->right = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->left->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x - 1 && NodeList[i]->y == currNode->y)
						{
							NodeList[i]->right = currNode;
							currNode->left = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 3)
		{
			if (!NodeInNodeList(currNode->x, currNode->y - 1))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->back = NodeList[currentUsedNode];
				currNode->back->front = currNode; // sets the new node to have the current node as its corresponding neighbour
				currNode->back->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x && NodeList[i]->y == currNode->y - 1)
						{
							NodeList[i]->front = currNode;
							currNode->back = NodeList[i];
						}
					}
				}
			}
		}
	}
	if (leftScannedLength < MINWALLLENGTH) // check if there is enough space to create a node
	{
		if (direction == 0)
		{
			currNode->left = NULL;
		}
		else if (direction == 1)
		{
			currNode->back = NULL;
		}
		else if (direction == 2)
		{
			currNode->right = NULL;
		}
		else if (direction == 3)
		{
			currNode->front = NULL;
		}
	}
	else
	{
		if (direction == 0)
		{
			if (!NodeInNodeList(currNode->x - 1, currNode->y))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->left = NodeList[currentUsedNode];
				currNode->left->right = currNode; // sets the new node to have the current node as its corresponding node side
				currNode->left->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x - 1 && NodeList[i]->y == currNode->y)
						{
							NodeList[i]->right = currNode;
							currNode->left = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 1)
		{
			if (!NodeInNodeList(currNode->x, currNode->y - 1))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->back = NodeList[currentUsedNode];
				currNode->back->front = currNode; // sets the new node to have the current node as its corresponding node side
				currNode->back->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x && NodeList[i]->y == currNode->y - 1)
						{
							NodeList[i]->front = currNode;
							currNode->back = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 2)
		{
			if (!NodeInNodeList(currNode->x + 1, currNode->y))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->right = NodeList[currentUsedNode];
				currNode->right->left = currNode; // sets the new node to have the current node as its corresponding node side
				currNode->right->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x + 1 && NodeList[i]->y == currNode->y)
						{
							NodeList[i]->left = currNode;
							currNode->right = NodeList[i];
						}
					}
				}
			}
		}
		else if (direction == 3)
		{
			if (!NodeInNodeList(currNode->x, currNode->y + 1))
			{
				currentUsedNode += 1;
				NodeList[currentUsedNode] = createnode();
				currNode->front = NodeList[currentUsedNode];
				currNode->front->back = currNode; // sets the new node to have the current node as its corresponding node side
				currNode->front->status = 0;
			}
			else
			{
				for (int i; i < NUMOFNODESTODECLARE; i++)
				{
					if (NodeList[i] != NULL)
					{
						if (NodeList[i]->x == currNode->x && NodeList[i]->y == currNode->y + 1)
						{
							NodeList[i]->back = currNode;
							currNode->front = NodeList[i];
						}
					}
				}
			}
		}
	}
}

// starts mapping sequence until there are no clear paths left
void Map()
{
	InitMapping();
	while (!mappingCompleted)
	{
		FindNode();
		if (mappingCompleted)
		{
			break;
		}
		Move();
		GenerateNeighbourNode();
	}
}

// goto the x and y node
void gotoNode(int zzx, int zzy)
{
	for (int i; i < NUMOFNODESTODECLARE; i++)
	{
		if (NodeList[i] != NULL)
		{
			if (NodeList[i]->x == zzx && NodeList[i]->y == zzy)
			{
				goalNode = NodeList[i];
				targetx = goalNode->x;
				targety = goalNode->y;
				Move();
				break;
			}
		}
	}
}
