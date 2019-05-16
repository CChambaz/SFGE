#include "..\include\p2quadtree.h"

p2QuadTree::p2QuadTree()
{
	
};

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	// Set base values
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
}

p2QuadTree::~p2QuadTree()
{
}

void p2QuadTree::Clear()
{
	// Clear the current quadtree object vector
	m_Objects.clear();

	// Goes through all his child quadtree
	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		// Check if the child quadtree exist
		if(m_Nodes[i] != nullptr)
		{ 
			// Clear the child quadtree
			m_Nodes[i]->Clear();

			// Delete the child quadtree
			m_Nodes[i] = nullptr;
		}
	}
}

void p2QuadTree::Split()
{
	// Define the corners of the current node
	const p2Vec2 extends = m_Bounds.GetExtends();

	// Set the current position
	p2Vec2 currentPosition = m_Bounds.m_BottomLeft;

	// Define the size of the child sides depending on the amount of child tree number
	const float childSideSize = (m_Bounds.m_TopRight.y - currentPosition.y) / sqrt(CHILD_TREE_NMB);

	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		p2AABB childAABB;

		childAABB.m_BottomLeft = currentPosition;

		childAABB.m_TopRight = { currentPosition.x + childSideSize, currentPosition.y + childSideSize };

		// Check if it needs to jump on the y axis
		if (currentPosition.x + childSideSize >= extends.x)
			currentPosition = { m_Bounds.m_BottomLeft.x, currentPosition.y + childSideSize };
		else
			currentPosition.x = currentPosition.x + childSideSize;

		// Add the node to the child array
		m_Nodes[i] = new p2QuadTree(m_NodeLevel + 1, childAABB);
	}
}

int p2QuadTree::GetIndex(p2Body * rect)
{
	// Get the center of the current quadtree
	const p2Vec2 quadCenter = m_Bounds.GetCenter();

	// Get the maximum and minimum position of the rect
	const p2Vec2 rectMin = rect->GetMinPosition();
	const p2Vec2 rectMax = rect->GetMaxPosition();

	// Define if the body is on the left
	const bool onLeft = rectMax.x < quadCenter.x;
	const bool onRight = rectMin.x > quadCenter.x;

	// Check if completely on the bottom part of the quadtree
	if (rectMax.y < quadCenter.y)
	{
		if (onLeft)
			return 0;

		if (onRight)
			return 1;
	}
	// Check if completely on the top part of the quadtree
	else if (rectMin.y > quadCenter.y)
	{
		if (onLeft)
			return 2;

		if (onRight)
			return 3;
	}

	// Return -1 if the body is not perfectly inside one of the child quadtree
	return -1;
}

void p2QuadTree::Insert(p2Body * obj)
{
	// Check if the split has already be made
	if(m_Nodes[0] != nullptr)
	{
		// Get the index of the child quadtree where the body belongs to
		int bodyIndex = GetIndex(obj);

		// Check if the body fit perfectly in one of the child quadtree
		if(bodyIndex != -1)
		{
			// Add the body to the corresponding child quadtree
			m_Nodes[bodyIndex]->Insert(obj);

			return;
		}
	}

	// Add the body to the current quadtree
	m_Objects.push_back(obj);

	// Check if the quadtree has enough object to split and if it is not the last split level
	if(m_Objects.size() > MAX_OBJECTS && m_NodeLevel < MAX_LEVELS)
	{
		// Check if the quadtree has already been splited
		if (m_Nodes[0] == nullptr)
			Split();

		int i = 0;

		// Go through the object vector
		while(i < m_Objects.size())
		{
			// Get the index of the child quadtree where the body belongs to
			int bodyIndex = GetIndex(m_Objects[i]);

			// Check if the body fit perfectly in one of the child quadtree
			if (bodyIndex != -1)
			{
				// Add the body to the corresponding child quadtree
				m_Nodes[bodyIndex]->Insert(m_Objects[i]);

				// Remove the body from the parent
				m_Objects.erase(m_Objects.begin() + i);
			}
			else
			{
				// Move to the next element
				i++;
			}
		}
	}
}

std::vector<p2Body*> p2QuadTree::Retrieve(std::vector<p2Body*> returnedBodies, p2Body* body)
{
	// Get the index of the child quadtree where the body is located
	int bodyIndex = GetIndex(body);

	// Check if the body fit perfectly in one of the child quadtree and if there is child quadtree
	if(bodyIndex != 0 && m_Nodes[0] != nullptr)
	{
		// Get the bodies from this child
		m_Nodes[bodyIndex]->Retrieve(returnedBodies, body);
	}

	// Add the bodies of this quadtree
	returnedBodies.insert(returnedBodies.end(), m_Objects.begin(), m_Objects.end());

	return returnedBodies;
}
