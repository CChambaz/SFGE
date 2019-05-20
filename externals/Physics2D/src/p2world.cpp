/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <p2world.h>
#include <iostream>


p2World::p2World(p2Vec2 gravity, p2Vec2 screenResolution)
{
	m_Gravity = gravity;
	m_ScreenResolution = screenResolution;

	m_ParentQuad = p2QuadTree(0, p2AABB({ 0.0f, screenResolution.y }, { screenResolution.x, 0.0f }));
	m_ContactManager = p2ContactManager();

	m_Bodies.resize(MAX_BODY_LEN);
}

void p2World::Step(float dt)
{
	// TODO: Review the forces calculation and application
	for (int i = 0; i < m_BodyIndex; i++)
	{
		// Add the current body to the quadtree
		m_ParentQuad.Insert(&m_Bodies[i]);

		if (m_Bodies[i].GetType() == p2BodyType::STATIC)
			continue;
		//*************************************** Calculate forces ***************************************//
		
		//************************************************************************************************//

		// Apply acceleration
		//body.SetLinearVelocity(body.GetLinearVelocity() + m_Gravity * dt);

		// Apply movement
		// TODO: Actually apply an acceleration with the gravity
		p2Vec2 newPos = m_Bodies[i].GetPosition() + (m_Bodies[i].GetLinearVelocity() + m_Gravity) * dt;
		m_Bodies[i].SetPosition(newPos);
	}

	// Vector containing all bodies that could collide with the current body
	std::vector<p2Body*> returnedBodies;

	// Check for collision
	for(int i = 0; i < m_BodyIndex; i++)
	{
		returnedBodies.clear();

		// Get the collider of the current body
		p2Collider* currentCollider = m_Bodies[i].GetCollider();

		// Define the shape of the current body
		ShapeType* currentType = &currentCollider->GetShape()->m_Type;

		// Get the center of the current AABB
		const p2Vec2 currentAABBCenter = m_Bodies[i].GetPosition();		

		// Get the bottom left and top right point of the current AABB
		const p2Vec2 currentAABBTopRight = m_Bodies[i].GetMaxPosition();
		const p2Vec2 currentAABBBottomLeft = m_Bodies[i].GetMinPosition();		

		// Get the extends of the current body's AABB
		const p2Vec2 currentAABBExtends = m_Bodies[i].GetAABBExtends();

		// Define the top left and bottom right point of the current AABB
		const p2Vec2 currentAABBBottomRight = { currentAABBTopRight.x, currentAABBTopRight.y - currentAABBExtends.y };
		const p2Vec2 currentAABBTopLeft = { currentAABBBottomLeft.x, currentAABBBottomLeft.y + currentAABBExtends.y };

		// Define the radius of the circle surrounding the current AABB
		const float currentAABBRadius = (currentAABBTopRight - currentAABBCenter).GetMagnitude();

		// Get the bodies that could collide with the current body
		returnedBodies = m_ParentQuad.Retrieve(returnedBodies, &m_Bodies[i]);
		std::cout << "Retrieved bodies : " << returnedBodies.size() << "\n";
		// Go through the retrieved bodies and check for collision
		for(int j = 0; j < returnedBodies.size(); j++)
		{
			// Get the collider of the checked body
			p2Collider* checkedCollider = returnedBodies[i]->GetCollider();

			// Define the shape of the checked body
			ShapeType* checkedType = &checkedCollider->GetShape()->m_Type;

			// Get the position and top right point of the checked body
			const p2Vec2 checkedAABBCenter = returnedBodies[j]->GetPosition();
			const p2Vec2 checkedAABBTopRight = returnedBodies[j]->GetMaxPosition();

			// Define the radius of the circle surrounding the checked AABB
			const float checkedAABBRadius = (checkedAABBTopRight - checkedAABBCenter).GetMagnitude();

			// Define the distance between the current to the checked AABB
			const float distanceBetweenAABB = (checkedAABBCenter - currentAABBCenter).GetMagnitude();

			// Check if the sum of the two radius is greater than the distance between the two bodies
			if (distanceBetweenAABB <= currentAABBRadius + checkedAABBRadius)
			{	
				bool collisionResult = true;

				//TODO: SAT
				// Define the type of shape colliding
				if(*currentType == ShapeType::CIRCLE && *checkedType == ShapeType::CIRCLE)
				{
					// Circle v Circle collision

					// Point of collision : 
				}
				else if((*currentType == ShapeType::CIRCLE || *checkedType == ShapeType::CIRCLE) && 
						(*currentType == ShapeType::RECT || *checkedType == ShapeType::RECT))
				{
					// Circle v Rect collision
				}
				else if(*currentType == ShapeType::RECT && *checkedType == ShapeType::RECT)
				{
					// Rect v Rect collision
				}
				
				// Get the bottom left point of the checked body
				/*const p2Vec2 currentAABBBottomLeft = m_Bodies[i].GetMinPosition();

				// Get the extends of the checked body's AABB
				const p2Vec2 currentAABBExtends = m_Bodies[i].GetAABBExtends();

				// Define the top left and bottom right point of the checked AABB
				const p2Vec2 currentAABBBottomRight = { currentAABBTopRight.x, currentAABBTopRight.y - currentAABBExtends.y };
				const p2Vec2 currentAABBTopLeft = { currentAABBBottomLeft.x, currentAABBBottomLeft.y + currentAABBExtends.y };*/

				// SAT success
				if(collisionResult)
				{
					// Create the contact
					p2Contact* contact = m_ContactManager.CreateContact(currentCollider, checkedCollider);

					// Apply the contact if the contact is a new one
					if (contact != nullptr)
					{
						m_ContactListener->BeginContact(contact);

						// TODO: Apply the collision forces
					}					

					// Move to the next body
					continue;
				}
			}

			// Try to get a contact between the two actual bodies
			int contactID = m_ContactManager.GetContactID(currentCollider, checkedCollider);

			// If the bodies where in contact before, end it and destroy it
			if (contactID != -1)
			{
				m_ContactListener->EndContact(m_ContactManager.GetContactByID(contactID));
				m_ContactManager.DestroyContact(contactID);
			}
		}
	}	

	// Reset the quatree
	m_ParentQuad.Clear();
}

p2Body * p2World::CreateBody(p2BodyDef* bodyDef)
{
	p2Body& body = m_Bodies[m_BodyIndex];
	body.Init(bodyDef);
	m_BodyIndex++;
	return &body;
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
	m_ContactListener = contactListener;
}
