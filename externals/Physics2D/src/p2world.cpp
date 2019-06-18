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
	// Reset the quatree
	m_ParentQuad.Clear();

	// TODO: Review the forces calculation and application
	for (int i = 0; i < m_BodyIndex; i++)
	{
		// Add the current body to the quadtree
		m_ParentQuad.Insert(&m_Bodies[i]);

		if (m_Bodies[i].GetType() == p2BodyType::STATIC)
			continue;

		// Apply acceleration
		m_Bodies[i].SetLinearVelocity(m_Bodies[i].GetLinearVelocity() + (m_Gravity * dt));

		p2Vec2 newPos = m_Bodies[i].GetPosition() + m_Bodies[i].GetLinearVelocity() * dt;

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
		ShapeType currentType = currentCollider->GetShape()->m_Type;

		// Get the center of the current AABB
		const p2Vec2 currentBodyPosition = m_Bodies[i].GetPosition();		

		// Get the bottom left and top right point of the current AABB
		const p2Vec2 currentAABBTopRight = m_Bodies[i].GetMaxPosition();
		const p2Vec2 currentAABBBottomLeft = m_Bodies[i].GetMinPosition();		

		// Get the bodies that could collide with the current body
		returnedBodies = m_ParentQuad.Retrieve(returnedBodies, &m_Bodies[i]);
		
		// Go through the retrieved bodies and check for collision
		for(int j = 0; j < returnedBodies.size(); j++)
		{
			// Check if the current body is not the same as the checked one
			if (&m_Bodies[i] != returnedBodies[j])
			{
				// Get the collider of the checked body
				p2Collider* checkedCollider = returnedBodies[j]->GetCollider();

				// Define the shape of the checked body
				ShapeType checkedType = checkedCollider->GetShape()->m_Type;

				// Get the position and top right point of the checked body
				const p2Vec2 checkedBodyPosition = returnedBodies[j]->GetPosition();
				const p2Vec2 checkedAABBTopRight = returnedBodies[j]->GetMaxPosition();
				const p2Vec2 checkedAABBBottomLeft = returnedBodies[j]->GetMinPosition();

				bool collisionResult = false;

				if (currentAABBBottomLeft.x < checkedAABBTopRight.x &&
					currentAABBBottomLeft.y > checkedAABBTopRight.y &&
					currentAABBTopRight.x > checkedAABBBottomLeft.x &&
					currentAABBTopRight.y < checkedAABBBottomLeft.y)
				{
					// Define the type of shape colliding
					if (currentType == ShapeType::CIRCLE && checkedType == ShapeType::CIRCLE)
					{
						// Circle v Circle collision
						// Define the radius of the circle surrounding the current AABB
						p2CircleShape* currentCircle = static_cast<p2CircleShape*>(currentCollider->GetShape());
						p2CircleShape* checkedCircle = static_cast<p2CircleShape*>(checkedCollider->GetShape());

						const float currentRadius = currentCircle->GetRadius();

						// Define the radius of the circle surrounding the checked AABB
						const float checkedRadius = checkedCircle->GetRadius();

						// Define the distance between the current to the checked AABB
						const float distanceBetweenBodies = (checkedBodyPosition - currentBodyPosition).GetMagnitude();

						// Check if the sum of the two radius is greater than the distance between the two bodies
						if (distanceBetweenBodies <= currentRadius + checkedRadius + 0.1f)
						{
							collisionResult = true;

							// Create the contact
							p2Contact* contact = m_ContactManager.CreateContact(currentCollider, checkedCollider);

							// Apply the contact if the contact is a new one
							if (contact != nullptr)
							{
								m_ContactListener->BeginContact(*contact);

								if (!currentCollider->IsSensor() && !checkedCollider->IsSensor())
								{
									p2Vec2 vectorBetweenCenters = checkedBodyPosition - currentBodyPosition;

									// Get point of collision of the current body
									float currentCollisionAngle = atan(vectorBetweenCenters.y / vectorBetweenCenters.x);
									float currentComponentX = sin(currentCollisionAngle) * currentRadius;
									float currentComponentY = cos(currentCollisionAngle) * currentRadius;
									p2Vec2 collisionPoint = { currentComponentX, currentComponentY };

									// Reflection calculation
									p2Vec2 currentNormal = collisionPoint - currentBodyPosition;
									p2Vec2 newCurrentVelocity = (m_Bodies[i].GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(m_Bodies[i].GetLinearVelocity(), currentNormal.Normalized()))) * m_Bodies[i].GetRestitution();
									currentNormal = collisionPoint - checkedBodyPosition;

									p2Vec2 newCheckedVelocity = (returnedBodies[j]->GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(returnedBodies[j]->GetLinearVelocity(), currentNormal.Normalized()))) * returnedBodies[j]->GetRestitution(); //m_Bodies[i].GetLinearVelocity() * m_Bodies[i].GetRestitution() * -1;
									m_Bodies[i].SetLinearVelocity(newCurrentVelocity);
									returnedBodies[j]->SetLinearVelocity(newCheckedVelocity);
								}
							}
						}
					}
					// Rect v Circle collision
					else if (currentType == ShapeType::RECT && checkedType == ShapeType::CIRCLE)
					{						
						p2RectShape* currentRect = static_cast<p2RectShape*>(currentCollider->GetShape());
						p2CircleShape* checkedCircle = static_cast<p2CircleShape*>(checkedCollider->GetShape());

						if ((checkedBodyPosition.x < currentAABBTopRight.x && checkedBodyPosition.x > currentAABBBottomLeft.x) ||
							(checkedBodyPosition.y > currentAABBTopRight.y && checkedBodyPosition.y < currentAABBBottomLeft.y))
						{
							collisionResult = true;
						}
						else
						{
							p2Vec2 rectTopLeft = { currentAABBBottomLeft.x, currentAABBTopRight.y };
							p2Vec2 rectBottomRight = { currentAABBBottomLeft.y, currentAABBTopRight.x };

							// Circle on top
							if (checkedBodyPosition.y < currentAABBTopRight.y)
							{
								// Circle on left
								if (checkedBodyPosition.x < currentAABBBottomLeft.x)
								{
									float distance = (rectTopLeft - checkedBodyPosition).GetMagnitude();

									if (checkedCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
								else
								{
									float distance = (currentAABBTopRight - checkedBodyPosition).GetMagnitude();

									if (checkedCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
							}
							else
							{
								// Circle on left
								if (checkedBodyPosition.x < currentAABBBottomLeft.x)
								{
									float distance = (currentAABBBottomLeft - checkedBodyPosition).GetMagnitude();

									if (checkedCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
								else
								{
									float distance = (rectBottomRight - checkedBodyPosition).GetMagnitude();

									if (checkedCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
							}
						}

						if (collisionResult)
						{
							// Create the contact
							p2Contact* contact = m_ContactManager.CreateContact(currentCollider, checkedCollider);

							// Apply the contact if the contact is a new one
							if (contact != nullptr)
							{
								m_ContactListener->BeginContact(*contact);

								if (!currentCollider->IsSensor() && !checkedCollider->IsSensor())
								{
									p2Vec2 checkedSize = currentRect->m_Size;

									p2Vec2 vectorBetweenCenters = checkedBodyPosition - currentBodyPosition;

									float checkedRadius = checkedCircle->m_Radius;

									// Get point of collision of the current body
									float currentCollisionAngle = atan(vectorBetweenCenters.y / vectorBetweenCenters.x);
									float currentComponentX = sin(currentCollisionAngle) * checkedRadius;
									float currentComponentY = cos(currentCollisionAngle) * checkedRadius;
									p2Vec2 collisionPoint = { currentComponentX, currentComponentY };

									// Reflection calculation
									p2Vec2 currentNormal = collisionPoint - currentBodyPosition;
									p2Vec2 newCurrentVelocity = (m_Bodies[i].GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(m_Bodies[i].GetLinearVelocity(), currentNormal.Normalized()))) * m_Bodies[i].GetRestitution();
									currentNormal = collisionPoint - checkedBodyPosition;

									p2Vec2 newCheckedVelocity = (returnedBodies[j]->GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(returnedBodies[j]->GetLinearVelocity(), currentNormal.Normalized()))) * returnedBodies[j]->GetRestitution(); //m_Bodies[i].GetLinearVelocity() * m_Bodies[i].GetRestitution() * -1;
									m_Bodies[i].SetLinearVelocity(newCurrentVelocity);
									returnedBodies[j]->SetLinearVelocity(newCheckedVelocity);
								}
							}
						}
					}
					// Circle v Rect collision
					else if (currentType == ShapeType::CIRCLE && checkedType == ShapeType::RECT)
					{
						p2CircleShape* currentCircle = static_cast<p2CircleShape*>(currentCollider->GetShape());
						p2RectShape* checkedRect = static_cast<p2RectShape*>(checkedCollider->GetShape());

						if ((currentBodyPosition.x < checkedAABBTopRight.x && currentBodyPosition.x > checkedAABBBottomLeft.x) ||
							(currentBodyPosition.y > checkedAABBTopRight.y && currentBodyPosition.y < checkedAABBBottomLeft.y))
						{
							collisionResult = true;
						}
						else
						{
							p2Vec2 rectTopLeft = { checkedAABBBottomLeft.x, checkedAABBTopRight.y };
							p2Vec2 rectBottomRight = { checkedAABBBottomLeft.y, checkedAABBTopRight.x };

							// Circle on top
							if (currentBodyPosition.y < checkedAABBTopRight.y)
							{
								// Circle on left
								if (currentBodyPosition.x < checkedAABBBottomLeft.x)
								{
									float distance = (rectTopLeft - currentBodyPosition).GetMagnitude();

									if (currentCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
								else
								{
									float distance = (checkedAABBTopRight - currentBodyPosition).GetMagnitude();

									if (currentCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
							}
							else
							{
								// Circle on left
								if (currentBodyPosition.x < checkedAABBBottomLeft.x)
								{
									float distance = (checkedAABBBottomLeft - currentBodyPosition).GetMagnitude();

									if (currentCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
								else
								{
									float distance = (rectBottomRight - currentBodyPosition).GetMagnitude();

									if (currentCircle->m_Radius <= distance)
									{
										collisionResult = true;
									}
								}
							}
						}

						if (collisionResult)
						{
							// Create the contact
							p2Contact* contact = m_ContactManager.CreateContact(currentCollider, checkedCollider);

							// Apply the contact if the contact is a new one
							if (contact != nullptr)
							{
								m_ContactListener->BeginContact(*contact);

								if (!currentCollider->IsSensor() && !checkedCollider->IsSensor())
								{
									p2Vec2 checkedSize = checkedRect->m_Size;

									p2Vec2 vectorBetweenCenters = checkedBodyPosition - currentBodyPosition;

									float currentRadius = currentCircle->m_Radius;

									// Get point of collision of the current body
									float currentCollisionAngle = atan(vectorBetweenCenters.y / vectorBetweenCenters.x);
									float currentComponentX = sin(currentCollisionAngle) * currentRadius;
									float currentComponentY = cos(currentCollisionAngle) * currentRadius;
									p2Vec2 collisionPoint = { currentComponentX, currentComponentY };

									// Reflection calculation
									p2Vec2 currentNormal = collisionPoint - currentBodyPosition;
									currentNormal.NormalizeSelf();
									p2Vec2 newCurrentVelocity = (m_Bodies[i].GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(m_Bodies[i].GetLinearVelocity(), currentNormal.Normalized()))) * m_Bodies[i].GetRestitution();
									currentNormal = collisionPoint - checkedBodyPosition;
									currentNormal.NormalizeSelf();

									p2Vec2 newCheckedVelocity = (returnedBodies[j]->GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(returnedBodies[j]->GetLinearVelocity(), currentNormal.Normalized()))) * returnedBodies[j]->GetRestitution();
									m_Bodies[i].SetLinearVelocity(newCurrentVelocity);
									returnedBodies[j]->SetLinearVelocity(newCheckedVelocity);
								}
							}
						}
					}
					// Rect v Rect collision
					else if (currentType == ShapeType::RECT && checkedType == ShapeType::RECT)
					{					
						collisionResult = true;

						// Create the contact
						p2Contact* contact = m_ContactManager.CreateContact(currentCollider, checkedCollider);

						// Apply the contact if the contact is a new one
						if (contact != nullptr)
						{
							m_ContactListener->BeginContact(*contact);

							if (!currentCollider->IsSensor() && !checkedCollider->IsSensor())
							{
								p2RectShape* currentRect = static_cast<p2RectShape*>(currentCollider->GetShape());
								p2RectShape* checkedRect = static_cast<p2RectShape*>(checkedCollider->GetShape());

								p2Vec2 currentSize = currentRect->m_Size;
								p2Vec2 checkedSize = checkedRect->m_Size;

								p2Vec2 vectorBetweenCenters = checkedBodyPosition - currentBodyPosition;

								float currentDiagonal = (currentAABBTopRight - currentBodyPosition).GetMagnitude();

								// Get point of collision of the current body
								float currentCollisionAngle = atan(vectorBetweenCenters.y / vectorBetweenCenters.x);
								float currentComponentX = sin(currentCollisionAngle) * currentDiagonal;
								float currentComponentY = cos(currentCollisionAngle) * currentDiagonal;
								p2Vec2 collisionPoint = { currentComponentX, currentComponentY };

								// Reflection calculation
								p2Vec2 currentNormal = collisionPoint - currentBodyPosition;
								p2Vec2 newCurrentVelocity = (m_Bodies[i].GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(m_Bodies[i].GetLinearVelocity(), currentNormal.Normalized()))) * m_Bodies[i].GetRestitution();
								currentNormal = collisionPoint - checkedBodyPosition;

								p2Vec2 newCheckedVelocity = (returnedBodies[j]->GetLinearVelocity() - currentNormal.Normalized() * 2 * (p2Vec2::Dot(returnedBodies[j]->GetLinearVelocity(), currentNormal.Normalized()))) * returnedBodies[j]->GetRestitution();
								m_Bodies[i].SetLinearVelocity(newCurrentVelocity);
								returnedBodies[j]->SetLinearVelocity(newCheckedVelocity);
							}
						}
					}
				}
				else
				{
					// Try to get a contact between the two actual bodies
					int contactID = m_ContactManager.GetContactID(currentCollider, checkedCollider);

					// If the bodies where in contact before, end it and destroy it
					if (contactID != -1)
					{
						m_ContactListener->EndContact(*m_ContactManager.GetContactByID(contactID));
						m_ContactManager.DestroyContact(contactID);
					}
				}
			}
		}
	}	
}

p2Body * p2World::CreateBody(p2BodyDef* bodyDef)
{
	if (m_Bodies.size() <= m_BodyIndex)
		m_Bodies.resize(MAX_BODY_LEN + m_BodyIndex);

	p2Body& body = m_Bodies[m_BodyIndex];
	body.Init(bodyDef);
	m_BodyIndex++;
	return &body;
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
	m_ContactListener = contactListener;
}

std::vector<p2Body>* p2World::GetBodies()
{
	return &m_Bodies;
}

p2QuadTree* p2World::GetQuad()
{
	return &m_ParentQuad;
}