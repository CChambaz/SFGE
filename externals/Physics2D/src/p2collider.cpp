#include "..\include\p2collider.h"

p2Collider::p2Collider(p2ColliderDef colDef)
{
	m_UserData = colDef.userData;
	m_ColliderDefinition = colDef;
}

p2Collider::p2Collider()
{
	
}


bool p2Collider::IsSensor() const
{
	return m_ColliderDefinition.isSensor;
}

sfge::ColliderData * p2Collider::GetUserData() const
{
	return m_UserData;
}

p2Shape* p2Collider::GetShape() const
{
	return m_ColliderDefinition.shape;
}

float p2Collider::GetRestitution() const
{
	return m_ColliderDefinition.restitution;
}

void p2Collider::SetUserData(sfge::ColliderData* colliderData)
{
	m_UserData = colliderData;
}
