#include "..\include\p2collider.h"
#include <iostream>

p2Collider::p2Collider(p2ColliderDef colDef)
{
	m_ColliderDefinition = colDef;
	m_UserData = colDef.userData;
	m_Entity = colDef.entity;
	
	switch (colDef.shape.m_Type)
	{
		case ShapeType::CIRCLE:
		{
			p2CircleShape* circle = static_cast<p2CircleShape*>(&colDef.shape);
			m_Shape = p2CircleShape(circle->GetRadius());
			//circle->SetRadius(circleRadius);
			std::cout << "Circle radius : " << static_cast<p2CircleShape*>(&m_Shape)->GetRadius() << "\n";
		}
		break;
		case ShapeType::RECT:
		{
			p2RectShape* rect = static_cast<p2RectShape*>(&colDef.shape);
			m_Shape = p2RectShape(rect->GetSize());
			//circle->SetSize(rectSize);
		}
		default:
			break;
	}
	//m_Shape = colDef.shape;
	m_IsSensor = colDef.isSensor;
	m_Restitution = colDef.restitution;
}

p2Collider::p2Collider()
{
	
}


bool p2Collider::IsSensor() const
{
	return m_IsSensor;
}

void* p2Collider::GetUserData()
{
	return m_UserData;
}

Entity p2Collider::GetEntity()
{
	return m_Entity;
}

p2Shape* p2Collider::GetShape()
{
	return &m_Shape;
}

float p2Collider::GetRestitution() const
{
	return m_Restitution;
}

void p2Collider::SetUserData(sfge::ColliderData* colliderData)
{
	m_UserData = colliderData;
}

void p2Collider::SetEntity(Entity entity)
{
	m_Entity = entity;
}

