//
// Created by efarhan on 09.08.18.
//

#include <engine/system.h>
#include <physics/collider2d.h>

namespace sfge
{
System::System(Engine& engine) : m_Engine(engine)
{
}


void System::OnEngineInit()
{
	m_Initialized = true;
}

void System::Destroy()
{
	OnBeforeSceneLoad();
	OnAfterSceneLoad();
	m_Initialized = false;
}


void System::OnContact(ColliderData* c1, ColliderData* c2, bool enter)
{
}

void System::SetEnable(bool enable)
{
	m_Enable = enable;
}

bool System::GetEnable() const
{
	return m_Enable;
}

Engine& System::GetEngine() const
{
	return m_Engine;
}

bool System::GetInitlialized() const
{
	return m_Initialized;
}
}
