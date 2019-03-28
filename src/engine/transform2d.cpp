/*
 * transform.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: efarhan
 */


#include <engine/transform2d.h>
#include <imgui.h>
#include <engine/engine.h>
namespace sfge
{
void editor::Transform2dInfo::DrawOnInspector()
{
	auto* transform = transformManager->GetComponentPtr(m_Entity);
	float pos[2] = { transform->Position.x, transform->Position.y };
	ImGui::Separator();
	ImGui::Text("Transform");
	ImGui::InputFloat2("Position", pos);
	float scale[2] = { transform->Scale.x, transform->Scale.y };
	ImGui::InputFloat2("Scale", scale);
	ImGui::InputFloat("Angle", &transform->EulerAngle);
}


Transform2d* Transform2dManager::AddComponent(Entity entity)
{

	auto& transform = GetComponentRef(entity);
	m_Engine.GetEntityManager()->AddComponentType(entity, ComponentType::TRANSFORM2D);
	auto& transformInfo = GetComponentInfo(entity);
	transformInfo.SetEntity(entity);
	transformInfo.transformManager = this;
	return &transform;
}

void Transform2dManager::CreateComponent(json& componentJson, Entity entity)
{

	//Log::GetInstance()->Msg("Create component Transform");
	auto* transform = AddComponent(entity);
	if (CheckJsonExists(componentJson, "position"))
		transform->Position = GetVectorFromJson(componentJson, "position");
	if (CheckJsonExists(componentJson, "scale"))
		transform->Scale = GetVectorFromJson(componentJson, "scale");
	if (CheckJsonExists(componentJson, "angle") && CheckJsonNumber(componentJson, "angle"))
		transform->EulerAngle = componentJson["angle"];
}

void Transform2dManager::DestroyComponent(Entity entity)
{
	m_Engine.GetEntityManager()->RemoveComponentType(entity, ComponentType::TRANSFORM2D);
}


void Transform2dManager::OnUpdate(float dt) {
	System::OnUpdate(dt);
    for(auto& transform : m_Components)
	{
    	if(transform.EulerAngle > 180.0f)
		{
    		transform.EulerAngle -= 360.0f;
		}

		if(transform.EulerAngle < -180.0f)
		{
			transform.EulerAngle += 360.0f;
		}
	}
}

}
