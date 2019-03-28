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

#ifndef SFGE_ENTITY_H
#define SFGE_ENTITY_H

#include <vector>
#include <set>

#include <engine/system.h>
#include <editor/editor_info.h>
#include <engine/globals.h>

namespace sfge
{
enum class ComponentType : int;

class ResizeObserver
{
public:
	virtual void OnResize(size_t newSize) = 0;
};

class DestroyObserver
{
 public:
  virtual void OnDestroy(Entity entity) = 0;
};
/**
 * \brief Entity index number, starting from 1U
 */

using EntityMask = int;

namespace editor
{

struct EntityInfo : NamableEditorComponent, IDrawableInspector
{
void DrawOnInspector() override;	
};

}

class EntityManager : public System
{
public:
	using System::System;
	void OnEngineInit() override;

	void OnBeforeSceneLoad() override;

	EntityMask GetMask(Entity entity);
	Entity CreateEntity(Entity wantedEntity);
	void DestroyEntity(Entity entity);
	bool HasComponent(Entity entity, ComponentType componentType);
	void AddComponentType(Entity entity, ComponentType componentType);
	void RemoveComponentType(Entity entity, ComponentType componentType);
	editor::EntityInfo& GetEntityInfo(Entity entity);

	Entity GetEntityByName(std::string entityName) const;
	void ResizeEntityNmb(size_t newSize);
	void AddResizeObserver(ResizeObserver *resizeObserver);
	void AddDestroyObserver(DestroyObserver *destroyObserver);

	std::vector<Entity> GetEntitiesWithType(ComponentType componentType);

private:
	std::vector<EntityMask> m_MaskArray{ INIT_ENTITY_NMB };
	std::vector<editor::EntityInfo> m_EntityInfos{ INIT_ENTITY_NMB };
	std::set<ResizeObserver*> m_ResizeObservers;
	std::set<DestroyObserver*> m_DestroyObservers;
};
/*
template <>
void EntityManager::AddResizeObserver(SingleComponentManager<std::any, std::any>* componentManager)
{
	m_ResizeObsververs.push_back(componentManager);
}
*/

}

#endif 
