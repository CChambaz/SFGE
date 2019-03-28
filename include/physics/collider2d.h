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


#ifndef SFGE_COLLIDER_H
#define SFGE_COLLIDER_H

#include <engine/entity.h>
#include <Box2D/Box2D.h>
#include <physics/body2d.h>


namespace sfge
{

enum class ColliderType
{
	NONE,
	CIRCLE,
	BOX,
	POLYGON
};

struct ColliderData
{
	Entity entity = INVALID_ENTITY;
	b2Fixture* fixture = nullptr;
	b2Body* body = nullptr;
};
namespace editor
{
struct ColliderInfo : ComponentInfo
{
    ColliderData* data;
	void DrawOnInspector() override;
};
}
class ColliderManager : public MultipleComponentManager<ColliderData, editor::ColliderInfo, ComponentType::COLLIDER2D>
{
public:
	using MultipleComponentManager::MultipleComponentManager;
	void OnEngineInit() override;
	ColliderData* AddComponent(Entity entity) override;
	void CreateComponent(json& componentJson, Entity entity)override;
	void DestroyComponent(Entity entity) override;
  	ColliderData* GetComponentPtr(Entity entity) override;
protected:

  	int GetFreeComponentIndex() override;
	Body2dManager* m_BodyManager = nullptr;
};

}

#endif