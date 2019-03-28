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

#ifndef SFGE_TRANSFORM_H_
#define SFGE_TRANSFORM_H_

#include <engine/entity.h>
#include <engine/component.h>
#include <engine/vector.h>

namespace sfge
{


struct Transform2d
{
	Vec2f Position;
	Vec2f Scale{1.0f,1.0f};
	float EulerAngle = 0.0f;
};

namespace editor
{
struct Transform2dInfo : ComponentInfo
{
	void DrawOnInspector() override;
	Transform2dManager* transformManager;
};
}

class Transform2dManager :
	public SingleComponentManager<Transform2d, editor::Transform2dInfo, ComponentType::TRANSFORM2D>
{
public:
	using SingleComponentManager::SingleComponentManager;
	Transform2d* AddComponent(Entity entity) override;
	void CreateComponent(json& componentJson, Entity entity) override;
	void DestroyComponent(Entity entity) override;
	void OnUpdate(float dt) override;
};

}

#endif /* INCLUDE_ENGINE_TRANSFORM_H_ */
