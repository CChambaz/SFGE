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

#ifndef SFGE_SHAPE_H_
#define SFGE_SHAPE_H_

#include <list>

#include <engine/system.h>
#include <engine/component.h>
#include <engine/transform2d.h>
#include <editor/editor.h>
//Externals
#include <SFML/Graphics.hpp>

namespace sfge
{

enum class ShapeType
{
	NONE,
	CIRCLE,
	RECTANGLE,
	POLYGON,
	CONVEX,
};

class Shape : public Offsetable
{
public:
  	Shape();
	Shape(Transform2d* transform, sf::Vector2f offset);
  	Shape ( Shape && ) = default; //move constructor
  	Shape ( const Shape & ) = delete; //delete copy constructor
  	virtual ~Shape();
	void Draw(sf::RenderWindow& window) const;
	void SetFillColor(sf::Color color) const;
	void Update() const;
	void SetShape(std::unique_ptr<sf::Shape> shape);
	sf::Shape* GetShape();
protected:
	friend class ShapeManager;
	Transform2d transform;
	std::unique_ptr<sf::Shape> m_Shape = nullptr;
	Entity entity = INVALID_ENTITY;
};
class ShapeManager;
namespace editor
{

struct ShapeInfo : ComponentInfo
{

	void DrawOnInspector() override;
	ShapeManager* shapeManager;
};

}

class ShapeManager :
	public SingleComponentManager<Shape, editor::ShapeInfo, ComponentType::SHAPE2D>
{

public:
	using SingleComponentManager::SingleComponentManager; 
	ShapeManager(ShapeManager&& shapeManager) = default;

	void OnEngineInit() override;
	void DrawShapes(sf::RenderWindow &window);
	void OnUpdate(float dt) override;
	void OnBeforeSceneLoad() override;

	Shape* AddComponent(Entity entity) override;
	void CreateComponent(json& componentJson, Entity entity) override;
	void DestroyComponent(Entity entity) override;

	void OnResize(size_t new_size) override;
protected:
	Transform2dManager* m_Transform2dManager;
};



}
#endif /* SRC_GRAPHICS_SHAPE_H_ */
