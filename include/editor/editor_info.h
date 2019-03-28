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
#ifndef SFGE_EDITOR_INFO_H
#define SFGE_EDITOR_INFO_H
#include <string>
#include <engine/globals.h>

namespace sfge
{
class Engine;
enum class ComponentType : int;
}
//Editor components
namespace sfge::editor
{

struct EntityInfo;
struct ComponentInfo;


class IDrawableInspector
{
public:
	virtual ~IDrawableInspector(){};
    void SetEntity(Entity entity);
    Entity GetEntity();
    virtual void DrawOnInspector() = 0;
protected:
  	Entity m_Entity;

};

class IDrawableManager
{
public:
  	virtual void DrawOnInspector(Entity entity) = 0;
};

struct NamableEditorComponent
{
	std::string name = "";
};

struct PathEditorComponent
{
	std::string path = "";
};


struct SceneInfo : NamableEditorComponent, PathEditorComponent
{
};

struct ComponentInfo : NamableEditorComponent, IDrawableInspector
{
};

}

#endif