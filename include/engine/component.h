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


#ifndef SFGE_COMPONENT_H
#define SFGE_COMPONENT_H

#include <queue>
#include <vector>
#include <any>

#include <engine/globals.h>
#include <utility/log.h>
#include <engine/entity.h>
#include <engine/system.h>
#include <engine/engine.h>
#include <engine/scene.h>
#include <editor/editor_info.h>
#include <editor/editor.h>

#include <utility/json_utility.h>
#include <engine/vector.h>

namespace sfge
{

enum class ComponentType : int
{
  	NONE = 0,
	TRANSFORM2D = 1 << 0,
	SPRITE2D = 1 << 1,
	SHAPE2D = 1 << 2,
	BODY2D = 1 << 3,
	COLLIDER2D = 1 << 4,
	SOUND = 1 << 5,
	PYCOMPONENT = 1 << 6,
	ANIMATION2D = 1 << 7
};

class IComponentFactory
{
 public:
  virtual void CreateComponent(json& componentJson, Entity entity) = 0;
};

template<typename T, ComponentType componentType>
class ComponentManager:
    public System,
    public DestroyObserver,
    public IComponentFactory
{
 protected:
  EntityManager* m_EntityManager = nullptr;
  std::vector<T> m_Components;
 public:
  ComponentManager(Engine& engine) : System(engine) {}
  ComponentManager(const ComponentManager&) = delete;
  ComponentManager(ComponentManager&& componentManager) = default;

  virtual T* AddComponent(Entity entity) = 0;
  virtual void DestroyComponent(Entity entity) = 0;

  void OnEngineInit() override
  {
	  System::OnEngineInit();
    m_EntityManager = m_Engine.GetEntityManager();
    m_EntityManager->AddDestroyObserver(this);
  }
  virtual ~ComponentManager()
  {
    m_Components.clear();
  }


  virtual T* GetComponentPtr(Entity entity) = 0;

  std::vector<T>& GetComponents()
  {
    return m_Components;
  }

  virtual void OnDestroy(Entity entity) override { (void) entity; }

};


template<typename TInfo>
class ComponentInfoManager : public editor::IDrawableManager
{
  static_assert(std::is_base_of<editor::ComponentInfo, TInfo>::value, "TInfo must be derived from ComponentInfo");
 public:
  ComponentInfoManager(ComponentType componentType): m_ComponentType(componentType)
  {

  }


protected:
  std::vector<TInfo> m_ComponentsInfo;
  ComponentType m_ComponentType;
};


template<typename T, typename TInfo, ComponentType componentType>
class BasicComponentManager: public ComponentManager<T, componentType>,
                             public ComponentInfoManager<TInfo>
{
public:
    BasicComponentManager(Engine& engine) : ComponentManager<T, componentType>(engine), ComponentInfoManager<TInfo>(componentType)
    {

    }
    virtual void DrawOnInspector(Entity entity) override
    {
      for (auto &info : ComponentInfoManager<TInfo>::m_ComponentsInfo)
      {
        if (ComponentManager<T, componentType>::m_EntityManager->HasComponent(entity, componentType) && info.GetEntity() == entity)
        {
          info.DrawOnInspector();
        }
      }
    }
    virtual void OnEngineInit() override
    {
		ComponentManager<T, componentType>::OnEngineInit();
		ComponentManager<T, componentType>::m_Engine.GetEditor()->AddDrawableObserver(this);
		ComponentManager<T, componentType>::m_Engine.GetSceneManager()->AddComponentManager(this, componentType);
	}

protected:
	virtual int GetFreeComponentIndex() = 0;
};

template<class T, class TInfo, ComponentType componentType>
class SingleComponentManager :
		public BasicComponentManager<T, TInfo, componentType>,
		public ResizeObserver
{
public:
	SingleComponentManager(Engine& engine):BasicComponentManager<T,TInfo, componentType>(engine)
	{
		BasicComponentManager<T,TInfo, componentType>::m_Components = std::vector<T>{ INIT_ENTITY_NMB };
        BasicComponentManager<T,TInfo, componentType>::m_ComponentsInfo = std::vector<TInfo>{ INIT_ENTITY_NMB };
        for(int i = 0; i < INIT_ENTITY_NMB;i++)
        {
          BasicComponentManager<T,TInfo, componentType>::m_ComponentsInfo[i].SetEntity(i+1);
        }
	}

	virtual void OnEngineInit() override
	{
		BasicComponentManager<T,TInfo, componentType>::OnEngineInit();
		BasicComponentManager<T,TInfo, componentType>::m_EntityManager = System::m_Engine.GetEntityManager();
		BasicComponentManager<T,TInfo, componentType>::m_EntityManager->AddResizeObserver(this);
	}
	virtual ~SingleComponentManager()
	{
	}

	TInfo& GetComponentInfo(Entity entity)
	{
		if (entity == INVALID_ENTITY)
		{
			Log::GetInstance()->Error("Trying to get component from INVALID_ENTITY");
		}
		return BasicComponentManager<T,TInfo, componentType>::m_ComponentsInfo[entity - 1];
	}

	virtual T* GetComponentPtr(Entity entity) override
	{
		if (entity == INVALID_ENTITY)
		{
			Log::GetInstance()->Error("Trying to get component from INVALID_ENTITY");
		}
		return &BasicComponentManager<T,TInfo, componentType>::m_Components[entity - 1];
	}

	T& GetComponentRef(Entity entity)
	{
		if (entity == INVALID_ENTITY)
		{
			Log::GetInstance()->Error("Trying to get component from INVALID_ENTITY");
		}
		return BasicComponentManager<T,TInfo, componentType>::m_Components[entity - 1];
	}

	void OnResize(size_t newSize) override
	{
		BasicComponentManager<T,TInfo, componentType>::m_Components.resize(newSize);
		BasicComponentManager<T,TInfo, componentType>::m_ComponentsInfo.resize(newSize);
	}
protected:

	virtual int GetFreeComponentIndex() override { return 0; };

};

template<class T, class TInfo, ComponentType componentType>
class MultipleComponentManager : 
	public BasicComponentManager<T,TInfo, componentType>,
	public ResizeObserver

{
 public:
	MultipleComponentManager(Engine& engine): BasicComponentManager<T,TInfo, componentType>(engine)
	{
		BasicComponentManager<T,TInfo, componentType>::m_Components = std::vector<T>{ INIT_ENTITY_NMB * MULTIPLE_COMPONENTS_MULTIPLIER};
		BasicComponentManager<T,TInfo, componentType>::m_ComponentsInfo = std::vector<TInfo>{ INIT_ENTITY_NMB * MULTIPLE_COMPONENTS_MULTIPLIER };
	}

	void OnEngineInit() override
    {
        BasicComponentManager<T,TInfo, componentType>::OnEngineInit();
		BasicComponentManager<T,TInfo, componentType>::m_EntityManager = BasicComponentManager<T,TInfo, componentType>::m_Engine.GetEntityManager();
		BasicComponentManager<T,TInfo, componentType>::m_EntityManager->AddResizeObserver(this);
    }

    virtual void OnResize(size_t newSize) override
    {
      BasicComponentManager<T,TInfo, componentType>::m_Components = std::vector<T>{ newSize * MULTIPLE_COMPONENTS_MULTIPLIER};
      BasicComponentManager<T,TInfo, componentType>::m_ComponentsInfo = std::vector<TInfo>{ newSize * MULTIPLE_COMPONENTS_MULTIPLIER };
    }
protected:

};



class LayerComponent
{
public:
	void SetLayer(int layer);
	int GetLayer() const;
	static bool LayerCompare(LayerComponent* s1, LayerComponent* s2);
protected:
	int m_Layer = 0;
};

template<class T>
class LayerComponentManager
{

};

template<>
class LayerComponentManager<LayerComponent>
{
public:

	virtual ~LayerComponentManager()
	{
		while (!m_LayerComponents.empty())
		{
			m_LayerComponents.pop();
		}
	};
	virtual void FillQueue() = 0;
protected:
	std::priority_queue<
		LayerComponent*, 
	std::deque<LayerComponent*>, 
	std::function<bool(LayerComponent*, LayerComponent*)>> m_LayerComponents{LayerComponent::LayerCompare};
};

class Offsetable
{
public:
	virtual ~Offsetable() = default;
	Offsetable(sf::Vector2f offset);
	Vec2f GetOffset() const;
	virtual void SetOffset(sf::Vector2f offset);
protected:
	Vec2f m_Offset;
};


}
#endif
