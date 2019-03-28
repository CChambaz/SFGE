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

#ifndef SFGE_EDITOR_H
#define SFGE_EDITOR_H

#include <memory>
#include <set>

#include <engine/system.h>
#include <engine/globals.h>

#include <SFML/Graphics/RenderWindow.hpp>
#include <editor/editor_info.h>
#include <editor/profiler.h>
#include <editor/tools_manager.h>

namespace sfge
{


class Body2dManager;
class Graphics2dManager;
class Transform2dManager;
class EntityManager;
class SceneManager;
class Physics2dManager;
class SoundManager;
class KeyboardManager;
struct Configuration;



class Editor : public System
{
public:
	Editor(Engine& engine);
	/**
	* \brief Initialize the SceneManager, get the Configuration from Engine and save the Scene lists from it
	*/
	void OnEngineInit() override;
	/**
	* \brief Update the SceneManager, mostly updating the GameObjects of the current Scene and doing the transition when needed
	* \param dt Delta time since last frame
	*/
	void OnUpdate(float dt) override;
	void ProcessEvent(sf::Event& event) const;
	void OnDraw() override;
	/**
	* \brief Finalize and delete everything created in the SceneManager
	*/
	void Destroy() override;

	void OnAfterSceneLoad() override;
	void OnBeforeSceneLoad() override;

	void SetCurrentScene(std::unique_ptr<editor::SceneInfo> sceneInfo);

	void AddDrawableObserver(editor::IDrawableManager* observer);
protected:
	sf::RenderWindow* m_Window = nullptr;
	Graphics2dManager* m_GraphicsManager = nullptr;
	EntityManager* m_EntityManager = nullptr;
	Configuration* m_Config = nullptr;
	KeyboardManager* m_KeyboardManager = nullptr;

	std::unique_ptr<editor::SceneInfo> m_CurrentScene = nullptr;
	editor::ProfilerEditorWindow m_ProfilerWindow{m_Engine};
	editor::ToolsManager m_ToolWindow{m_Engine};
	Entity selectedEntity = INVALID_ENTITY;
	int selectedDrawingProgram = -1;
	int selectedPySystem = -1;
	std::set<editor::IDrawableManager*> m_DrawableObservers;
#ifdef WIN32

	const sf::Keyboard::Key enablingKey = static_cast<sf::Keyboard::Key>(52);
#else
	const sf::Keyboard::Key enablingKey = static_cast<sf::Keyboard::Key>(54);
#endif
};

}

#endif
