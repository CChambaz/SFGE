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
#ifdef WIN32
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <vector>

//SFGE includes
#include <engine/scene.h>
#include <utility/log.h>
#include <utility/json_utility.h>
#include <editor/editor.h>
#include <engine/config.h>
#include <utility/file_utility.h>
#include <engine/entity.h>
#include <graphics/graphics2d.h>
#include <python/python_engine.h>
#include <python/pysystem.h>
#include <physics/physics2d.h>
#include <audio/audio.h>
#include <engine/engine.h>

// for convenience


namespace sfge
{
SceneManager::SceneManager(Engine& engine):
	System(engine)
	
{
}

void SceneManager::OnEngineInit()
{
	m_EntityManager = m_Engine.GetEntityManager();
	if(auto config = m_Engine.GetConfig())
	{
		SearchScenes(config->dataDirname);
	}
	else
	{
		Log::GetInstance()->Error("No config in SceneManager Init");
	}
}

void SceneManager::SearchScenes(std::string& dataDirname)
{
	std::function<void(std::string)> SearchAllScenes;
	SearchAllScenes = [&SearchAllScenes, this](std::string entry)
	{
		
		if (IsRegularFile(entry))
		{
			const std::string::size_type filenameExtensionIndex = entry.find_last_of('.');
			const std::string extension = entry.substr(filenameExtensionIndex);
			if(extension == ".scene")
			{
				const auto sceneJsonPtr = LoadJson(entry);
				if(sceneJsonPtr && CheckJsonExists(*sceneJsonPtr, "name"))
				{
					m_ScenePathMap.insert(std::pair<std::string, std::string>((*sceneJsonPtr)["name"],entry));
				}
			}
		}

		if (IsDirectory(entry))
		{
			{
				std::ostringstream oss;
				oss << "Opening folder: " << entry << "\n";
				Log::GetInstance()->Msg(oss.str());
			}
			IterateDirectory(entry, SearchAllScenes);
		}
	};
	IterateDirectory(dataDirname, SearchAllScenes);
}


void SceneManager::LoadSceneFromPath(const std::string& scenePath)
{
	{
		std::ostringstream oss;
		oss << "Loading scene from: " << scenePath;
		Log::GetInstance()->Msg(oss.str());
	}
	const auto sceneJsonPtr = LoadJson(scenePath);
	
	if(sceneJsonPtr != nullptr)
	{
		auto sceneInfo = std::make_unique<editor::SceneInfo>();
		sceneInfo->path = scenePath;
		LoadSceneFromJson(*sceneJsonPtr, std::move(sceneInfo));
	}
	else
	{
		Log::GetInstance()->Error("Invalid JSON format for scene");
	}

	
}



void SceneManager::LoadSceneFromJson(json& sceneJson, std::unique_ptr<editor::SceneInfo> sceneInfo)
{
	m_Engine.Clear();
	if(!sceneInfo)
		sceneInfo = std::make_unique<editor::SceneInfo>();
	if (CheckJsonParameter(sceneJson, "name", json::value_t::string))
	{
		sceneInfo->name = sceneJson["name"].get<std::string>();
	}
	else
	{
		sceneInfo->name = "NewScene";
	}
	{
		std::ostringstream oss;
		oss << "Loading scene: " << sceneInfo->name;
		Log::GetInstance()->Msg(oss.str());
	}
	if (CheckJsonParameter(sceneJson, "systems", json::value_t::array))
	{
		for (auto& systemJson : sceneJson["systems"])
		{
			auto* pythonEngine = m_Engine.GetPythonEngine();
			if (CheckJsonExists(systemJson, "script_path"))
			{
				std::string path = systemJson["script_path"];
				const ModuleId moduleId = pythonEngine->LoadPyModule(path);
				if (moduleId != INVALID_MODULE)
				{
					const InstanceId instanceId = pythonEngine->GetPySystemManager().LoadPySystem(moduleId);
					PySystem* pySystem = pythonEngine->GetPySystemManager().GetPySystemFromInstanceId(instanceId);
					if(pySystem != nullptr)
					{
						m_ScenePySystems.push_back(pySystem);
					}
					else
					{
						Log::GetInstance()->Error("[Python Error] Returned PySystem is null");
					}
				}
				else
				{
					std::ostringstream oss;
					oss << "Could not load PySystem at "<<path;
					Log::GetInstance()->Error(oss.str());
				}
			}
			if(CheckJsonExists(systemJson, "systemClassName"))
			{
				const std::string systemClassName = systemJson["systemClassName"];
				auto instanceId = pythonEngine->GetPySystemManager().LoadCppExtensionSystem(systemClassName);
				if(instanceId != INVALID_INSTANCE)
				{
					PySystem* pySystem = pythonEngine->GetPySystemManager().GetPySystemFromInstanceId(instanceId);
					if(pySystem != nullptr)
					{
						m_ScenePySystems.push_back(pySystem);
					}
				}
			}
		}
	}
	if (CheckJsonParameter(sceneJson, "entities", json::value_t::array))
	{
		const auto entityNmb = sceneJson["entities"].size();
		if(entityNmb > INIT_ENTITY_NMB)
		{
			m_EntityManager->ResizeEntityNmb(entityNmb);
		}
		for(auto& entityJson : sceneJson["entities"])
		{
			Entity entity = INVALID_ENTITY;
			entity = m_EntityManager->CreateEntity(INVALID_ENTITY);
			if(entity == INVALID_ENTITY)
			{
				std::ostringstream oss;
				oss << "[Error] Scene: not enough entities left";
				Log::GetInstance()->Error(oss.str());
				continue;
			}
			if(CheckJsonExists(entityJson, "name"))
			{
				m_EntityManager->GetEntityInfo(entity).name = entityJson["name"].get<std::string>();
			}
			else
			{
				std::ostringstream oss;
				oss << "Entity " << entity;
				m_EntityManager->GetEntityInfo(entity).name = oss.str();
			}
			if (entity != INVALID_ENTITY && 
				CheckJsonExists(entityJson, "components"))
			{
				
				for (auto& componentJson : entityJson["components"])
				{
					if (CheckJsonExists(componentJson, "type"))
					{
						const ComponentType componentType = componentJson["type"];
						const auto index = static_cast<int>(log2(static_cast<double>(componentType)));
						if(m_ComponentManager[index] != nullptr)
						{
							m_ComponentManager[index]->CreateComponent(componentJson, entity);
							m_EntityManager->AddComponentType(entity, componentType);
						}
					}
					else
					{
						std::ostringstream oss;
						oss << "[Error] No type specified for component with json content: " << componentJson;
						Log::GetInstance()->Error(oss.str());
					}
				}
			}
			else
			{
				std::ostringstream oss;
				oss << "[Error] No components attached in the JSON entity: " << entity << "with json content: " << entityJson;
				Log::GetInstance()->Error(oss.str());
			}
		}
	}
	else
	{
		std::ostringstream oss;
		oss << "No Entities in " << sceneInfo->name;
		Log::GetInstance()->Error(oss.str());
	}

	//remove previous scene assets

	m_Engine.Collect();

	auto* editor = m_Engine.GetEditor();
	editor->SetCurrentScene(std::move(sceneInfo));
	
	auto* pythonEngine = m_Engine.GetPythonEngine();
	pythonEngine->InitScriptsInstances();

	InitScenePySystems();
	
	
}

std::list<std::string> SceneManager::GetAllScenes()
{
	std::list<std::string> scenes;
	std::for_each(m_ScenePathMap.begin(),m_ScenePathMap.end(), [&](const std::pair<const std::string, std::string>& ref) {
		scenes.push_back(ref.first);
	});
	return scenes;
}


void SceneManager::LoadSceneFromName(const std::string& sceneName)
{
	if (m_ScenePathMap.find(sceneName) != m_ScenePathMap.end())
	{

		sf::Clock loadingClock;
		m_Engine.Clear();
		LoadSceneFromPath(m_ScenePathMap[sceneName]);
		{
			sf::Time loadingTime = loadingClock.getElapsedTime();
			std::ostringstream oss;
			oss << "Scene Loading Time: " << loadingTime.asSeconds();
			Log::GetInstance()->Msg(oss.str());
		}
	}
	else
	{
		std::ostringstream oss;
		oss << "[ERROR] No scene is named: " << sceneName <<"\n";
		oss << "Here are the list of scenes:\n";
		for(auto& sceneNamePair : m_ScenePathMap)
		{
			oss << "- " << sceneNamePair.first << "\n";
		}
		Log::GetInstance()->Error(oss.str());
	}
}
void SceneManager::AddComponentManager(IComponentFactory *componentFactory, ComponentType componentType)
{
	const auto index = static_cast<int>(log2((double)componentType));
	m_ComponentManager[index] = componentFactory;
}
void SceneManager::OnUpdate(float dt)
{
	rmt_ScopedCPUSample(PySceneSystemUpdate,0);
	for(auto* pySystem: m_ScenePySystems)
	{
		pySystem->OnUpdate(dt);
	}
}
void SceneManager::OnFixedUpdate()
{
	rmt_ScopedCPUSample(PySceneSystemFixedUpdate,0);
	for(auto* pySystem: m_ScenePySystems)
	{
		pySystem->OnFixedUpdate();
	}
}
void SceneManager::Destroy()
{
	m_ScenePySystems.clear();
}
void SceneManager::InitScenePySystems()
{
	rmt_ScopedCPUSample(PySceneSystemInit,0);
	for(auto* pySystem: m_ScenePySystems)
	{
		if(pySystem != nullptr)
		{
			pySystem->OnEngineInit();
		}
	}
}
void SceneManager::OnDraw()
{
	rmt_ScopedCPUSample(PySceneSystemDraw,0);
	for(auto* pySystem: m_ScenePySystems)
	{
		pySystem->OnDraw();
	}
}

	void SceneManager::OnBeforeSceneLoad() {
		Destroy();
	}

std::vector<PySystem*>& SceneManager::GetSceneSystems()
{
	return m_ScenePySystems;
}
}
