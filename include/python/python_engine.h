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

#ifndef SFGE_PYENGINE_H
#define SFGE_PYENGINE_H

#include <list>
#include <list>

#include <engine/system.h>
#include <utility/python_utility.h>
#include <engine/component.h>
#include <python/pysystem.h>

namespace sfge
{


struct ColliderData;

using ModuleId = unsigned;
const ModuleId INVALID_MODULE = 0U;

using InstanceId = unsigned;
const InstanceId INVALID_INSTANCE = 0U;

/**
* \brief Manage the python interpreter
*/
class PythonEngine :
		public System
{
public:
	using System::System;
	/**
	* \brief Initialize the python interpreter
	*/
	void OnEngineInit() override;


	void InitScriptsInstances();
	/**
	* \brief Update the python interpreter, called only in play mode
	* \param dt The delta time since last frame
	*/
	void OnUpdate(float dt) override;
	void OnFixedUpdate() override;
	void OnDraw() override;
	/**
	* \brief Finalize the python interpreter
	*/
	void Destroy() override;

	void OnAfterSceneLoad() override;
	void OnBeforeSceneLoad() override;

	ModuleId LoadPyModule(std::string moduleFilename);


	PySystemManager& GetPySystemManager(){ return m_PySystemManager; }

	void ExecutePythonCommand(std::string pythonCommand);

	const std::string& GetClassNameFrom(ModuleId moduleId);
	const std::string& GetModuleNameFrom(ModuleId moduleId);
	const std::string& GetModulePathFrom(ModuleId moduleId);
	const py::object & GetModuleObjFrom(ModuleId moduleId);

	/*
	*/
    void SpreadClasses();
private:
	/**
	 * \brief Load all the python scripts at initialization or reset
	 */
	void LoadScripts(std::string dirname = "scripts/");


	std::vector<std::string> m_PythonModulePaths{ INIT_ENTITY_NMB * MULTIPLE_COMPONENTS_MULTIPLIER };
	std::vector<std::string> m_PyClassNames{ INIT_ENTITY_NMB * MULTIPLE_COMPONENTS_MULTIPLIER };
	std::vector<std::string> m_PyModuleNames{ INIT_ENTITY_NMB * MULTIPLE_COMPONENTS_MULTIPLIER };
	std::vector<py::object> m_PyModuleObjs{INIT_ENTITY_NMB * MULTIPLE_COMPONENTS_MULTIPLIER};

	ModuleId m_IncrementalModuleId = 1U;

	PySystemManager m_PySystemManager{m_Engine};

};

}
#endif /* SFGE_PYENGINE_H */
