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


#ifndef SFGE_SOUND_H
#define SFGE_SOUND_H
#include <vector>

#include <SFML/Audio.hpp>
#include <engine/component.h>
#include <editor/editor_info.h>

namespace sfge
{
class EntityManager;
class SoundManager;

using SoundId = unsigned int;
const SoundId INVALID_SOUND_ID = 0U;

using SoundBufferId = unsigned int;
const SoundBufferId INVALID_SOUND_BUFFER = 0U;

const auto MAX_SOUND_BUFFER_SIZE = 1'000'000ll;



/**
* \brief Sound class child is a Component
*/
class Sound
{
public:
	Sound();
	~Sound();
	/**
	* \brief initialize the Sound class
	*/
	void Init();


	void SetBuffer(sf::SoundBuffer* buffer);
	void Play();
	void Stop();

	Entity GetEntity();
	void SetEntity(Entity newEntity);
protected:
	sf::Sound m_Sound;
	Entity m_Entity = INVALID_ENTITY;
};

class SoundBufferManager : public System
{
public:

	using System::System;

	~SoundBufferManager();

	void OnEngineInit() override;

	void LoadSoundBuffers(std::string filename);

	void OnBeforeSceneLoad() override;
	
	void OnAfterSceneLoad() override;

	SoundBufferId LoadSoundBuffer(std::string filename);
	sf::SoundBuffer* GetSoundBuffer(SoundBufferId soundBufferId);
private:

  	bool HasValidExtension(std::string filename);
	std::vector<std::string> m_SoundBufferPaths{ INIT_ENTITY_NMB };
	std::vector<size_t> m_SoundBufferCountRefs{ INIT_ENTITY_NMB };
	std::vector<std::unique_ptr<sf::SoundBuffer>> m_SoundBuffers{INIT_ENTITY_NMB};
	SoundBufferId m_IncrementId = 0U;

};

namespace editor
{
struct SoundInfo : ComponentInfo, PathEditorComponent
{
	void DrawOnInspector() override;
	sfge::SoundBufferId SoundBufferId = sfge::INVALID_SOUND_BUFFER;
	sfge::Sound* Sound = nullptr;
	std::string bufferPath = "";
};
}

const size_t MAX_SOUND_CHANNELS = 16;

class SoundManager : public BasicComponentManager<Sound, editor::SoundInfo, ComponentType::SOUND>
{
public:
	SoundManager(Engine& engine);
	~SoundManager();

    void OnEngineInit() override;

	Sound* AddComponent(Entity entity) override;

	//TODO use similar construction as pycomponent
	void CreateComponent(json& componentJson, Entity entity) override;
	void DestroyComponent(Entity entity) override;
	void OnBeforeSceneLoad() override;
	void OnAfterSceneLoad() override;

	Sound* GetComponentPtr(Entity entity) override;

protected:
	int GetFreeComponentIndex() override;
	SoundBufferManager* m_SoundBufferManager = nullptr;
};
}
#endif