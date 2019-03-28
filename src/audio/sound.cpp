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
#include <sstream>
#include <string>
#include <list>
#include <set>


#include "imgui.h"

#include <audio/sound.h>
#include <audio/audio.h>
#include <utility/log.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <utility/file_utility.h>
#include <utility/json_utility.h>


namespace sfge
{

static std::set<std::string> sndExtensionSet
{
	".wav",
	".ogg",
	".flac"
};



SoundManager::SoundManager(Engine& engine): BasicComponentManager(engine)
{
	m_Components.resize(MAX_SOUND_CHANNELS-MUSIC_INSTANCES_NMB);
	m_ComponentsInfo.resize(MAX_SOUND_CHANNELS-MUSIC_INSTANCES_NMB);
}

SoundManager::~SoundManager()
{

}

Sound* SoundManager::AddComponent(Entity entity)
{
	Sound* sound = nullptr;
	editor::SoundInfo* soundInfo = nullptr;
	const int id = GetFreeComponentIndex();
	if(id != -1)
	{
		sound = &m_Components[id];
		soundInfo = &m_ComponentsInfo[id];

		soundInfo->Sound = sound;
		m_EntityManager->AddComponentType(entity, ComponentType::SOUND);
		return sound;
	}
	return nullptr;
}

void SoundManager::CreateComponent(json & componentJson, Entity entity)
{

	if (CheckJsonParameter(componentJson, "path", json::value_t::string))
	{
		const auto path = componentJson["path"].get<std::string>();
		sf::SoundBuffer* soundBuffer = nullptr;
		if (FileExists(path))
		{
			auto* sound = AddComponent(entity);
			if(sound == nullptr)
			{
				std::ostringstream oss;
				oss << "All sound channels used";
				Log::GetInstance()->Error(oss.str());
				return;
			}
			sound->SetEntity(entity);
			const int index = sound - &m_Components[0];
			auto* soundInfo = &m_ComponentsInfo[index];
			const SoundBufferId soundBufferId = m_SoundBufferManager->LoadSoundBuffer(path);
			if (soundBufferId != INVALID_SOUND_BUFFER)
			{
				soundInfo->SetEntity(entity);
				soundInfo->path = path;
				soundBuffer = m_SoundBufferManager->GetSoundBuffer(soundBufferId);
				sound->SetBuffer(soundBuffer);
				soundInfo->SoundBufferId = soundBufferId;
			}
			else
			{
				std::ostringstream oss;
				oss << "Sound file " << path << " cannot be loaded";
				Log::GetInstance()->Error(oss.str());
			}
		}
		else
		{
			std::ostringstream oss;
			oss << "Sound file " << path << " does not exist";
			Log::GetInstance()->Error(oss.str());
		}
	}
	else
	{
		Log::GetInstance()->Error("[Error] No Path for Sound");
	}

}

void SoundManager::DestroyComponent(Entity entity)
{
	(void) entity;
}

sfge::Sound::Sound()
{
}

sfge::Sound::~Sound()
{
}
void Sound::Init()
{
}
void Sound::SetBuffer(sf::SoundBuffer* buffer)
{
	m_Sound.setBuffer(*buffer);
}

void SoundManager::OnEngineInit()
{
	BasicComponentManager::OnEngineInit();
	m_Components.resize(MAX_SOUND_CHANNELS);
	m_ComponentsInfo.resize(MAX_SOUND_CHANNELS);
	m_SoundBufferManager = m_Engine.GetAudioManager()->GetSoundBufferManager();
}
void SoundManager::OnBeforeSceneLoad() {}

void SoundManager::OnAfterSceneLoad() {}

Sound *SoundManager::GetComponentPtr(Entity entity)
{
	for(auto& sound : m_Components)
	{
		if (sound.GetEntity() == entity)
			return &sound;
	}
	return nullptr;
}

int SoundManager::GetFreeComponentIndex()
{
	for (auto i = 0u; i < MAX_SOUND_CHANNELS; i++)
	{
		if (m_Components[i].GetEntity() == INVALID_ENTITY)
		{
			return i;
		}
	}
	return -1;
}

void Sound::Play()
{
	m_Sound.play();

}

void Sound::Stop()
{
	m_Sound.stop();
}

Entity Sound::GetEntity()
{
	return m_Entity;
}
void Sound::SetEntity(Entity entity)
{
	m_Entity = entity;
}

SoundBufferManager::~SoundBufferManager()
{
	m_SoundBuffers.clear();
}

void SoundBufferManager::OnEngineInit()
{
	if (const auto config = m_Engine.GetConfig())
	{
		if (config->devMode)
		{
			LoadSoundBuffers(config->dataDirname);
		}
	}
}

void SoundBufferManager::LoadSoundBuffers(std::string dataDirname)
{
	std::function<void(std::string)> LoadAllSoundBuffers;
	LoadAllSoundBuffers = [&LoadAllSoundBuffers, this](std::string entry)
	{
		if (IsRegularFile(entry) && HasValidExtension(entry))
		{
			if(CalculateFileSize(entry) > MAX_SOUND_BUFFER_SIZE)
			{
				const auto newSoundBufferId = LoadSoundBuffer(entry);
				if (newSoundBufferId != INVALID_SOUND_BUFFER)
				{
					std::ostringstream oss;
					oss << "Loading soundbuffers: " << entry << "\n";
					Log::GetInstance()->Msg(oss.str());
				}
			}
		}

		if (IsDirectory(entry))
		{
			IterateDirectory(entry, LoadAllSoundBuffers);
		}
	};
	IterateDirectory(dataDirname, LoadAllSoundBuffers);
}


bool SoundBufferManager::HasValidExtension(std::string filename)
{
	const std::string::size_type filenameExtensionIndex = filename.find_last_of('.');
	if (filenameExtensionIndex >= filename.size())
	{
		return false;
	}

	//Check extension first
	const std::string extension = filename.substr(filenameExtensionIndex);
	return sndExtensionSet.find(extension) != sndExtensionSet.end();
}

void SoundBufferManager::OnBeforeSceneLoad()
{
	for (auto& soundBufferRefCount : m_SoundBufferCountRefs)
	{
		soundBufferRefCount = 0U;
	}
}

void SoundBufferManager::OnAfterSceneLoad()
{
	std::list<SoundBufferId> unusedBufferIds;
	for (auto i = 0U; i < m_SoundBufferCountRefs.size(); i++)
	{
		if (m_SoundBuffers[i] && m_SoundBufferCountRefs[i] == 0U)
		{
			unusedBufferIds.push_back(i + 1);
		}
	}
	for (auto unusedTextureId : unusedBufferIds)
	{
		m_SoundBuffers[unusedTextureId] = nullptr;
	}
}

SoundBufferId SoundBufferManager::LoadSoundBuffer(std::string filename)
{
	if(!FileExists (filename))
	{
		std::ostringstream oss;
		oss << "[ERROR] Sound buffer path: " << filename << " does not exist";
		Log::GetInstance()->Error(oss.str());
		return INVALID_SOUND_BUFFER;
	}
	if (!HasValidExtension(filename))
	{
		std::ostringstream oss;
		oss << "[ERROR] Sound buffer path: " << filename << " has invalid extension";
		Log::GetInstance()->Error(oss.str());
		return INVALID_SOUND_BUFFER;
	}



	auto soundBufferId = INVALID_SOUND_ID;
	for (SoundBufferId checkedId = 1U; checkedId <= m_IncrementId; checkedId++)
	{
		if (filename == m_SoundBufferPaths[checkedId])
		{
			soundBufferId = checkedId;
		}
	}
	//Was or still is loaded
	if (soundBufferId != INVALID_SOUND_BUFFER)
	{

		//Check if the texture was destroyed
		if (m_SoundBuffers[soundBufferId - 1] != nullptr)
		{
			m_SoundBufferCountRefs[soundBufferId - 1]++;
			return soundBufferId;
		}
		else
		{
			auto soundBuffer = std::make_unique<sf::SoundBuffer>();
			if (!soundBuffer->loadFromFile(filename))
			{
				std::ostringstream oss;
				oss << "[ERROR] Could not load sound buffer file: " << filename;
				Log::GetInstance()->Error(oss.str());
				return INVALID_SOUND_BUFFER;
			}
			m_SoundBufferCountRefs[soundBufferId - 1] = 1U;
			m_SoundBuffers[soundBufferId - 1] = std::move(soundBuffer);
			return m_IncrementId;
		}
	}
	else
	{
		//SoundBuffer was never loaded
		if (FileExists(filename))
		{

			auto soundBuffer = std::make_unique<sf::SoundBuffer>();
			if (!soundBuffer->loadFromFile(filename))
			{
				std::ostringstream oss;
				oss << "[ERROR] Could not load sound file: " << filename;
				Log::GetInstance()->Error(oss.str());
				return INVALID_SOUND_ID;
			}

			m_IncrementId++;
			m_SoundBufferPaths[m_IncrementId - 1] = filename;
			m_SoundBufferCountRefs[m_IncrementId - 1] = 1U;
			m_SoundBuffers[m_IncrementId - 1] = std::move(soundBuffer);
			return m_IncrementId;
		}
		else
		{
			std::ostringstream oss;
			oss << "[ERROR] Could not load sound file: " << filename;
			Log::GetInstance()->Error(oss.str());
		}
	}
	return INVALID_SOUND_BUFFER;
}

sf::SoundBuffer* SoundBufferManager::GetSoundBuffer(SoundBufferId soundBufferId)
{
	return m_SoundBuffers[soundBufferId - 1].get();
}

}

void sfge::editor::SoundInfo::DrawOnInspector()
{
	ImGui::Separator();
	ImGui::Text("Sound");


	ImGui::LabelText("Sound Path", "%s", path.c_str());
	ImGui::InputInt("Sound Id", reinterpret_cast<int*>(&SoundBufferId));
	
}
