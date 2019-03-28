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

#ifndef SFGE_AUDIO_H
#define SFGE_AUDIO_H

#include <map>
#include <string>
#include <memory>
#include <string>

#include <SFML/Audio.hpp>

#include <engine/system.h>
#include <audio/sound.h>
#include <audio/music.h>

namespace sfge
{

class AudioManager : public System
{

public:
	using System::System;

	//AudioManager(const AudioManager&) = delete;
	/**
	* \brief Initialize SoundManager class, SoundBuffer class and MusicManager class
	*/
	void OnEngineInit() override;
	/**
	* \brief Update the audioManager, called only in play mode
	* \ param dt The delta time since last frame
	*/
	void OnUpdate(float dt) override;

	SoundManager* GetSoundManager();
	MusicManager* GetMusicManager();
	SoundBufferManager* GetSoundBufferManager();
	void Destroy() override;
	void OnBeforeSceneLoad() override;
	void OnAfterSceneLoad() override;
protected:
	SoundManager m_SoundManager {m_Engine};
	SoundBufferManager m_SoundBufferManager{ m_Engine };
	MusicManager m_MusicManager;
};
}
#endif // !SFGE_AUDIO
