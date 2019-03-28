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
#include <engine/engine.h>
#include <engine/config.h>
#include <engine/scene.h>
#include <gtest/gtest.h>
#include <audio/audio.h>
#include <audio/sound.h>
#include <utility/json_utility.h>
#include <SFML/Audio.hpp>

TEST(Audio, TestSoundComponent)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	config->windowLess = false ;
	engine.Init(std::move(config));

	json sceneJson;
	json entityJson;
	json soundJson;
	soundJson["path"] = "data/sounds/doorClose_1.ogg";
	soundJson["type"] = static_cast<int>(sfge::ComponentType::SOUND);
	

	json playSoundScriptJson;
	playSoundScriptJson["script_path"] = "scripts/play_sound.py";
	playSoundScriptJson["type"] = static_cast<int>(sfge::ComponentType::PYCOMPONENT);
	entityJson["components"] = json::array({ soundJson, playSoundScriptJson });

	json fakeEntityJson;
	json fakeSoundJson;
	fakeSoundJson["path"] = "fake/path/prout.wav";
	fakeSoundJson["type"] = static_cast<int>(sfge::ComponentType::SOUND);
	fakeEntityJson["components"] = json::array({ fakeSoundJson });


	sceneJson["entities"] = json::array({ entityJson, fakeEntityJson });
	sceneJson["name"] = "Test Sound";
	engine.GetSceneManager()->LoadSceneFromJson(sceneJson);

	engine.Start();
}


TEST(Audio, TestSoundBuffer)
{
	sfge::Engine engine;

	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	config->windowLess = true;
	engine.Init(std::move(config));

	auto* soundBufferManager = engine.GetAudioManager()->GetSoundBufferManager();

	const std::string goodSoundBufferPath = "data/sounds/doorClose_1.ogg";
	const std::string badTextPath = "fake/path/prout.ogg";
	const std::string badTextPathWithoutExtension = "fake/path/prout";

	const auto goodSndBufferId = soundBufferManager->LoadSoundBuffer(goodSoundBufferPath);
	const auto badSndBufferId = soundBufferManager->LoadSoundBuffer(badTextPath);
	const auto badExtSndBufferId = soundBufferManager->LoadSoundBuffer(badTextPathWithoutExtension);


	ASSERT_EQ(badExtSndBufferId, sfge::INVALID_SOUND_BUFFER);
	ASSERT_EQ(badSndBufferId, sfge::INVALID_SOUND_BUFFER);
	ASSERT_NE(goodSndBufferId, sfge::INVALID_SOUND_BUFFER);

	sf::Sound sound;
	if (badSndBufferId != sfge::INVALID_SOUND_BUFFER)
	{
		sfge::Log::GetInstance()->Error("Loading Bad File");
		sound.setBuffer(*soundBufferManager->GetSoundBuffer(badSndBufferId));
	}
	else
	{
		std::ostringstream oss;
		oss << "Bad file: " << badTextPath << " could not be loaded";
		sfge::Log::GetInstance()->Error(oss.str());
	}
	if (goodSndBufferId != sfge::INVALID_SOUND_BUFFER)
	{

		sfge::Log::GetInstance()->Msg("Loading Good File");
		sound.setBuffer(*soundBufferManager->GetSoundBuffer(goodSndBufferId));
	}
	else
	{
		std::ostringstream oss;
		oss << "Bad file: " << goodSoundBufferPath << " could not be loaded";
		sfge::Log::GetInstance()->Error(oss.str());
	}

	sound.play();

	std::this_thread::sleep_for(std::chrono::milliseconds
	(soundBufferManager->GetSoundBuffer(goodSndBufferId)->getDuration().asMilliseconds()));

	engine.Destroy();
}
