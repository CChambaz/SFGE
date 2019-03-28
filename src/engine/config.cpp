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

//STL includes
#include <fstream>
//SFGE includes
#include <engine/config.h>
#include <utility/log.h>


namespace sfge
{
Configuration::~Configuration()
{
}
std::unique_ptr<Configuration> Configuration::LoadConfig(std::string configFilename)
{
	{
		std::ostringstream oss;
		oss << "Creating Configuration from " << configFilename;
		Log::GetInstance()->Msg(oss.str());
	}

	auto jsonConfigPtr = LoadJson(configFilename);
	if (jsonConfigPtr == nullptr)
	{
		std::ostringstream oss;
		oss << "[Error] Config JSON file: " << configFilename << " failed to open or did not parse as JSON";
		Log::GetInstance()->Error(oss.str());
		return nullptr;
	}
	return LoadConfig(*jsonConfigPtr);
}
std::unique_ptr<Configuration> Configuration::LoadConfig(json& configJson)
{

	auto newConfig = std::make_unique<Configuration>();
	newConfig->screenResolution = sf::Vector2i(
		configJson["screenResolution"]["x"],
		configJson["screenResolution"]["y"]);
	if (CheckJsonExists(configJson, "gravity"))
	{
		newConfig->gravity = b2Vec2(
			configJson["gravity"]["x"],
			configJson["gravity"]["y"]
		);
	}
	newConfig->maxFramerate = configJson["maxFramerate"];

	if(CheckJsonExists(configJson, "devMode"))
		newConfig->devMode = configJson["devMode"];
	return newConfig;
}

}
