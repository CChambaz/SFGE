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
#ifndef SFGE_MUSIC_H
#define SFGE_MUSIC_H

#include <vector>
#include <string>
#include <SFML/Audio/Music.hpp>

#include <engine/system.h>
namespace sfge
{

using MusicId = unsigned int;
const MusicId INVALID_MUSIC_ID = 0U;

const size_t MUSIC_INSTANCES_NMB = 1;

class MusicManager
{
public:
	MusicManager() = default;
	MusicManager(const MusicManager&) = delete;
	MusicManager& operator=(const MusicManager&) = delete;
	MusicManager(MusicManager&&) = default;
	MusicManager& operator = (MusicManager &&) = default;

	void Play(const std::string& musicPath);

protected:
	sf::Music m_Music;
};

}

#endif