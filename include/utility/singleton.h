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
#ifndef SFGE_SINGLETON_H
#define SFGE_SINGLETON_H

#include <memory>
namespace sfge
{
/**
* \brief Singleton template class used by the modules of the Engine
*/
template<typename T>
class Singleton
{
protected:
	/* Here will be the instance stored. */
	static std::unique_ptr<T> instancePtr;
	static T* instance;
	/**
	* \brief Private Constructor to prevent instancing.
	*/
	Singleton() {};
	/**
	* \brief Private destructor
	*/
	virtual ~Singleton() {
		instance = nullptr;
	};

public:
	/**
	* \brief Static access method.
	*/
	static T* GetInstance()
	{
		{
			if (instance == nullptr)
			{
				instancePtr = std::make_unique<T>();
				instance = instancePtr.get();
			}

			return instance;

		}
	}
};

template<typename T>
std::unique_ptr<T> Singleton<T>::instancePtr = nullptr;
template<typename T>
T* Singleton<T>::instance = nullptr;

}
#endif
