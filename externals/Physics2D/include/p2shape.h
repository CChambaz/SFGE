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

#ifndef SFGE_P2SHAPE_H
#define SFGE_P2SHAPE_H

#include <p2vector.h>
#include <SFML/Graphics/Shape.hpp>
#include "SFML/Graphics/RectangleShape.hpp"
#include "SFML/Graphics/CircleShape.hpp"

enum ShapeType
{
	CIRCLE,
	RECT
};
/**
* \brief Abstract representation of a shape
*/
class p2Shape
{
public:
	ShapeType m_Type;
};

/**
* \brief Representation of a physics circle
*/
class p2CircleShape : public p2Shape
{
public:
	p2CircleShape(float radius = 1.0f);
	/**
	* \brief Setter for the radius
	*/
	void SetRadius(float radius);
private:
	float m_Radius;
};

/** 
* \brief Representation of a rectangle
*/
class p2RectShape : public p2Shape
{
public:
	p2RectShape(p2Vec2 size = p2Vec2());
	void SetSize(p2Vec2 size);
private:
	p2Vec2 m_Size;
};

class p2PolygonShape : public p2Shape
{
	
};

#endif