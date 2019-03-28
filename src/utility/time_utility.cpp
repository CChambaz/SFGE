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

#include <utility/time_utility.h>

namespace sfge
{

Timer::Timer(float time, float period):
	m_Time(time), m_Period(period)
{
}

void Timer::Update(float dt)
{
	if(m_Time>0.0f)
	{
		m_Time -= dt;
	}
}
bool Timer::IsOver()
{
	return !(m_Time > 0.0f);
}

void Timer::Reset()
{
	if(m_Time < 0.0f)
	{
		m_Time = m_Period + m_Time;
	}
	else
	{
		m_Time = m_Period;
	}
}

float Timer::GetCurrentRatio()
{
	return GetTimeFromStart() / m_Period;
}

float Timer::GetTimeFromStart() const
{
	return m_Period - m_Time;
}

float Timer::GetPeriod() const
{
	return m_Period;
}

void Timer::SetPeriod(float period)
{
	m_Period = period;
}

float Timer::GetTime() const
{
	return m_Time;
}

void Timer::SetTime(float time)
{
	m_Time = time;
}
}


