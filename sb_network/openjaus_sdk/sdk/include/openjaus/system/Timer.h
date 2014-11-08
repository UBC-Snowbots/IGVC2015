/**
\file Timer.h

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of the OpenJAUS Software Development Kit (SDK). This 
software is distributed under one of two licenses, the OpenJAUS SDK 
Commercial End User License Agreement or the OpenJAUS SDK Non-Commercial 
End User License Agreement. The appropriate licensing details were included 
in with your developer credentials and software download. See the LICENSE 
file included with this software for full details.
 
THIS SOFTWARE IS PROVIDED BY THE LICENSOR (OPENJAUS LCC) "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LICENSOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. THE LICENSOR DOES NOT WARRANT THAT THE LICENSED SOFTWARE WILL MEET
LICENSEE'S REQUIREMENTS OR THAT THE OPERATION OF THE LICENSED SOFTWARE
WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ERRORS IN THE LICENSED
SOFTWARE WILL BE CORRECTED.

\ Software History
- [2011-08-23] - Added AS6057: Manipulators
- [2011-08-01] - Added AS6060: Environment Sensing
- [2011-06-16] - First Release 

*/
#ifndef SYSTEM_TIMER_H
#define SYSTEM_TIMER_H

#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
#include <time.h>
#include <signal.h>
#else
#error "No Thread implementation defined for this platform."
#endif

namespace openjaus
{
namespace system
{
	class Timer;
}
}

template <class TimerClass, void (TimerClass::*TimerFunc)(openjaus::system::Timer *)>
struct TimerProxy
{
	static void TimerProxyFunction(void *data, openjaus::system::Timer *timer)
	{
		(static_cast<TimerClass*>(data)->*TimerFunc)(timer);
	}
};

#define TIMER_METHOD(Class, TimerFunc) TimerProxy<Class, &Class::TimerFunc>::TimerProxyFunction
// End of user code

namespace openjaus
{
namespace system
{

/// \class Timer Timer.h

class OPENJAUS_EXPORT Timer 
{
public:
	Timer(); 
	virtual ~Timer();
	// Start of user code for additional constructors
	Timer(void (*functionPointer)(void*, Timer *), void *data);
	// End of user code
	/// Operation reset.
	/// \param milliseconds 
	 void reset(int milliseconds);

	/// Operation setInterval.
	/// \param milliseconds 
	 void setInterval(int milliseconds);

	/// Operation stop.
	 void stop();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Timer& object);

protected:

// Start of user code for additional member data
#ifdef WIN32
    static HANDLE timerQueue;
	static int timerCount;
	HANDLE hTimer;
	static void CALLBACK Timer::timerEventFunction(void *param, bool timerOrWaitFired);
#else
	timer_t timerId;
	struct sigevent event;
	struct itimerspec intervalSpec;
	static void timerEventFunction(sigval_t);
#endif
	bool running;
	bool stopped;
	bool created;

	void *data;
	void (*timerFunction)(void *, Timer *);

public:
	void setTimerFunction(void (*functionPointer)(void*, Timer *), void *data);
// End of user code

}; // class Timer

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_TIMER_H

