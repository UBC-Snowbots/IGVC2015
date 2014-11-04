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

#include "openjaus/system/Timer.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
#include <string.h>
#include <errno.h>
#include "openjaus/system/Logger.h"
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Timer::Timer() :
	running(false),
	stopped(false),
	created(false),
	data(NULL),
	timerFunction(NULL)
{
#ifdef WIN32
	if(!timerCount)
	{
		 timerQueue = CreateTimerQueue();
	}
	timerCount++;
	hTimer = NULL;
#endif
}

Timer::Timer(void (*functionPointer)(void *, Timer *), void *data) :
	running(false),
	stopped(false),
	created(true),
	data(data),
	timerFunction(functionPointer)
{
#ifdef WIN32
	if(!timerCount)
	{
		 timerQueue = CreateTimerQueue();
	}
	timerCount++;
	hTimer = NULL;
#else
	timerId = 0;

	memset(&event, 0, sizeof(struct sigevent));
	event.sigev_notify = SIGEV_THREAD;
	event.sigev_signo = 0;
	event.sigev_value.sival_ptr = this;
	event.sigev_notify_function = timerEventFunction;
	event.sigev_notify_attributes = NULL;
	if(timer_create(CLOCK_REALTIME, &event, &timerId))
	{
		THROW_EXCEPTION("Could not create timer: " << strerror(errno));
	}

	LOG_DEBUG("Timer Created: " << timerId);
#endif
}
// End of user code

// Start of user code for default destructor:
Timer::~Timer()
{
	while(running)
	{
		stopped = true;
		Time::sleep(10); // TODO: Need to define cross platform sleep
	}

	if(created)
	{
#ifdef WIN32
#else
		timer_delete(timerId);
#endif
	}

#ifdef WIN32
	timerCount--;
	if(!timerCount)
	{
		DeleteTimerQueue(timerQueue);
	}
#endif
}
// End of user code


// Class Methods
void Timer::reset(int milliseconds)
{
	// Start of user code for method reset:
#ifdef WIN32
	if(hTimer)
	{
		DeleteTimerQueueTimer(hTimer, timerQueue, NULL);
		hTimer = NULL;
	}
	CreateTimerQueueTimer(&hTimer, timerQueue, (WAITORTIMERCALLBACK)timerEventFunction, this, milliseconds, 0, 0);
#else
	running = true;

	intervalSpec.it_value.tv_sec = milliseconds / 1000;
	intervalSpec.it_value.tv_nsec = 1000000 * (milliseconds % 1000);
	intervalSpec.it_interval.tv_sec = 0;
	intervalSpec.it_interval.tv_nsec = 0;

	timer_settime(timerId, 0, &intervalSpec, NULL);
#endif
	// End of user code
}


void Timer::setInterval(int milliseconds)
{
	// Start of user code for method setInterval:
#ifdef WIN32
	if(hTimer)
	{
		DeleteTimerQueueTimer(hTimer, timerQueue, NULL);
		hTimer = NULL;
	}
	CreateTimerQueueTimer(&hTimer, timerQueue, (WAITORTIMERCALLBACK)timerEventFunction, this, milliseconds, milliseconds, 0);
#else
	running = true;

	intervalSpec.it_value.tv_sec = milliseconds / 1000;
	intervalSpec.it_value.tv_nsec = 1000000 * (milliseconds % 1000);
	intervalSpec.it_interval.tv_sec = intervalSpec.it_value.tv_sec;
	intervalSpec.it_interval.tv_nsec = intervalSpec.it_value.tv_nsec;

	timer_settime(timerId, 0, &intervalSpec, NULL);
#endif
	// End of user code
}


void Timer::stop()
{
	// Start of user code for method stop:
	stopped = true;
	running = false;

#ifdef WIN32
	//if(hTimer)
	//{
	//	DeleteTimerQueueTimer(hTimer, timerQueue, NULL);
	//	hTimer = NULL;
	//}
#else
	intervalSpec.it_value.tv_sec = 0;
	intervalSpec.it_value.tv_nsec = 0;
	intervalSpec.it_interval.tv_sec = 0;
	intervalSpec.it_interval.tv_nsec = 0;

	timer_settime(timerId, 0, &intervalSpec, NULL);
#endif
	// End of user code
}




std::string Timer::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Timer& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
#ifdef WIN32
HANDLE Timer::timerQueue = NULL;
int Timer::timerCount = 0;

void Timer::timerEventFunction(void *param, bool timerOrWaitFired)
{
	Timer *timer = static_cast<Timer*>(param);
	if(timer->stopped)
	{
		timer->stop();
		timer->running = false;
		return;
	}
	if(timer->timerFunction)
	{
		timer->timerFunction(timer->data, timer);
	}
}

#else

void Timer::timerEventFunction(sigval_t sigval)
{
	Timer *timer = static_cast<Timer*>(sigval.sival_ptr);
	if(timer->stopped)
	{
		timer->stop();
		timer->running = false;
		return;
	}
	timer->timerFunction(timer->data, timer);
}

#endif

void Timer::setTimerFunction(void (*functionPointer)(void *, Timer *), void *data)
{
	this->data = data;
	timerFunction = functionPointer;
}
// End of user code

} // namespace system
} // namespace openjaus

