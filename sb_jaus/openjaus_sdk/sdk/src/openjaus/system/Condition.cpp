/**
\file Condition.h

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

#include "openjaus/system/Condition.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Condition::Condition() :
	numberOfWaitingThreads(0)
{
#ifdef WIN32
	InitializeCriticalSection(&mutex);
#if(WINVER >= 0x0600)
	InitializeConditionVariable(&condition);
#else
	condition = CreateEvent( 
            NULL,   // default security attributes
            TRUE,  // auto-reset event object
            FALSE,  // initial state is nonsignaled
            NULL);  // unnamed object
#endif

#else
	int result = pthread_cond_init(&condition, NULL);
	if(result)
	{
		THROW_EXCEPTION("Could not create condition: Error = " << result);
	}

	result = pthread_mutex_init(&mutex, NULL);
	if(result)
	{
		THROW_EXCEPTION("Could not create condition mutex: Error = " << result);
	}

#endif
}
// End of user code

// Start of user code for default destructor:
Condition::~Condition()
{
	signalAll();
#ifdef WIN32
	DeleteCriticalSection(&mutex);
#if(WINVER >= 0x0600)
#else
	CloseHandle(condition);
#endif
#else
	int result = pthread_cond_destroy(&condition);
	if(result)
	{
		THROW_EXCEPTION("Could not destroy condition: Error = " << result);
	}

	result = pthread_mutex_destroy(&mutex);
	if(result)
	{
		THROW_EXCEPTION("Could not destroy condition mutex: Error = " << result);
	}
#endif
}
// End of user code


// Class Methods
void Condition::signal()
{
	// Start of user code for method signal:
#ifdef WIN32

#if(WINVER >= 0x0600)
	WakeConditionVariable(&condition);
#else
	PulseEvent(condition);
#endif

#else
	int result = pthread_cond_signal(&condition);
	if(result)
	{
		THROW_EXCEPTION("Could not signal condition: Error = " << result);
	}
#endif
	// End of user code
}


void Condition::signalAll()
{
	// Start of user code for method signalAll:
#ifdef WIN32
#if(WINVER >= 0x0600)
	WakeAllConditionVariable(&condition);
#else
	SetEvent(condition);
#endif
#else
	int result = pthread_cond_broadcast(&condition);
	if(result)
	{
		THROW_EXCEPTION("Could not signal all condition: Error = " << result);
	}
#endif
	// End of user code
}


bool Condition::timedWait(int milliseconds)
{
	// Start of user code for method timedWait:
#ifdef WIN32
	EnterCriticalSection(&mutex);
#if(WINVER >= 0x0600)
	bool woken = SleepConditionVariableCS(&condition, &mutex, milliseconds);
	LeaveCriticalSection(&mutex);
	return !woken; // Returns true if timed out, false if woken by another thread
#else
	numberOfWaitingThreads++;
	DWORD woken = WaitForSingleObject(condition, milliseconds);
	numberOfWaitingThreads--;
	if(!numberOfWaitingThreads)
	{
		ResetEvent(condition);
	}
	LeaveCriticalSection(&mutex);
	return woken != WAIT_OBJECT_0; // Returns true if timed out, false if woken by another thread
#endif
#else
	struct timespec timeout;
	double timeout_sec = system::Time::getTime().inSec() + milliseconds / 1000.0;
	timeout.tv_sec = static_cast<__time_t>(timeout_sec);
	timeout.tv_nsec = static_cast<long int>(1e9 *(timeout_sec - timeout.tv_sec));

	int result = pthread_mutex_lock(&mutex);
	if(result)
	{
		THROW_EXCEPTION("Could not lock condition mutex: Error = " << result);
	}

	result = pthread_cond_timedwait(&condition, &mutex, &timeout);
	if(result && result != ETIMEDOUT)
	{
		THROW_EXCEPTION("Could not wait on condition: Error = " << result);
	}

	pthread_mutex_unlock(&mutex);

	return (result == ETIMEDOUT);
#endif
	// End of user code
}




std::string Condition::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Condition& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace system
} // namespace openjaus

