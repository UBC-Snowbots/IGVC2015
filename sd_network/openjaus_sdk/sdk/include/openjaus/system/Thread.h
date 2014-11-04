/**
\file Thread.h

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
#ifndef SYSTEM_THREAD_H
#define SYSTEM_THREAD_H

#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <process.h>
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
#include <pthread.h>
#endif

template <class ThreadClass, void *(ThreadClass::*ThreadFunc)(void)>
struct ThreadProxy
{
	static void* ThreadProxyFunction(void *data)
	{
		return (static_cast<ThreadClass*>(data)->*ThreadFunc)();
	}
};

#define THREAD_METHOD(Class, ThreadFunc) ThreadProxy<Class, &Class::ThreadFunc>::ThreadProxyFunction

// End of user code

namespace openjaus
{
namespace system
{

/// \class Thread Thread.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Thread 
{
public:
	Thread(); 
	virtual ~Thread();
	// Start of user code for additional constructors
	Thread(void * (*functionPointer)(void*), void *data);
	// End of user code
	/// Accessor to get the value of running.
	bool isRunning() const;

	/// Accessor to set value of running.
	/// \param running The value of the new running.
	bool setRunning(bool running);

	/// Operation create.
	 void create();

	/// Operation join.
	 void join();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Thread& object);

protected:
	// Member attributes & references
	bool running;

// Start of user code for additional member data
#ifdef WIN32
	HANDLE thread;
	static unsigned int __stdcall start(void *threadPtr);
#else
	pthread_t thread;
	static void* start(void *classPointer);
#endif

	void *data;
	void *(*threadFunction)(void *);
	virtual void* run()
	{
		return NULL;
	};

public:
	void setThreadFunction(void * (*functionPointer)(void*), void *data);
// End of user code

}; // class Thread

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_THREAD_H

