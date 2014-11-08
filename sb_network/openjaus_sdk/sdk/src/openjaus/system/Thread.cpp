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

#include "openjaus/system/Thread.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Thread::Thread() :
		running(false),
		thread(0),
		data(NULL),
		threadFunction(NULL)
{
}

Thread::Thread(void * (*functionPointer)(void*), void *data) :
		running(false),
		thread(0),
		data(data),
		threadFunction(functionPointer)
{
}

// End of user code

// Start of user code for default destructor:
Thread::~Thread()
{
}
// End of user code

bool Thread::isRunning() const
{
	// Start of user code for accessor getRunning:

	return running;
	// End of user code
}

bool Thread::setRunning(bool running)
{
	// Start of user code for accessor setRunning:
	this->running = running;
	
	return true;
	// End of user code
}



// Class Methods
void Thread::create()
{
	// Start of user code for method create:
	setRunning(true);

#ifdef WIN32
	thread = (HANDLE)_beginthreadex(NULL, 0, start, this, 0, NULL);
	if (thread == 0)
	{
		// TODO: Throw error
	}
#else
	int result =pthread_create(&thread, NULL, start, this);
	if(result)
	{
		THROW_EXCEPTION("Could not create thread: Error = " << result);
	}
#endif
	// End of user code
}


void Thread::join()
{
	// Start of user code for method join:
	setRunning(false);

#ifdef WIN32
	WaitForSingleObject(thread, INFINITE);
	CloseHandle(thread);
#else
	int result = pthread_join(thread, NULL);
	if(result)
	{
		THROW_EXCEPTION("Could not join thread: Error = " << result);
	}
#endif
	// End of user code
}




std::string Thread::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Thread& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
#ifdef WIN32

unsigned int __stdcall Thread::start(void *threadPtr)
{
	Thread* thread = static_cast<Thread*>(threadPtr);
	if(thread->threadFunction)
	{
		thread->threadFunction(thread->data);
	}
	else
	{
		thread->run();
	}
	return 0;
}

#else

void* Thread::start(void* threadPtr)
{
	Thread* thread = static_cast<Thread*>(threadPtr);
	if(thread->threadFunction)
	{
		return thread->threadFunction(thread->data);
	}
	else
	{
		return thread->run();
	}
}

#endif

void Thread::setThreadFunction(void * (*functionPointer)(void*), void *data)
{
	if(!running)
	{
		this->data = data;
		this->threadFunction = functionPointer;
	}
}
// End of user code

} // namespace system
} // namespace openjaus

