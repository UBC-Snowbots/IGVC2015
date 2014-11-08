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
#ifndef SYSTEM_CONDITION_H
#define SYSTEM_CONDITION_H

#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
#include <pthread.h>
#include <errno.h>
#else
	#error "No Condition implementation defined for this platform."
#endif
// End of user code

namespace openjaus
{
namespace system
{

/// \class Condition Condition.h

class OPENJAUS_EXPORT Condition 
{
public:
	Condition(); 
	virtual ~Condition();
	// Start of user code for additional constructors
	// End of user code
	/// Operation signal.
	 void signal();

	/// Operation signalAll.
	 void signalAll();

	/// Operation timedWait.
	/// \param milliseconds 
	 bool timedWait(int milliseconds);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Condition& object);

protected:

// Start of user code for additional member data
	int numberOfWaitingThreads;
#ifdef WIN32
	CRITICAL_SECTION   mutex;
#if(WINVER >= 0x0600)
	CONDITION_VARIABLE condition;
#else
	HANDLE condition;
#endif

#else
	pthread_cond_t condition;
	pthread_mutex_t mutex;
#endif
public:
	static const int DEFAULT_WAIT_MSEC = 500;
// End of user code

}; // class Condition

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_CONDITION_H

