/**
\file Logger.h

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
#ifndef SYSTEM_LOGGER_H
#define SYSTEM_LOGGER_H

#include "openjaus/system/Event.h"
#include "openjaus/system/Thread.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
//#include "openjaus/system/Application.h"
#include <sstream>

#ifdef DEBUG
#define LOG_DEBUG(msg)	LOG_VERBOSE(__FILE__,msg)
#else
#define LOG_DEBUG(msg)
#endif

#define LOG(msg)	LOG_VERBOSE(__FILE__,msg)

#define LOG_VERBOSE(logSpace, msg)			\
{											\
	using namespace openjaus::system;		\
	std::ostringstream oss;					\
	oss <<  msg;							\
	openjaus::system::Event evnt;			\
	evnt.setNameSpace(logSpace);			\
	evnt.setDescription(oss.str());			\
	evnt.setTime(Time::getTime());			\
	Logger::log(evnt);						\
}

// End of user code

namespace openjaus
{
namespace system
{
class Event;

/// \class Logger Logger.h

class OPENJAUS_EXPORT Logger : public Thread
{
public:
	Logger(); 
	virtual ~Logger();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of file.
	std::string getFile() const;

	/// Accessor to set value of file.
	/// \param file The value of the new file.
	bool setFile(std::string file);

	/// Operation log.
	/// \param event 
	static bool log(Event event);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Logger& object);

protected:
	// Member attributes & references
	std::string file;

// Start of user code for additional member data
	virtual void* run()
	{
		return NULL;
	};
// End of user code

}; // class Logger

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_LOGGER_H

