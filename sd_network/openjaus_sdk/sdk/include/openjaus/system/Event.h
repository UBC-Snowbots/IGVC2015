/**
\file Event.h

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
#ifndef SYSTEM_EVENT_H
#define SYSTEM_EVENT_H

#include "openjaus/system/Time.h"
#include "openjaus/system/Accessable.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace system
{
class Time;

/// \class Event Event.h

class OPENJAUS_EXPORT Event : public Accessable
{
public:
	Event(); 
	virtual ~Event();
	// Start of user code for additional constructors
	Event(std::string description);
	// End of user code
	/// Accessor to get the value of description.
	std::string getDescription() const;

	/// Accessor to set value of description.
	/// \param description The value of the new description.
	bool setDescription(std::string description);

	/// Accessor to get the value of maxLoggingHz.
	float getMaxLoggingHz() const;

	/// Accessor to set value of maxLoggingHz.
	/// \param maxLoggingHz The value of the new maxLoggingHz.
	bool setMaxLoggingHz(float maxLoggingHz);

	/// Accessor to get the value of save.
	bool isSave() const;

	/// Accessor to set value of save.
	/// \param save The value of the new save.
	bool setSave(bool save);

	/// Accessor to get the value of nameSpace.
	std::string getNameSpace() const;

	/// Accessor to set value of nameSpace.
	/// \param nameSpace The value of the new nameSpace.
	bool setNameSpace(std::string nameSpace);

	/// Accessor to get the value of time.
	const Time& getTime() const;

	/// Accessor to set value of time.
	/// \param time The value of the new time.
	bool setTime(const Time& time);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Event& object);

protected:
	// Member attributes & references
	std::string description;
	float maxLoggingHz;
	bool save;
	std::string nameSpace;
	Time time;

// Start of user code for additional member data
// End of user code

}; // class Event

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_EVENT_H

