/**
\file Trigger.h

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

#include "openjaus/model/Trigger.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Trigger::Trigger()
{
}
// End of user code

// Start of user code for default destructor:
Trigger::~Trigger()
{
}
// End of user code

std::string Trigger::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool Trigger::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


double Trigger::getTimeStamp() const
{
	// Start of user code for accessor getTimeStamp:
	
	return timeStamp;
	// End of user code
}

bool Trigger::setTimeStamp(double timeStamp)
{
	// Start of user code for accessor setTimeStamp:
	this->timeStamp = timeStamp;
	return true;
	// End of user code
}


uint16_t Trigger::getId() const
{
	// Start of user code for accessor getId:
	
	return id;
	// End of user code
}

bool Trigger::setId(uint16_t id)
{
	// Start of user code for accessor setId:
	this->id = id;
	return true;
	// End of user code
}


std::string Trigger::getMessageID() const
{
	// Start of user code for accessor getMessageID:
	
	return MessageID;
	// End of user code
}

bool Trigger::setMessageID(std::string MessageID)
{
	// Start of user code for accessor setMessageID:
	this->MessageID = MessageID;
	return true;
	// End of user code
}


const std::vector< fields::Field* >& Trigger::getFields() const
{
	// Start of user code for accessor getFields:
	
	return fields;
	// End of user code
}

bool Trigger::setFields(const fields::Field& fields)
{
	// Start of user code for accessor setFields:
	return true;
	// End of user code
}


TriggerQueue* Trigger::getChangedQueue() const
{
	// Start of user code for accessor getChangedQueue:
	
	return changedQueue;
	// End of user code
}

bool Trigger::setChangedQueue(TriggerQueue* changedQueue)
{
	// Start of user code for accessor setChangedQueue:
	this->changedQueue = changedQueue;
	return true;
	// End of user code
}



// Class Methods


std::string Trigger::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Trigger& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace model
} // namespace openjaus

