/**
\file Connection.h

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

#include "openjaus/model/Connection.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Connection::Connection() :
	eventId(0),
	localId(0),
	requestId(0),
	rate_Hz(1.0),
	type(PERIODIC),
	sequenceNumber(0),
	active(false),
	responseId(0),
	lastRequestType(CREATE),
	timeout(),
	address(),
	query(NULL),
	timer(NULL)
{
}
// End of user code

// Start of user code for default destructor:
Connection::~Connection()
{
}
// End of user code

uint8_t Connection::getEventId() const
{
	// Start of user code for accessor getEventId:
	
	return eventId;
	// End of user code
}

bool Connection::setEventId(uint8_t eventId)
{
	// Start of user code for accessor setEventId:
	this->eventId = eventId;
	return true;
	// End of user code
}


uint32_t Connection::getLocalId() const
{
	// Start of user code for accessor getLocalId:
	
	return localId;
	// End of user code
}

bool Connection::setLocalId(uint32_t localId)
{
	// Start of user code for accessor setLocalId:
	this->localId = localId;
	return true;
	// End of user code
}


uint8_t Connection::getRequestId() const
{
	// Start of user code for accessor getRequestId:
	
	return requestId;
	// End of user code
}

bool Connection::setRequestId(uint8_t requestId)
{
	// Start of user code for accessor setRequestId:
	this->requestId = requestId;
	return true;
	// End of user code
}


double Connection::getRate_Hz() const
{
	// Start of user code for accessor getRate_Hz:
	
	return rate_Hz;
	// End of user code
}

bool Connection::setRate_Hz(double rate_Hz)
{
	// Start of user code for accessor setRate_Hz:
	this->rate_Hz = rate_Hz;
	return true;
	// End of user code
}


ConnectionType Connection::getType() const
{
	// Start of user code for accessor getType:
	
	return type;
	// End of user code
}

bool Connection::setType(ConnectionType type)
{
	// Start of user code for accessor setType:
	this->type = type;
	return true;
	// End of user code
}


uint8_t Connection::getSequenceNumber() const
{
	// Start of user code for accessor getSequenceNumber:
	
	return sequenceNumber;
	// End of user code
}

bool Connection::setSequenceNumber(uint8_t sequenceNumber)
{
	// Start of user code for accessor setSequenceNumber:
	this->sequenceNumber = sequenceNumber;
	return true;
	// End of user code
}


bool Connection::isActive() const
{
	// Start of user code for accessor getActive:
	
	return active;
	// End of user code
}

bool Connection::setActive(bool active)
{
	// Start of user code for accessor setActive:
	this->active = active;
	return true;
	// End of user code
}


uint16_t Connection::getResponseId() const
{
	// Start of user code for accessor getResponseId:
	
	return responseId;
	// End of user code
}

bool Connection::setResponseId(uint16_t responseId)
{
	// Start of user code for accessor setResponseId:
	this->responseId = responseId;
	return true;
	// End of user code
}


ConnectionRequestType Connection::getLastRequestType() const
{
	// Start of user code for accessor getLastRequestType:
	
	return lastRequestType;
	// End of user code
}

bool Connection::setLastRequestType(ConnectionRequestType lastRequestType)
{
	// Start of user code for accessor setLastRequestType:
	this->lastRequestType = lastRequestType;
	return true;
	// End of user code
}


const system::Time& Connection::getTimeout() const
{
	// Start of user code for accessor getTimeout:
	
	return timeout;
	// End of user code
}

bool Connection::setTimeout(const system::Time& timeout)
{
	// Start of user code for accessor setTimeout:
	this->timeout = timeout;
	return true;
	// End of user code
}


const transport::Address& Connection::getAddress() const
{
	// Start of user code for accessor getAddress:
	
	return address;
	// End of user code
}

bool Connection::setAddress(const transport::Address& address)
{
	// Start of user code for accessor setAddress:
	this->address = address;
	return true;
	// End of user code
}


Message* Connection::getQuery() const
{
	// Start of user code for accessor getQuery:
	
	return query;
	// End of user code
}

bool Connection::setQuery(Message* query)
{
	// Start of user code for accessor setQuery:
	this->query = query;
	return true;
	// End of user code
}


system::Timer* Connection::getTimer() const
{
	// Start of user code for accessor getTimer:
	
	return timer;
	// End of user code
}

bool Connection::setTimer(system::Timer* timer)
{
	// Start of user code for accessor setTimer:
	this->timer = timer;
	return true;
	// End of user code
}



// Class Methods


std::string Connection::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Connection& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace model
} // namespace openjaus

