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
#ifndef MODEL_CONNECTION_H
#define MODEL_CONNECTION_H

#include "openjaus/model/ConnectionType.h"
#include "openjaus/system/Time.h"
#include "openjaus/transport/Address.h"
#include "openjaus/model/Message.h"
#include "openjaus/system/Timer.h"
#include "openjaus/model/ConnectionRequestType.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
class Message;

/// \class Connection Connection.h

class OPENJAUS_EXPORT Connection 
{
public:
	Connection(); 
	virtual ~Connection();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of eventId.
	uint8_t getEventId() const;

	/// Accessor to set value of eventId.
	/// \param eventId The value of the new eventId.
	bool setEventId(uint8_t eventId);

	/// Accessor to get the value of localId.
	uint32_t getLocalId() const;

	/// Accessor to set value of localId.
	/// \param localId The value of the new localId.
	bool setLocalId(uint32_t localId);

	/// Accessor to get the value of requestId.
	uint8_t getRequestId() const;

	/// Accessor to set value of requestId.
	/// \param requestId The value of the new requestId.
	bool setRequestId(uint8_t requestId);

	/// Accessor to get the value of rate_Hz.
	double getRate_Hz() const;

	/// Accessor to set value of rate_Hz.
	/// \param rate_Hz The value of the new rate_Hz.
	bool setRate_Hz(double rate_Hz);

	/// Accessor to get the value of type.
	ConnectionType getType() const;

	/// Accessor to set value of type.
	/// \param type The value of the new type.
	bool setType(ConnectionType type);

	/// Accessor to get the value of sequenceNumber.
	uint8_t getSequenceNumber() const;

	/// Accessor to set value of sequenceNumber.
	/// \param sequenceNumber The value of the new sequenceNumber.
	bool setSequenceNumber(uint8_t sequenceNumber);

	/// Accessor to get the value of active.
	bool isActive() const;

	/// Accessor to set value of active.
	/// \param active The value of the new active.
	bool setActive(bool active);

	/// Accessor to get the value of responseId.
	uint16_t getResponseId() const;

	/// Accessor to set value of responseId.
	/// \param responseId The value of the new responseId.
	bool setResponseId(uint16_t responseId);

	/// Accessor to get the value of lastRequestType.
	ConnectionRequestType getLastRequestType() const;

	/// Accessor to set value of lastRequestType.
	/// \param lastRequestType The value of the new lastRequestType.
	bool setLastRequestType(ConnectionRequestType lastRequestType);

	/// Accessor to get the value of timeout.
	const system::Time& getTimeout() const;

	/// Accessor to set value of timeout.
	/// \param timeout The value of the new timeout.
	bool setTimeout(const system::Time& timeout);

	/// Accessor to get the value of address.
	const transport::Address& getAddress() const;

	/// Accessor to set value of address.
	/// \param address The value of the new address.
	bool setAddress(const transport::Address& address);

	/// Accessor to get the value of query.
	Message* getQuery() const;

	/// Accessor to set value of query.
	/// \param query The value of the new query.
	bool setQuery(Message* query);

	/// Accessor to get the value of timer.
	system::Timer* getTimer() const;

	/// Accessor to set value of timer.
	/// \param timer The value of the new timer.
	bool setTimer(system::Timer* timer);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Connection& object);

protected:
	// Member attributes & references
	uint8_t eventId;
	uint32_t localId;
	uint8_t requestId;
	double rate_Hz;
	ConnectionType type;
	uint8_t sequenceNumber;
	bool active;
	uint16_t responseId;
	ConnectionRequestType lastRequestType;
	system::Time timeout;
	transport::Address address;
	Message *query;
	system::Timer *timer;

// Start of user code for additional member data
// End of user code

}; // class Connection

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_CONNECTION_H

