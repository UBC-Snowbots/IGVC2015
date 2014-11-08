/**
\file Wrapper.h

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
#ifndef TRANSPORT_WRAPPER_H
#define TRANSPORT_WRAPPER_H

#include "openjaus/transport/AckNakType.h"
#include "openjaus/transport/Address.h"
#include "openjaus/system/Transportable.h"
#include "openjaus/transport/TransportData.h"
#include "openjaus/transport/WrapperType.h"
#include "openjaus/transport/Priority.h"
#include "openjaus/transport/BroadcastType.h"
#include "openjaus/transport/LargeMessageType.h"
#include "openjaus/system/Prioritized.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

#include "openjaus/system/Buffer.h"
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{
class Address;
class TransportData;

/// \class Wrapper Wrapper.h
/// \brief Application layer wrapper.
/// This is an application layer transport wrapper. It is meant to contain all of the information necessary for the application developer to specify exactly how a message will be transported.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Wrapper : public system::Transportable, public system::Prioritized
{
public:
	Wrapper(); 
	virtual ~Wrapper();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of ackNak.
	AckNakType getAckNak() const;

	/// Accessor to set value of ackNak.
	/// \param ackNak The value of the new ackNak.
	bool setAckNak(AckNakType ackNak);

	/// Accessor to get the value of compressible.
	bool isCompressible() const;

	/// Accessor to set value of compressible.
	/// \param compressible The value of the new compressible.
	bool setCompressible(bool compressible);

	/// Accessor to get the value of mustArrive.
	bool isMustArrive() const;

	/// Accessor to set value of mustArrive.
	/// \param mustArrive The value of the new mustArrive.
	bool setMustArrive(bool mustArrive);

	/// Accessor to get the value of sequenceNumber.
	uint16_t getSequenceNumber() const;

	/// Accessor to set value of sequenceNumber.
	/// \param sequenceNumber The value of the new sequenceNumber.
	bool setSequenceNumber(uint16_t sequenceNumber);

	/// Accessor to get the value of type.
	WrapperType getType() const;

	/// Accessor to set value of type.
	/// \param type The value of the new type.
	bool setType(WrapperType type);

	/// Accessor to get the value of priority.
	Priority getPriority() const;

	/// Accessor to set value of priority.
	/// \param priority The value of the new priority.
	bool setPriority(Priority priority);

	/// Accessor to get the value of broadcastFlag.
	BroadcastType getBroadcastFlag() const;

	/// Accessor to set value of broadcastFlag.
	/// \param broadcastFlag The value of the new broadcastFlag.
	bool setBroadcastFlag(BroadcastType broadcastFlag);

	/// Accessor to get the value of largeMessageFlag.
	LargeMessageType getLargeMessageFlag() const;

	/// Accessor to set value of largeMessageFlag.
	/// \param largeMessageFlag The value of the new largeMessageFlag.
	bool setLargeMessageFlag(LargeMessageType largeMessageFlag);

	/// Accessor to get the value of destination.
	const Address& getDestination() const;

	/// Accessor to set value of destination.
	/// \param destination The value of the new destination.
	bool setDestination(const Address& destination);

	/// Accessor to get the value of payload.
	system::Transportable* getPayload() const;

	/// Accessor to set value of payload.
	/// \param payload The value of the new payload.
	bool setPayload(system::Transportable* payload);

	/// Accessor to get the value of source.
	const Address& getSource() const;

	/// Accessor to set value of source.
	/// \param source The value of the new source.
	bool setSource(const Address& source);

	/// Accessor to get the value of transportData.
	TransportData* getTransportData() const;

	/// Accessor to set value of transportData.
	/// \param transportData The value of the new transportData.
	bool setTransportData(TransportData* transportData);

	// Inherited pure virtuals from Transportable that need to be implemented
	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);	
	virtual int length();	

	// Inherited pure virtuals from Prioritized that need to be implemented
	virtual int prioritizedValue();	

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Wrapper& object);

protected:
	// Member attributes & references
	AckNakType ackNak;
	bool compressible;
	bool mustArrive;
	uint16_t sequenceNumber;
	WrapperType type;
	Priority priority;
	BroadcastType broadcastFlag;
	LargeMessageType largeMessageFlag;
	Address destination;
	system::Transportable *payload;
	Address source;
	TransportData *transportData;

// Start of user code for additional member data
// End of user code

}; // class Wrapper

// Start of user code for inline functions
// End of user code



} // namespace transport
} // namespace openjaus

#endif // TRANSPORT_WRAPPER_H

