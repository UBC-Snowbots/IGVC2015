
/**
\file CreateEvent.h

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

#ifndef CREATEEVENT_H
#define CREATEEVENT_H

#include <openjaus.h>

#include "openjaus/core/Triggers/Fields/EventTypeEnumeration.h"
#include "openjaus/core/Triggers/Fields/PeriodicRateScaledInteger.h"

namespace openjaus
{
namespace core
{

/// \class CreateEvent CreateEvent.h
/// \brief CreateEvent Message Implementation.
/// This message is used to set up a command event. Field 1 is a local request ID that the event provider returns in the
/// Confirm or Reject message. Field 2 is the maximum allowed execution time; any command not completed within its
/// specified duration is considered a failure.  Field 3 contains the size of the Command message that is to specify the
/// command to be executed.  Field 4 contains the encoded command message (including its two byte header).
class OPENJAUS_EXPORT CreateEvent : public model::Message
{
public:
	CreateEvent();
	CreateEvent(model::Message *message);
	~CreateEvent();
	
	static const uint16_t ID = 0x01F0;

	/// \brief Pack this message to the given openjaus::system::Buffer. 
	/// \copybrief
	/// \param[out] dst - The destination openjaus::system::Buffer to which this message will be packed.
	/// \return The number of bytes packed into the destination buffer
	virtual int to(system::Buffer *dst);	

	/// \brief Unpack this message from the given openjaus::system::Buffer.
	/// \copybrief
	/// \param[in] src - The source openjaus::system::Buffer from which this message will be unpacked.
	/// \return The number of bytes unpacked from the source buffer
	virtual int from(system::Buffer *src);

	/// \brief Get the number of bytes this message would occupy in a serialized buffer. 
	/// \copybrief
	/// \return The number of bytes this message would occupy in a buffer
	virtual int length();
	
	/// \brief Used to serialize the runtime state of the message to an XML format. 
	/// \copybrief
	/// \param[in] level - Used to determine how many tabs are inserted per line for pretty formating. 
	/// \return The serialized XML string
	std::string toXml(unsigned char level=0) const;



	uint8_t getRequestID(void);
	void setRequestID(uint8_t value);

	EventTypeEnumeration::EventTypeEnum getEventType(void);
	void setEventType(EventTypeEnumeration::EventTypeEnum value);

	double getRequestedPeriodicRate_Hz(void);
	void setRequestedPeriodicRate_Hz(double value);

	model::Message *getQueryMessage(void);
	void setQueryMessage(model::Message* message);

private:
	model::fields::UnsignedByte requestID;
	EventTypeEnumeration eventType;
	PeriodicRateScaledInteger requestedPeriodicRate_Hz;
	model::fields::MessageField queryMessage;

};

} // namespace core
} // namespace openjaus

#endif // CREATEEVENT_H


