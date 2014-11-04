
/**
\file RejectEventRequest.h

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

#ifndef REJECTEVENTREQUEST_H
#define REJECTEVENTREQUEST_H

#include <openjaus.h>

#include "openjaus/core/Triggers/Fields/ResponseCodeEnumeration.h"

namespace openjaus
{
namespace core
{

/// \class RejectEventRequest RejectEventRequest.h
/// \brief RejectEventRequest Message Implementation.
/// The Reject Event Request message is used to reject an Event creation, update or cancellation.  Field 2 represents
/// the Request ID from the Create, Update, or Cancel message that initiated this message.  The Request ID’s scope is
/// local to the requesting client only.
class OPENJAUS_EXPORT RejectEventRequest : public model::Message
{
public:
	RejectEventRequest();
	RejectEventRequest(model::Message *message);
	~RejectEventRequest();
	
	static const uint16_t ID = 0x01F4;

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

	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isErrorMessageEnabled(void) const;
	void enableErrorMessage(void);
	void disableErrorMessage(void);



	int8_t getRequestID(void);
	void setRequestID(int8_t value);

	ResponseCodeEnumeration::ResponseCodeEnum getResponseCode(void);
	void setResponseCode(ResponseCodeEnumeration::ResponseCodeEnum value);

	std::string getErrorMessage(void);
	void setErrorMessage(std::string value);

private:
	model::fields::Byte requestID;
	ResponseCodeEnumeration responseCode;
	model::fields::FixedLengthString errorMessage;

	uint8_t presenceVector;
	enum pvEnum {ERRORMESSAGE = 0};
};

} // namespace core
} // namespace openjaus

#endif // REJECTEVENTREQUEST_H


