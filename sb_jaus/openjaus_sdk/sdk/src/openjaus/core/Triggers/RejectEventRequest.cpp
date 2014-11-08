
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

#include <openjaus.h>
#include "openjaus/core/Triggers/RejectEventRequest.h"

namespace openjaus
{
namespace core
{

RejectEventRequest::RejectEventRequest() : 
	model::Message(),
	requestID(),
	responseCode(),
	errorMessage()
{
	this->id = RejectEventRequest::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setInterpretation("ID of the event maintenance request (Create, Update, or Cancel)");
	requestID.setValue(0);

	fields.push_back(&responseCode);
	responseCode.setName("ResponseCode");
	responseCode.setOptional(false);
	responseCode.setInterpretation("Enumerated reason the event maintenance request was rejected.");
	// Nothing to init

	fields.push_back(&errorMessage);
	errorMessage.setName("ErrorMessage");
	errorMessage.setOptional(true);
	errorMessage.setInterpretation("String for additional information");
	errorMessage.setMaxLength(80);

}

RejectEventRequest::RejectEventRequest(model::Message *message) :
	model::Message(message),
	requestID(),
	responseCode(),
	errorMessage()
{
	this->id = RejectEventRequest::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setInterpretation("ID of the event maintenance request (Create, Update, or Cancel)");
	requestID.setValue(0);

	fields.push_back(&responseCode);
	responseCode.setName("ResponseCode");
	responseCode.setOptional(false);
	responseCode.setInterpretation("Enumerated reason the event maintenance request was rejected.");
	// Nothing to init

	fields.push_back(&errorMessage);
	errorMessage.setName("ErrorMessage");
	errorMessage.setOptional(true);
	errorMessage.setInterpretation("String for additional information");
	errorMessage.setMaxLength(80);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

RejectEventRequest::~RejectEventRequest()
{

}


int8_t RejectEventRequest::getRequestID(void)
{
	return this->requestID.getValue();
}

void RejectEventRequest::setRequestID(int8_t value)
{
	this->requestID.setValue(value);
}

ResponseCodeEnumeration::ResponseCodeEnum RejectEventRequest::getResponseCode(void)
{
	return this->responseCode.getValue();
}

void RejectEventRequest::setResponseCode(ResponseCodeEnumeration::ResponseCodeEnum value)
{
	this->responseCode.setValue(value);
}

std::string RejectEventRequest::getErrorMessage(void)
{
	return this->errorMessage.getValue();
}

void RejectEventRequest::setErrorMessage(std::string value)
{
	this->errorMessage.setValue(value);
}

int RejectEventRequest::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(requestID);
	byteSize += dst->pack(responseCode);
	if(this->isErrorMessageEnabled())
	{
		byteSize += dst->pack(errorMessage);
	}
	return byteSize;
}

int RejectEventRequest::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(requestID);
	byteSize += src->unpack(responseCode);
	if(this->isErrorMessageEnabled())
	{
		byteSize += src->unpack(errorMessage);
	}
	return byteSize;
}

int RejectEventRequest::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	length += requestID.length(); // requestID
	length += responseCode.length(); // responseCode
	if(this->isErrorMessageEnabled())
	{
		length += errorMessage.length(); // errorMessage
	}
	return length;
}

std::string RejectEventRequest::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"RejectEventRequest\"";
	oss << " id=\"0x01F4\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isErrorMessageEnabled value=\"" << std::boolalpha << this->isErrorMessageEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << requestID.toXml(level+1); // requestID
	oss << responseCode.toXml(level+1); // responseCode
	if(this->isErrorMessageEnabled())
	{
		oss << errorMessage.toXml(level+1); // errorMessage
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void RejectEventRequest::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t RejectEventRequest::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool RejectEventRequest::isErrorMessageEnabled(void) const
{
	return (this->presenceVector & (0x01 << RejectEventRequest::ERRORMESSAGE));
}

void RejectEventRequest::enableErrorMessage(void)
{
	this->presenceVector |= 0x01 << RejectEventRequest::ERRORMESSAGE;
}

void RejectEventRequest::disableErrorMessage(void)
{
	this->presenceVector &= ~(0x01 << RejectEventRequest::ERRORMESSAGE);
}

} // namespace core
} // namespace openjaus


