
/**
\file ConfirmEventRequest.h

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
#include "openjaus/core/Triggers/ConfirmEventRequest.h"

namespace openjaus
{
namespace core
{

ConfirmEventRequest::ConfirmEventRequest() : 
	model::Message(),
	requestID(),
	eventID(),
	confirmedPeriodicRate_Hz()
{
	this->id = ConfirmEventRequest::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setInterpretation("ID of the event maintenance request (Create, Update, or Cancel)");
	requestID.setValue(0);

	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of existing event to be removed");
	eventID.setValue(0);

	fields.push_back(&confirmedPeriodicRate_Hz);
	confirmedPeriodicRate_Hz.setName("ConfirmedPeriodicRate");
	confirmedPeriodicRate_Hz.setOptional(false);
	confirmedPeriodicRate_Hz.setInterpretation("Requested rate or closest to requested rate");
	// Nothing to init

}

ConfirmEventRequest::ConfirmEventRequest(model::Message *message) :
	model::Message(message),
	requestID(),
	eventID(),
	confirmedPeriodicRate_Hz()
{
	this->id = ConfirmEventRequest::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setInterpretation("ID of the event maintenance request (Create, Update, or Cancel)");
	requestID.setValue(0);

	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of existing event to be removed");
	eventID.setValue(0);

	fields.push_back(&confirmedPeriodicRate_Hz);
	confirmedPeriodicRate_Hz.setName("ConfirmedPeriodicRate");
	confirmedPeriodicRate_Hz.setOptional(false);
	confirmedPeriodicRate_Hz.setInterpretation("Requested rate or closest to requested rate");
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ConfirmEventRequest::~ConfirmEventRequest()
{

}


int8_t ConfirmEventRequest::getRequestID(void)
{
	return this->requestID.getValue();
}

void ConfirmEventRequest::setRequestID(int8_t value)
{
	this->requestID.setValue(value);
}

int8_t ConfirmEventRequest::getEventID(void)
{
	return this->eventID.getValue();
}

void ConfirmEventRequest::setEventID(int8_t value)
{
	this->eventID.setValue(value);
}

double ConfirmEventRequest::getConfirmedPeriodicRate_Hz(void)
{
	return this->confirmedPeriodicRate_Hz.getValue();
}

void ConfirmEventRequest::setConfirmedPeriodicRate_Hz(double value)
{
	this->confirmedPeriodicRate_Hz.setValue(value);
}

int ConfirmEventRequest::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(requestID);
	byteSize += dst->pack(eventID);
	byteSize += dst->pack(confirmedPeriodicRate_Hz);
	return byteSize;
}

int ConfirmEventRequest::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(requestID);
	byteSize += src->unpack(eventID);
	byteSize += src->unpack(confirmedPeriodicRate_Hz);
	return byteSize;
}

int ConfirmEventRequest::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += requestID.length(); // requestID
	length += eventID.length(); // eventID
	length += confirmedPeriodicRate_Hz.length(); // confirmedPeriodicRate_Hz
	return length;
}

std::string ConfirmEventRequest::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ConfirmEventRequest\"";
	oss << " id=\"0x01F3\" >\n";
	oss << requestID.toXml(level+1); // requestID
	oss << eventID.toXml(level+1); // eventID
	oss << confirmedPeriodicRate_Hz.toXml(level+1); // confirmedPeriodicRate_Hz
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


