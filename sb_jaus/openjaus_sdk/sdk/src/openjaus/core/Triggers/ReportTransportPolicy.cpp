
/**
\file ReportTransportPolicy.h

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
#include "openjaus/core/Triggers/ReportTransportPolicy.h"

namespace openjaus
{
namespace core
{

ReportTransportPolicy::ReportTransportPolicy() : 
	model::Message(),
	supportOJWrappers(),
	supportTCP(),
	preferenceTCP()
{
	this->id = ReportTransportPolicy::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&supportOJWrappers);
	supportOJWrappers.setName("supportOJWrappers");
	supportOJWrappers.setOptional(false);
	supportOJWrappers.setValue(false);

	fields.push_back(&supportTCP);
	supportTCP.setName("supportTCP");
	supportTCP.setOptional(false);
	supportTCP.setValue(false);

	fields.push_back(&preferenceTCP);
	preferenceTCP.setName("preferenceTCP");
	preferenceTCP.setOptional(false);
	// Nothing to init

}

ReportTransportPolicy::ReportTransportPolicy(model::Message *message) :
	model::Message(message),
	supportOJWrappers(),
	supportTCP(),
	preferenceTCP()
{
	this->id = ReportTransportPolicy::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&supportOJWrappers);
	supportOJWrappers.setName("supportOJWrappers");
	supportOJWrappers.setOptional(false);
	supportOJWrappers.setValue(false);

	fields.push_back(&supportTCP);
	supportTCP.setName("supportTCP");
	supportTCP.setOptional(false);
	supportTCP.setValue(false);

	fields.push_back(&preferenceTCP);
	preferenceTCP.setName("preferenceTCP");
	preferenceTCP.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportTransportPolicy::~ReportTransportPolicy()
{

}


bool ReportTransportPolicy::getSupportOJWrappers(void)
{
	return this->supportOJWrappers.isValue();
}

void ReportTransportPolicy::setSupportOJWrappers(bool value)
{
	this->supportOJWrappers.setValue(value);
}

bool ReportTransportPolicy::getSupportTCP(void)
{
	return this->supportTCP.isValue();
}

void ReportTransportPolicy::setSupportTCP(bool value)
{
	this->supportTCP.setValue(value);
}

PreferenceTCPEnumeration::PreferenceTCPEnum ReportTransportPolicy::getPreferenceTCP(void)
{
	return this->preferenceTCP.getValue();
}

void ReportTransportPolicy::setPreferenceTCP(PreferenceTCPEnumeration::PreferenceTCPEnum value)
{
	this->preferenceTCP.setValue(value);
}

int ReportTransportPolicy::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(supportOJWrappers);
	byteSize += dst->pack(supportTCP);
	byteSize += dst->pack(preferenceTCP);
	return byteSize;
}

int ReportTransportPolicy::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(supportOJWrappers);
	byteSize += src->unpack(supportTCP);
	byteSize += src->unpack(preferenceTCP);
	return byteSize;
}

int ReportTransportPolicy::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += supportOJWrappers.length(); // supportOJWrappers
	length += supportTCP.length(); // supportTCP
	length += preferenceTCP.length(); // preferenceTCP
	return length;
}

std::string ReportTransportPolicy::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportTransportPolicy\"";
	oss << " id=\"0x6502\" >\n";
	oss << supportOJWrappers.toXml(level+1); // supportOJWrappers
	oss << supportTCP.toXml(level+1); // supportTCP
	oss << preferenceTCP.toXml(level+1); // preferenceTCP
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


