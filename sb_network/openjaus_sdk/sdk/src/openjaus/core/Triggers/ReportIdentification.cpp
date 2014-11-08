
/**
\file ReportIdentification.h

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
#include "openjaus/core/Triggers/ReportIdentification.h"

namespace openjaus
{
namespace core
{

ReportIdentification::ReportIdentification() : 
	model::Message(),
	queryType(),
	type(),
	identification()
{
	this->id = ReportIdentification::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&queryType);
	queryType.setName("queryType");
	queryType.setOptional(false);
	// Nothing to init

	fields.push_back(&type);
	type.setName("type");
	type.setOptional(false);
	type.setValue(0);

	fields.push_back(&identification);
	identification.setName("identification");
	identification.setOptional(false);
	identification.setSizeType(model::fields::UNSIGNED_BYTE);

}

ReportIdentification::ReportIdentification(model::Message *message) :
	model::Message(message),
	queryType(),
	type(),
	identification()
{
	this->id = ReportIdentification::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&queryType);
	queryType.setName("queryType");
	queryType.setOptional(false);
	// Nothing to init

	fields.push_back(&type);
	type.setName("type");
	type.setOptional(false);
	type.setValue(0);

	fields.push_back(&identification);
	identification.setName("identification");
	identification.setOptional(false);
	identification.setSizeType(model::fields::UNSIGNED_BYTE);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportIdentification::~ReportIdentification()
{

}


SystemLevelEnumeration::SystemLevelEnum ReportIdentification::getQueryType(void)
{
	return this->queryType.getValue();
}

void ReportIdentification::setQueryType(SystemLevelEnumeration::SystemLevelEnum value)
{
	this->queryType.setValue(value);
}

uint16_t ReportIdentification::getType(void)
{
	return this->type.getValue();
}

void ReportIdentification::setType(uint16_t value)
{
	this->type.setValue(value);
}

std::string ReportIdentification::getIdentification(void)
{
	return this->identification.getValue();
}

void ReportIdentification::setIdentification(std::string value)
{
	this->identification.setValue(value);
}

int ReportIdentification::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(queryType);
	byteSize += dst->pack(type);
	byteSize += dst->pack(identification);
	return byteSize;
}

int ReportIdentification::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(queryType);
	byteSize += src->unpack(type);
	byteSize += src->unpack(identification);
	return byteSize;
}

int ReportIdentification::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += queryType.length(); // queryType
	length += type.length(); // type
	length += identification.length(); // identification
	return length;
}

std::string ReportIdentification::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportIdentification\"";
	oss << " id=\"0x4B00\" >\n";
	oss << queryType.toXml(level+1); // queryType
	oss << type.toXml(level+1); // type
	oss << identification.toXml(level+1); // identification
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


