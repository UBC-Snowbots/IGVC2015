
/**
\file QueryGlobalPathSegment.h

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
#include "openjaus/mobility/Triggers/QueryGlobalPathSegment.h"

namespace openjaus
{
namespace mobility
{

QueryGlobalPathSegment::QueryGlobalPathSegment() : 
	model::Message(),
	queryPresenceVector()
{
	this->id = QueryGlobalPathSegment::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&queryPresenceVector);
	queryPresenceVector.setName("QueryPresenceVector");
	queryPresenceVector.setOptional(false);
	queryPresenceVector.setValue(0);

}

QueryGlobalPathSegment::QueryGlobalPathSegment(model::Message *message) :
	model::Message(message),
	queryPresenceVector()
{
	this->id = QueryGlobalPathSegment::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&queryPresenceVector);
	queryPresenceVector.setName("QueryPresenceVector");
	queryPresenceVector.setOptional(false);
	queryPresenceVector.setValue(0);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

QueryGlobalPathSegment::~QueryGlobalPathSegment()
{

}


uint8_t QueryGlobalPathSegment::getQueryPresenceVector(void)
{
	return this->queryPresenceVector.getValue();
}

void QueryGlobalPathSegment::setQueryPresenceVector(uint8_t value)
{
	this->queryPresenceVector.setValue(value);
}

int QueryGlobalPathSegment::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(queryPresenceVector);
	return byteSize;
}

int QueryGlobalPathSegment::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(queryPresenceVector);
	return byteSize;
}

int QueryGlobalPathSegment::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += queryPresenceVector.length(); // queryPresenceVector
	return length;
}

std::string QueryGlobalPathSegment::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"QueryGlobalPathSegment\"";
	oss << " id=\"0x240F\" >\n";
	oss << queryPresenceVector.toXml(level+1); // queryPresenceVector
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace mobility
} // namespace openjaus


