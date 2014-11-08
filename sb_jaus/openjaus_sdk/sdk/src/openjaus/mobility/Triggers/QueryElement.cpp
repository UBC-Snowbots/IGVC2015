
/**
\file QueryElement.h

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
#include "openjaus/mobility/Triggers/QueryElement.h"

namespace openjaus
{
namespace mobility
{

QueryElement::QueryElement() : 
	model::Message(),
	elementUID()
{
	this->id = QueryElement::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&elementUID);
	elementUID.setName("ElementUID");
	elementUID.setOptional(false);
	elementUID.setInterpretation("1 to 65534, values of 0 and 65535 are reserved.");
	elementUID.setValue(0);

}

QueryElement::QueryElement(model::Message *message) :
	model::Message(message),
	elementUID()
{
	this->id = QueryElement::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&elementUID);
	elementUID.setName("ElementUID");
	elementUID.setOptional(false);
	elementUID.setInterpretation("1 to 65534, values of 0 and 65535 are reserved.");
	elementUID.setValue(0);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

QueryElement::~QueryElement()
{

}


uint16_t QueryElement::getElementUID(void)
{
	return this->elementUID.getValue();
}

void QueryElement::setElementUID(uint16_t value)
{
	this->elementUID.setValue(value);
}

int QueryElement::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(elementUID);
	return byteSize;
}

int QueryElement::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(elementUID);
	return byteSize;
}

int QueryElement::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += elementUID.length(); // elementUID
	return length;
}

std::string QueryElement::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"QueryElement\"";
	oss << " id=\"0x241A\" >\n";
	oss << elementUID.toXml(level+1); // elementUID
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace mobility
} // namespace openjaus


