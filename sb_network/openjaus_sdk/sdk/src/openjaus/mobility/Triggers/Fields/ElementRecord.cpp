/**
\file ElementRecord.h

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
#include "openjaus/mobility/Triggers/Fields/ElementRecord.h"

namespace openjaus
{
namespace mobility
{

ElementRecord::ElementRecord():
	elementUID(),
	previousUID(),
	nextUID(),
	elementData()
{

	fields.push_back(&elementUID);
	elementUID.setName("ElementUID");
	elementUID.setOptional(false);
	elementUID.setInterpretation("1 to 65534, values of 0 and 65535 are reserved.");
	elementUID.setValue(0);

	fields.push_back(&previousUID);
	previousUID.setName("PreviousUID");
	previousUID.setOptional(false);
	previousUID.setInterpretation("UID of the previous (parent) element in this list. The value is 0 if this is the first (head) element.");
	previousUID.setValue(0);

	fields.push_back(&nextUID);
	nextUID.setName("NextUID");
	nextUID.setOptional(false);
	nextUID.setInterpretation("UID of the next (child) element in this list. The value is 0 if this is the last (tail) element.");
	nextUID.setValue(0);

	fields.push_back(&elementData);
	elementData.setName("ElementData");
	elementData.setOptional(false);
	// Nothing to Init

}

ElementRecord::ElementRecord(const ElementRecord &source)
{
	this->copy(const_cast<ElementRecord&>(source));
}

ElementRecord::~ElementRecord()
{

}


uint16_t ElementRecord::getElementUID(void)
{
	return this->elementUID.getValue();
}

void ElementRecord::setElementUID(uint16_t value)
{
	this->elementUID.setValue(value);
}

uint16_t ElementRecord::getPreviousUID(void)
{
	return this->previousUID.getValue();
}

void ElementRecord::setPreviousUID(uint16_t value)
{
	this->previousUID.setValue(value);
}

uint16_t ElementRecord::getNextUID(void)
{
	return this->nextUID.getValue();
}

void ElementRecord::setNextUID(uint16_t value)
{
	this->nextUID.setValue(value);
}

model::Message *ElementRecord::getElementData(void)
{
	model::Message *msg = this->elementData.getMessage();
	msg->setType(transport::JAUS_MESSAGE);

	return msg;
}

void ElementRecord::setElementData(model::Message* message)
{
	this->elementData.setMessage(message);
}

int ElementRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(elementUID);
	byteSize += dst->pack(previousUID);
	byteSize += dst->pack(nextUID);
	byteSize += dst->pack(elementData);
	return byteSize;
}
int ElementRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(elementUID);
	byteSize += src->unpack(previousUID);
	byteSize += src->unpack(nextUID);
	byteSize += src->unpack(elementData);
	return byteSize;
}

int ElementRecord::length(void)
{
	int length = 0;
	length += elementUID.length(); // elementUID
	length += previousUID.length(); // previousUID
	length += nextUID.length(); // nextUID
	length += elementData.length(); // elementData
	return length;
}

std::string ElementRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ElementRecord\">\n";
	oss << elementUID.toXml(level+1); // elementUID
	oss << previousUID.toXml(level+1); // previousUID
	oss << nextUID.toXml(level+1); // nextUID
	oss << elementData.toXml(level+1); // elementData
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ElementRecord::copy(ElementRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->elementUID.setName("ElementUID");
	this->elementUID.setOptional(false);
	this->elementUID.setInterpretation("1 to 65534, values of 0 and 65535 are reserved.");
	this->elementUID.setValue(source.getElementUID()); 
 
	this->previousUID.setName("PreviousUID");
	this->previousUID.setOptional(false);
	this->previousUID.setInterpretation("UID of the previous (parent) element in this list. The value is 0 if this is the first (head) element.");
	this->previousUID.setValue(source.getPreviousUID()); 
 
	this->nextUID.setName("NextUID");
	this->nextUID.setOptional(false);
	this->nextUID.setInterpretation("UID of the next (child) element in this list. The value is 0 if this is the last (tail) element.");
	this->nextUID.setValue(source.getNextUID()); 
 
	this->elementData.copy(source.getElementData()); 
 
}

} // namespace mobility
} // namespace openjaus

