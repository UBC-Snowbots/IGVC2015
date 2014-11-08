/**
\file MessageField.h

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

#include "openjaus/model/fields/MessageField.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{

// Start of user code for default constructor:
MessageField::MessageField()
{
}
// End of user code

// Start of user code for default destructor:
MessageField::~MessageField()
{
	this->buffer.free();
}
// End of user code

model::Message* MessageField::getMessage() const
{
	// Start of user code for accessor getMessage:
	system::Buffer *thisBuffer = const_cast<system::Buffer *>(&this->buffer);
	thisBuffer->reset();

	model::Message *message = new model::Message();
	message->setPayload(thisBuffer);
	thisBuffer->reset();
	message->from(thisBuffer);
	return message;
	// End of user code
}

bool MessageField::setMessage(model::Message* message)
{
	// Start of user code for accessor setMessage:
	this->buffer.setMaxSize(message->length());
	this->buffer.pack(*message);
	return true;
	// End of user code
}


const system::Buffer& MessageField::getBuffer() const
{
	// Start of user code for accessor getBuffer:
	THROW_EXCEPTION("Message::getBuffer: DO NOT CALL THIS FUNCTION");
	// End of user code
}

bool MessageField::setBuffer(const system::Buffer& buffer)
{
	// Start of user code for accessor setBuffer:
	THROW_EXCEPTION("Message::setBuffer: DO NOT CALL THIS FUNCTION");
	// End of user code
}



// Class Methods

int MessageField::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int result = 0;

	if(this->length() > dst->remainingBytes())
	{
		THROW_EXCEPTION("Destination Buffer Too Small. Not sufficient room to pack Message Field");
	}

	this->buffer.reset();
	result += dst->pack(static_cast<uint32_t>(this->buffer.getMaxSize()));
	result += dst->pack(this->buffer);

	return result;
	// End of user code
}

int MessageField::from(system::Buffer *src)
{
	// Start of user code for method from:
	int result = 0;

	uint32_t messageSize;
	result += src->unpack(messageSize);

	this->buffer.setMaxSize(messageSize);
	result += src->unpack(this->buffer);

	return result;
	// End of user code
}

int MessageField::length()
{
	// Start of user code for method length:
	int result = 0;

	result += sizeof(uint32_t); // Size
	result += this->buffer.containedBytes();

	return result;
	// End of user code
}


std::string MessageField::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const MessageField& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
std::string MessageField::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	// TODO: Can we recurse into the message?
	oss << prefix.str() << "<MessageField name=\"" << this->name << "\" size=\"" << const_cast<system::Buffer *>(&buffer)->containedBytes() << "\" >\n";
	return oss.str();
}

void MessageField::copy(model::Message* source)
{
	this->setMessage(source);
}

// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

