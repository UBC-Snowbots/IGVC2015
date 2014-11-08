// File Header Here

#include "openjaus/model/fields/Message.h"
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
Message::Message()
{
}
// End of user code

// Start of user code for default destructor:
Message::~Message()
{
}
// End of user code

model::Message* Message::getMessage() const
{
	// Start of user code for accessor getMessage:
	system::Buffer *thisBuffer = const_cast<system::Buffer *>(&this->buffer);
	thisBuffer->reset();

	system::Buffer *buffer = new system::Buffer();
	buffer->setMaxSize(this->buffer.getMaxSize());
	buffer->from(thisBuffer);
	buffer->reset();

	model::Message *message = new model::Message();
	message->setPayload(buffer);
	thisBuffer->reset();
	message->from(thisBuffer);
	return message;
	// End of user code
}

bool Message::setMessage(model::Message* message)
{
	// Start of user code for accessor setMessage:
	this->buffer.setMaxSize(message->length());
	this->buffer.pack(*message);
	return true;
	// End of user code
}


const system::Buffer& Message::getBuffer() const
{
	// Start of user code for accessor getBuffer:
	THROW_EXCEPTION("Message::getBuffer: DO NOT CALL THIS FUNCTION");
	// End of user code
}

bool Message::setBuffer(const system::Buffer& buffer)
{
	// Start of user code for accessor setBuffer:
	THROW_EXCEPTION("Message::setBuffer: DO NOT CALL THIS FUNCTION");
	// End of user code
}



// Class Methods

int Message::to(system::Buffer *dst)
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

int Message::from(system::Buffer *src)
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

int Message::length()
{
	// Start of user code for method length:
	int result = 0;

	result += sizeof(uint32_t); // Size
	result += this->buffer.containedBytes();

	return result;
	// End of user code
}


std::string Message::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

// Start of user code for additional methods
// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

