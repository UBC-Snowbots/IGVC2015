// File Header Here
#ifndef FIELDS_MESSAGE_H
#define FIELDS_MESSAGE_H

#include "openjaus/model/Message.h"
#include "openjaus/system/Buffer.h"
#include "openjaus/model/fields/Field.h"
#include "openjaus/system/Transportable.h"
#include <string>
#include "openjaus/system/Buffer.h"
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
class Message;
namespace fields
{
class Message;

/// \class Message Message.h

class Message : public Field, public system::Transportable
{
public:
	Message(); 
	virtual ~Message();
	// Start of user code for additional constructors
	// End of user code

	/// Accessor to get the value of message.
	model::Message* getMessage() const;

	/// Accessor to set value of message.
	/// \param message The value of the new message.
	bool setMessage(model::Message* message);

	/// Accessor to get the value of buffer.
	const system::Buffer& getBuffer() const;

	/// Accessor to set value of buffer.
	/// \param buffer The value of the new buffer.
	bool setBuffer(const system::Buffer& buffer);

	// Inherited pure virtuals from Transportable that need to be implemented
	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);	
	virtual int length();	

	std::string toString() const;

protected:
	// Member attributes & references
	model::Message *message;
	system::Buffer buffer;

// Start of user code for additional member data
// End of user code

}; // class Message

// Start of user code for inline functions
// End of user code


} // namespace fields
} // namespace model
} // namespace openjaus

#endif // FIELDS_MESSAGE_H

