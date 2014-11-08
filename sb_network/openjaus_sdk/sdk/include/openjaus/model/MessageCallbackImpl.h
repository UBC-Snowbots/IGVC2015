/**
\file MessageCallbackImpl.h

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
#ifndef MODEL_MESSAGECALLBACKIMPL_H
#define MODEL_MESSAGECALLBACKIMPL_H

#include "openjaus/model/Trigger.h"
#include "openjaus/model/MessageCallback.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
class Trigger;

/// \class MessageCallbackImpl MessageCallbackImpl.h

template < class MessageType, class CallbackClass >
class OPENJAUS_EXPORT MessageCallbackImpl : public MessageCallback
{
public:
	MessageCallbackImpl(); 
	virtual ~MessageCallbackImpl();
	// Start of user code for additional constructors
	MessageCallbackImpl(bool(CallbackClass::*callback)(MessageType &messageRef), CallbackClass* object);
	// End of user code
	/// Accessor to get the value of object.
	CallbackClass* getObject() const;

	/// Accessor to set value of object.
	/// \param object The value of the new object.
	bool setObject(CallbackClass* object);


	/// \param trigger 
	virtual bool processTrigger(Trigger *trigger);

	std::string toString() const;

protected:
	// Member attributes & references
	CallbackClass *object;

// Start of user code for additional member data
	bool (CallbackClass::*callback)(MessageType &messageRef);
// End of user code

}; // class MessageCallbackImpl

// Start of user code for inline functions
// End of user code

// Start of user code for default constructor:
template <class MessageType, class CallbackClass>
MessageCallbackImpl<MessageType, CallbackClass>::MessageCallbackImpl() :
	object(NULL),
	callback(NULL)
{
}
// End of user code

// Start of user code for default destructor:
template <class MessageType, class CallbackClass>
MessageCallbackImpl<MessageType, CallbackClass>::~MessageCallbackImpl()
{
}
// End of user code

template <class MessageType, class CallbackClass>
CallbackClass* MessageCallbackImpl<MessageType, CallbackClass>::getObject() const
{
	// Start of user code for accessor getObject:
	
	return object;
	// End of user code
}

template <class MessageType, class CallbackClass>
bool MessageCallbackImpl<MessageType, CallbackClass>::setObject(CallbackClass* object)
{
	// Start of user code for accessor setObject:
	this->object = object;
	return true;
	// End of user code
}



// Class Methods
template <class MessageType, class CallbackClass>
bool MessageCallbackImpl<MessageType, CallbackClass>::processTrigger(Trigger *trigger)
{
	// Start of user code for method processTrigger:
	if(trigger->getId() != MessageType::ID)
	{
		return false;
	}

	model::Message *message = dynamic_cast<model::Message *>(trigger);
	MessageType specificMessage(message);

	if(object && callback)
	{
		return (this->object->*callback)(specificMessage);
	}

	return false;
	// End of user code
}




template <class MessageType, class CallbackClass>
std::string MessageCallbackImpl<MessageType, CallbackClass>::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

// Start of user code for additional methods
template <class MessageType, class CallbackClass>
MessageCallbackImpl<MessageType, CallbackClass>::MessageCallbackImpl(bool(CallbackClass::*callback)(MessageType &messageRef), CallbackClass* object)
{
	this->callback = callback;
	this->object = object;
}
// End of user code


} // namespace model
} // namespace openjaus

#endif // MODEL_MESSAGECALLBACKIMPL_H

