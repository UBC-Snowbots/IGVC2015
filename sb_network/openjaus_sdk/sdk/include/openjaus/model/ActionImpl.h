/**
\file ActionImpl.h

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
#ifndef MODEL_ACTIONIMPL_H
#define MODEL_ACTIONIMPL_H

#include "openjaus/model/Action.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "openjaus/model/Action.h"
// End of user code

namespace openjaus
{
namespace model
{

/// \class ActionImpl ActionImpl.h

template < class C >
class OPENJAUS_EXPORT ActionImpl : public Action
{
public:
	ActionImpl(); 
	virtual ~ActionImpl();
	// Start of user code for additional constructors
	ActionImpl(void(C::*fp)(), C* object);
	// End of user code

	virtual void execute();

	std::string toString() const;

protected:

// Start of user code for additional member data
	void (C::*action)();
	C *object;
// End of user code

}; // class ActionImpl

// Start of user code for inline functions
// End of user code

// Start of user code for default constructor:
template <class C>
ActionImpl<C>::ActionImpl()
{
}
// End of user code

// Start of user code for default destructor:
template <class C>
ActionImpl<C>::~ActionImpl()
{
}
// End of user code


// Class Methods
template <class C>
void ActionImpl<C>::execute()
{
	// Start of user code for method execute:
	(object->*action)();
	// End of user code
}




template <class C>
std::string ActionImpl<C>::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

// Start of user code for additional methods
template <class C>
ActionImpl<C>::ActionImpl(void(C::*fp)(), C* object)
{
	this->action = fp;
	this->object = object;
}

// End of user code


} // namespace model
} // namespace openjaus

#endif // MODEL_ACTIONIMPL_H

