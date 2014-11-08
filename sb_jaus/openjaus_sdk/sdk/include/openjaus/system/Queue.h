/**
\file Queue.h

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
#ifndef SYSTEM_QUEUE_H
#define SYSTEM_QUEUE_H

#include "openjaus/system/Mutex.h"
#include <list>
#include "openjaus/system/Condition.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include <iostream>
// End of user code

namespace openjaus
{
namespace system
{
class Mutex;

/// \class Queue Queue.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
template < class Type >
class OPENJAUS_EXPORT Queue : public Condition
{
public:
	Queue(); 
	virtual ~Queue();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of objects.
	const std::list< Type >& getObjects() const;


	/// Accessor to get the value of mutex.
	const Mutex& getMutex() const;


	/// Operation push.
	/// \param item 
	 bool push(Type item);

	/// Operation pop.
	 Type pop();

	/// Operation isEmpty.
	 bool isEmpty();

	/// Operation getSize.
	 int getSize();

	std::string toString() const;

protected:
	// Member attributes & references
	std::string name;
	std::list< Type > objects;
	Mutex mutex;

// Start of user code for additional member data
// End of user code

}; // class Queue

// Start of user code for inline functions
// End of user code

// Start of user code for default constructor:
template <class Type>
Queue<Type>::Queue() :
	Condition(),
	name("queue"),
	objects(),
	mutex()
{
}
// End of user code

// Start of user code for default destructor:
template <class Type>
Queue<Type>::~Queue()
{
}
// End of user code

template <class Type>
std::string Queue<Type>::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

template <class Type>
bool Queue<Type>::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


template <class Type>
const std::list< Type >& Queue<Type>::getObjects() const
{
	// Start of user code for accessor getObjects:
	
	return objects;
	// End of user code
}


template <class Type>
const Mutex& Queue<Type>::getMutex() const
{
	// Start of user code for accessor getMutex:
	
	return mutex;
	// End of user code
}



// Class Methods
template <class Type>
bool Queue<Type>::push(Type item)
{
	// Start of user code for method push:
	mutex.lock();

	objects.push_back(item);

	signal();

	mutex.unlock();
	return true;
	// End of user code
}


template <class Type>
Type Queue<Type>::pop()
{
	// Start of user code for method pop:
	if(objects.empty())
	{
		return Type();
	}

	mutex.lock();

	Type object = objects.front();
	objects.pop_front();

	mutex.unlock();
	return object;
	// End of user code
}


template <class Type>
bool Queue<Type>::isEmpty()
{
	// Start of user code for method isEmpty:
	return objects.empty();
	// End of user code
}


template <class Type>
int Queue<Type>::getSize()
{
	// Start of user code for method getSize:
	return objects.size();
	// End of user code
}




template <class Type>
std::string Queue<Type>::toString() const
{	
	// Start of user code for toString
	return name;
	// End of user code
}

// Start of user code for additional methods
// End of user code


} // namespace system
} // namespace openjaus

#endif // SYSTEM_QUEUE_H

