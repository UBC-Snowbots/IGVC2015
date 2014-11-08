/**
\file Variant.h

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

#include "openjaus/model/fields/Variant.h"
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
Variant::Variant()
{
}
// End of user code

// Start of user code for default destructor:
Variant::~Variant()
{
}
// End of user code

long Variant::getIndex() const
{
	// Start of user code for accessor getIndex:
	
	return index;
	// End of user code
}

bool Variant::setIndex(long index)
{
	// Start of user code for accessor setIndex:
	this->index = index;
	return true;
	// End of user code
}


TypesUnsigned Variant::getTypeCount() const
{
	// Start of user code for accessor getTypeCount:
	
	return typeCount;
	// End of user code
}

bool Variant::setTypeCount(TypesUnsigned typeCount)
{
	// Start of user code for accessor setTypeCount:
	this->typeCount = typeCount;
	return true;
	// End of user code
}


Field* Variant::getValue() const
{
	// Start of user code for accessor getValue:
	
	return value;
	// End of user code
}

bool Variant::setValue(Field* value)
{
	// Start of user code for accessor setValue:
	this->value = value;
	return true;
	// End of user code
}


const std::vector< Field* >& Variant::getItems() const
{
	// Start of user code for accessor getItems:
	
	return items;
	// End of user code
}

bool Variant::setItems(const Field& items)
{
	// Start of user code for accessor setItems:
	return true;
	// End of user code
}



// Class Methods

int Variant::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int result = 0;

	return result;
	// End of user code
}

int Variant::from(system::Buffer *src)
{
	// Start of user code for method from:
	int result = 0;

	return result;
	// End of user code
}

int Variant::length()
{
	// Start of user code for method length:
	int result = 0;

	return result;
	// End of user code
}


std::string Variant::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Variant& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

