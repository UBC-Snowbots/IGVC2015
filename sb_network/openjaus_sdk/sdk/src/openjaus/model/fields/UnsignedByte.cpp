/**
\file UnsignedByte.h

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

#include "openjaus/model/fields/UnsignedByte.h"
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
UnsignedByte::UnsignedByte()
{
}
// End of user code

// Start of user code for default destructor:
UnsignedByte::~UnsignedByte()
{
}
// End of user code

uint8_t UnsignedByte::getValue() const
{
	// Start of user code for accessor getValue:
	
	return value;
	// End of user code
}

bool UnsignedByte::setValue(uint8_t value)
{
	// Start of user code for accessor setValue:
	this->value = value;
	return true;
	// End of user code
}


short UnsignedByte::getDefaultValue() const
{
	// Start of user code for accessor getDefaultValue:
	
	return defaultValue;
	// End of user code
}

bool UnsignedByte::setDefaultValue(short defaultValue)
{
	// Start of user code for accessor setDefaultValue:
	this->defaultValue = defaultValue;
	return true;
	// End of user code
}



// Class Methods

int UnsignedByte::to(system::Buffer *dst)
{
	// Start of user code for method to:
	return dst->pack(value);
	// End of user code
}

int UnsignedByte::from(system::Buffer *src)
{
	// Start of user code for method from:
	return src->unpack(value);
	// End of user code
}

int UnsignedByte::length()
{
	// Start of user code for method length:
	return sizeof(uint8_t);
	// End of user code
}


std::string UnsignedByte::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const UnsignedByte& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
std::string UnsignedByte::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<UnsignedByte name=\"" << this->name << "\" value=\"" << (int)this->value << "\" />\n";
	return oss.str();
}

// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

