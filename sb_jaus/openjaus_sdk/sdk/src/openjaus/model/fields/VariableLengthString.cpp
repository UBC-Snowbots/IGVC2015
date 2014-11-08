/**
\file VariableLengthString.h

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

#include "openjaus/model/fields/VariableLengthString.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{

// Start of user code for default constructor:
VariableLengthString::VariableLengthString()
{
}
// End of user code

// Start of user code for default destructor:
VariableLengthString::~VariableLengthString()
{
}
// End of user code

std::string VariableLengthString::getValue() const
{
	// Start of user code for accessor getValue:
	
	return value;
	// End of user code
}

bool VariableLengthString::setValue(std::string value)
{
	// Start of user code for accessor setValue:
	this->value = value;
	return true;
	// End of user code
}


std::string VariableLengthString::getDefaultValue() const
{
	// Start of user code for accessor getDefaultValue:
	
	return defaultValue;
	// End of user code
}

bool VariableLengthString::setDefaultValue(std::string defaultValue)
{
	// Start of user code for accessor setDefaultValue:
	this->defaultValue = defaultValue;
	return true;
	// End of user code
}


TypesUnsigned VariableLengthString::getSizeType() const
{
	// Start of user code for accessor getSizeType:
	
	return sizeType;
	// End of user code
}

bool VariableLengthString::setSizeType(TypesUnsigned sizeType)
{
	// Start of user code for accessor setSizeType:
	this->sizeType = sizeType;
	return true;
	// End of user code
}



// Class Methods

int VariableLengthString::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int result = 0;

	// Pack count_field
	switch(this->getSizeType())
	{
		case fields::UNSIGNED_BYTE:
		{
			uint8_t length = static_cast<uint8_t>(this->value.length());
			result += dst->pack(length);
			break;
		}

		case fields::UNSIGNED_SHORT:
		{
			uint16_t length = static_cast<uint16_t>(this->value.length());
			result += dst->pack(length);
			break;
		}

		case fields::UNSIGNED_INTEGER:
		{
			uint32_t length = static_cast<uint32_t>(this->value.length());
			result += dst->pack(length);
			break;
		}

		case fields::UNSIGNED_LONG:
		{
			uint64_t length = static_cast<uint64_t>(this->value.length());
			result += dst->pack(length);
			break;
		}

		default:
			THROW_EXCEPTION("VariableLengthString::to: Unknown String SizeType: \"" << this->value << "\" Type: " << this->getSizeType());
	}

	result += dst->pack(this->value);

	return result;
	// End of user code
}

int VariableLengthString::from(system::Buffer *src)
{
	// Start of user code for method from:
	int result = 0;

	switch(this->getSizeType())
	{
		case fields::UNSIGNED_BYTE:
		{
			// Unpack count_field
			uint8_t length;
			result += src->unpack(length);

			// Unpack String
			result += src->unpack(this->value, length);
			return result;
		}

		case fields::UNSIGNED_SHORT:
		{
			// Unpack count_field
			uint16_t length;
			result += src->unpack(length);

			// Unpack String
			result += src->unpack(this->value, length);
			return result;
		}

		case fields::UNSIGNED_INTEGER:
		{
			// Unpack count_field
			uint32_t length;
			result += src->unpack(length);

			// Unpack String
			result += src->unpack(this->value, length);
			return result;
		}

		case fields::UNSIGNED_LONG:
		{
			// Unpack count_field
			uint64_t length;
			result += src->unpack(length);

			// Unpack String
			result += src->unpack(this->value, length);
			return result;
		}

		default:
			THROW_EXCEPTION("VariableLengthString::from: Unknown String SizeType: \"" << this->value << "\" Type: " << this->getSizeType());

	}

	// End of user code
}

int VariableLengthString::length()
{
	// Start of user code for method length:
	int result = 0;

	switch(this->getSizeType())
	{
		case fields::UNSIGNED_BYTE:
			result += sizeof(uint8_t); // count_field
			break;

		case fields::UNSIGNED_SHORT:
			result += sizeof(uint16_t); // count_field
			break;

		case fields::UNSIGNED_INTEGER:
			result += sizeof(uint32_t); // count_field
			break;

		case fields::UNSIGNED_LONG:
			result += sizeof(uint64_t); // count_field
			break;

		default:
			THROW_EXCEPTION("VariableLengthString::length: Unknown String SizeType: \"" << this->value << "\" Type: " << this->getSizeType());
	}

	result += (this->value.length()); // String data
	return result;
	// End of user code
}


std::string VariableLengthString::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const VariableLengthString& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
std::string VariableLengthString::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<VariableLengthString name=\"" << this->name << "\" >\n";
	oss << prefix.str() << "\t" << "<length>" << (this->value.length()) << "</length>\n";
	oss << prefix.str() << "\t" << "<value>" << this->value << "</value>\n";
	oss << prefix.str() << "</VariableLengthString>\n";
	return oss.str();
}

// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

