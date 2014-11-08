/**
\file ScaledInteger.h

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

#include "openjaus/model/fields/ScaledInteger.h"
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
ScaledInteger::ScaledInteger() :
	value(0),
	integerType(UNSIGNED_INTEGER),
	roundingType(FLOOR),
	defaultValue(0),
	upperLimit(),
	lowerLimit()
{
}
// End of user code

// Start of user code for default destructor:
ScaledInteger::~ScaledInteger()
{
}
// End of user code

double ScaledInteger::getValue() const
{
	// Start of user code for accessor getValue:
	
	return value;
	// End of user code
}

bool ScaledInteger::setValue(double value)
{
	// Start of user code for accessor setValue:
	this->value = value;
	return true;
	// End of user code
}


TypesUnsigned ScaledInteger::getIntegerType() const
{
	// Start of user code for accessor getIntegerType:
	
	return integerType;
	// End of user code
}

bool ScaledInteger::setIntegerType(TypesUnsigned integerType)
{
	// Start of user code for accessor setIntegerType:
	this->integerType = integerType;
	return true;
	// End of user code
}


RoundingType ScaledInteger::getRoundingType() const
{
	// Start of user code for accessor getRoundingType:
	
	return roundingType;
	// End of user code
}

bool ScaledInteger::setRoundingType(RoundingType roundingType)
{
	// Start of user code for accessor setRoundingType:
	this->roundingType = roundingType;
	return true;
	// End of user code
}


double ScaledInteger::getDefaultValue() const
{
	// Start of user code for accessor getDefaultValue:
	
	return defaultValue;
	// End of user code
}

bool ScaledInteger::setDefaultValue(double defaultValue)
{
	// Start of user code for accessor setDefaultValue:
	this->defaultValue = defaultValue;
	return true;
	// End of user code
}


const ScaledIntegerLimit& ScaledInteger::getUpperLimit() const
{
	// Start of user code for accessor getUpperLimit:
	
	return upperLimit;
	// End of user code
}

bool ScaledInteger::setUpperLimit(const ScaledIntegerLimit& upperLimit)
{
	// Start of user code for accessor setUpperLimit:
	this->upperLimit = upperLimit;
	return true;
	// End of user code
}


const ScaledIntegerLimit& ScaledInteger::getLowerLimit() const
{
	// Start of user code for accessor getLowerLimit:
	
	return lowerLimit;
	// End of user code
}

bool ScaledInteger::setLowerLimit(const ScaledIntegerLimit& lowerLimit)
{
	// Start of user code for accessor setLowerLimit:
	this->lowerLimit = lowerLimit;
	return true;
	// End of user code
}



// Class Methods

int ScaledInteger::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int result = 0;

	return result;
	// End of user code
}

int ScaledInteger::from(system::Buffer *src)
{
	// Start of user code for method from:
	int result = 0;

	return result;
	// End of user code
}

int ScaledInteger::length()
{
	// Start of user code for method length:
	int result = 0;

	return result;
	// End of user code
}


std::string ScaledInteger::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const ScaledInteger& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

