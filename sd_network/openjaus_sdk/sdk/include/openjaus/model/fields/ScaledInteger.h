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
#ifndef FIELDS_SCALEDINTEGER_H
#define FIELDS_SCALEDINTEGER_H

#include "openjaus/model/fields/ScaledIntegerLimit.h"
#include "openjaus/model/fields/TypesUnsigned.h"
#include "openjaus/model/fields/RoundingType.h"
#include "openjaus/model/fields/ComplexField.h"
#include "openjaus/model/fields/Field.h"
#include "openjaus/model/fields/Unitized.h"
#include "openjaus/system/Transportable.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

#include "openjaus/model/fields/Units.h"
#include "openjaus/system/Buffer.h"
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{
class ScaledIntegerLimit;

/// \class ScaledInteger ScaledInteger.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT ScaledInteger : public ComplexField, public Field, public Unitized, public system::Transportable
{
public:
	ScaledInteger(); 
	virtual ~ScaledInteger();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of value.
	double getValue() const;

	/// Accessor to set value of value.
	/// \param value The value of the new value.
	bool setValue(double value);

	/// Accessor to get the value of integerType.
	TypesUnsigned getIntegerType() const;

	/// Accessor to set value of integerType.
	/// \param integerType The value of the new integerType.
	bool setIntegerType(TypesUnsigned integerType);

	/// Accessor to get the value of roundingType.
	RoundingType getRoundingType() const;

	/// Accessor to set value of roundingType.
	/// \param roundingType The value of the new roundingType.
	bool setRoundingType(RoundingType roundingType);

	/// Accessor to get the value of defaultValue.
	double getDefaultValue() const;

	/// Accessor to set value of defaultValue.
	/// \param defaultValue The value of the new defaultValue.
	bool setDefaultValue(double defaultValue);

	/// Accessor to get the value of upperLimit.
	const ScaledIntegerLimit& getUpperLimit() const;

	/// Accessor to set value of upperLimit.
	/// \param upperLimit The value of the new upperLimit.
	bool setUpperLimit(const ScaledIntegerLimit& upperLimit);

	/// Accessor to get the value of lowerLimit.
	const ScaledIntegerLimit& getLowerLimit() const;

	/// Accessor to set value of lowerLimit.
	/// \param lowerLimit The value of the new lowerLimit.
	bool setLowerLimit(const ScaledIntegerLimit& lowerLimit);

	// Inherited pure virtuals from ComplexField that need to be implemented

	// Inherited pure virtuals from Unitized that need to be implemented

	// Inherited pure virtuals from Transportable that need to be implemented
	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);	
	virtual int length();	

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const ScaledInteger& object);

protected:
	// Member attributes & references
	double value;
	TypesUnsigned integerType;
	RoundingType roundingType;
	double defaultValue;
	ScaledIntegerLimit upperLimit;
	ScaledIntegerLimit lowerLimit;

// Start of user code for additional member data
// End of user code

}; // class ScaledInteger

// Start of user code for inline functions
// End of user code



} // namespace fields
} // namespace model
} // namespace openjaus

#endif // FIELDS_SCALEDINTEGER_H

