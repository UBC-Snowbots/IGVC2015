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
#ifndef FIELDS_VARIANT_H
#define FIELDS_VARIANT_H

#include "openjaus/model/fields/Field.h"
#include "openjaus/model/fields/TypesUnsigned.h"
#include "openjaus/model/fields/ComplexField.h"
#include "openjaus/system/Transportable.h"
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

#include "openjaus/system/Buffer.h"
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{

/// \class Variant Variant.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Variant : public ComplexField, public Field, public system::Transportable
{
public:
	Variant(); 
	virtual ~Variant();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of index.
	long getIndex() const;

	/// Accessor to set value of index.
	/// \param index The value of the new index.
	bool setIndex(long index);

	/// Accessor to get the value of typeCount.
	TypesUnsigned getTypeCount() const;

	/// Accessor to set value of typeCount.
	/// \param typeCount The value of the new typeCount.
	bool setTypeCount(TypesUnsigned typeCount);

	/// Accessor to get the value of value.
	Field* getValue() const;

	/// Accessor to set value of value.
	/// \param value The value of the new value.
	bool setValue(Field* value);

	/// Accessor to get the value of items.
	const std::vector< Field* >& getItems() const;

	/// Accessor to set value of items.
	/// \param items The value of the new items.
	bool setItems(const Field& items);

	// Inherited pure virtuals from ComplexField that need to be implemented

	// Inherited pure virtuals from Transportable that need to be implemented
	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);	
	virtual int length();	

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Variant& object);

protected:
	// Member attributes & references
	long index;
	TypesUnsigned typeCount;
	Field *value;
	std::vector< Field* > items;

// Start of user code for additional member data
// End of user code

}; // class Variant

// Start of user code for inline functions
// End of user code



} // namespace fields
} // namespace model
} // namespace openjaus

#endif // FIELDS_VARIANT_H

