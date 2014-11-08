// File Header Here
#ifndef FIELDS_VARIANTITEM_H
#define FIELDS_VARIANTITEM_H

#include "openjaus/model/fields/Field.h"
#include <string>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{
class Field;

/// \class VariantItem VariantItem.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class VariantItem 
{
public:
	VariantItem(); 
	virtual ~VariantItem();
	// Start of user code for additional constructors
	// End of user code

	/// Accessor to get the value of id.
	long getId() const;

	/// Accessor to set value of id.
	/// \param id The value of the new id.
	bool setId(long id);

	/// Accessor to get the value of type.
	const Field& getType() const;

	/// Accessor to set value of type.
	/// \param type The value of the new type.
	bool setType(const Field& type);

	std::string toString() const;

protected:
	// Member attributes & references
	long id;
	Field type;

// Start of user code for additional member data
// End of user code

}; // class VariantItem


} // namespace fields
} // namespace model
} // namespace openjaus

#endif // FIELDS_VARIANTITEM_H

