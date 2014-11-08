// File Header Here

#include "openjaus/model/fields/VariantItem.h"
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
VariantItem::VariantItem()
{
}
// End of user code

// Start of user code for default destructor:
VariantItem::~VariantItem()
{
}
// End of user code

long VariantItem::getId() const
{
	// Start of user code for accessor getId:
	
	return id;
	// End of user code
}

bool VariantItem::setId(long id)
{
	// Start of user code for accessor setId:
	this->id = id;
	return true;
	// End of user code
}


const Field& VariantItem::getType() const
{
	// Start of user code for accessor getType:
	
	return type;
	// End of user code
}

bool VariantItem::setType(const Field& type)
{
	// Start of user code for accessor setType:
	this->type = type;
	return true;
	// End of user code
}



// Class Methods


std::string VariantItem::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

// Start of user code for additional methods
// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

