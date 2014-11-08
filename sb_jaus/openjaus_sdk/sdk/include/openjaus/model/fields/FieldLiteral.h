// File Header Here
#ifndef FIELDS_FIELDLITERAL_H
#define FIELDS_FIELDLITERAL_H

#include <string>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{

/// \class FieldLiteral FieldLiteral.h

class FieldLiteral 
{
public:
	 
	virtual ~FieldLiteral(){};
	// Start of user code for additional constructors
	// End of user code

	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of optional.
	bool isOptional() const;

	/// Accessor to set value of optional.
	/// \param optional The value of the new optional.
	bool setOptional(bool optional);

	/// Accessor to get the value of interpretation.
	std::string getInterpretation() const;

	/// Accessor to set value of interpretation.
	/// \param interpretation The value of the new interpretation.
	bool setInterpretation(std::string interpretation);

	std::string toString() const;

protected:
	// Member attributes & references
	std::string name;
	bool optional;
	std::string interpretation;

// Start of user code for additional member data
// End of user code

}; // class FieldLiteral


} // namespace fields
} // namespace model
} // namespace openjaus

#endif // FIELDS_FIELDLITERAL_H

