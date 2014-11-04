// File Header Here

#ifndef RESPONSECODEENUMERATION_H
#define RESPONSECODEENUMERATION_H

#include <openjaus.h>

namespace openjaus
{
namespace mobility
{

class ResponseCodeEnumeration : public openjaus::model::fields::Enumeration
{
public:
	ResponseCodeEnumeration();
	~ResponseCodeEnumeration();

	enum ResponseCodeEnum {INVALID_UID = 1, INVALID_PREVIOUS = 2, INVALID_NEXT = 3, UNSUPPORTED_TYPE = 4, ELEMENT_NOT_FOUND = 5, OUT_OF_MEMORY = 6, UNSPECIFIED_ERROR = 7};
	
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;	
	std::string toString() const;
	
	ResponseCodeEnum getValue(void) const;
	void setValue(ResponseCodeEnum value);

protected:
	ResponseCodeEnum value;
};

} // namespace mobility
} // namespace openjaus

#endif // RESPONSECODEENUMERATION_H

