/**
\file Buffer.h

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
#ifndef SYSTEM_BUFFER_H
#define SYSTEM_BUFFER_H

#include "openjaus/system/Buffer.h"
#include "openjaus/system/Transportable.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

#include "openjaus/system/Buffer.h"
// Start of user code for additional includes
#include <cstring>
#include "openjaus/system/Exception.h"
#include "openjaus/types.h"
// End of user code

namespace openjaus
{
namespace system
{
class Buffer;

/// \class Buffer Buffer.h

class OPENJAUS_EXPORT Buffer : public Transportable
{
public:
	Buffer(); 
	virtual ~Buffer();
	// Start of user code for additional constructors
	Buffer(int maxSize);
	Buffer(const Buffer &buffer);
	// End of user code
	/// Accessor to get the value of pointer.
	unsigned char * getPointer() const;


	/// Accessor to get the value of maxSize.
	int getMaxSize() const;

	/// Accessor to set value of maxSize.
	/// \param maxSize The value of the new maxSize.
	bool setMaxSize(int maxSize);

	/// Accessor to get the value of buffer.
	unsigned char * getBuffer() const;


	/// Operation append.
	/// \param newBuffer 
	 int append(Buffer &newBuffer);

	/// Operation free.
	 void free();

	/// Operation increment.
	/// \param byteCount 
	 void increment(int byteCount);


	 int remainingBytes() const;


	 int containedBytes() const;

	/// Operation clear.
	 int clear();

	/// Operation reset.
	 unsigned char * reset();

	/// Operation to.
	/// \param dst 
	/// \param byteCount 
	 int to(Buffer *dst, int byteCount);

	/// Operation from.
	/// \param src 
	/// \param byteCount 
	 int from(Buffer *src, int byteCount);

	/// Operation setAllTo.
	/// \param newValue 
	 int setAllTo(unsigned char newValue);

	/// Operation set.
	/// \param value 
	/// \param count 
	 int set(unsigned char value, int count);

	// Inherited pure virtuals from Transportable that need to be implemented
	virtual int to(Buffer *dst);	
	virtual int from(Buffer *src);	
	virtual int length();	

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Buffer& object);

protected:
	// Member attributes & references
	unsigned char * pointer;
	int maxSize;
	unsigned char * buffer;

// Start of user code for additional member data
public:
	Buffer& operator=(const Buffer &rhs);

	template <typename Type>
	int peekTemplate(Type &value);

	int peek(int8_t &value);
	int peek(int16_t &value);
	int peek(int32_t &value);
	int peek(int64_t &value);
	int peek(uint8_t &value);
	int peek(uint16_t &value);
	int peek(uint32_t &value);
	int peek(uint64_t &value);
	int peek(float &value);
	int peek(double &value);

	template <typename Type>
	int unpackTemplate(Type &value);

	int unpack(int8_t &value);
	int unpack(int16_t &value);
	int unpack(int32_t &value);
	int unpack(int64_t &value);
	int unpack(uint8_t &value);
	int unpack(uint16_t &value);
	int unpack(uint32_t &value);
	int unpack(uint64_t &value);
	int unpack(float &value);
	int unpack(double &value);
	int unpack(Transportable &value);
	int unpack(std::string& dst, unsigned long length);

	template <typename Type>
	int packTemplate(const Type &value);

	int pack(const int8_t &value);
	int pack(const int16_t &value);
	int pack(const int32_t &value);
	int pack(const int64_t &value);
	int pack(const uint8_t &value);
	int pack(const uint16_t &value);
	int pack(const uint32_t &value);
	int pack(const uint64_t &value);
	int pack(const float &value);
	int pack(const double &value);
	int pack(Transportable &value);
	int pack(std::string& source);
	int pack(std::string& source, unsigned int length);

// End of user code

}; // class Buffer

// Start of user code for inline functions
template <typename Type>
inline int Buffer::peekTemplate(Type &value)
{
	if(remainingBytes() < static_cast<int>(sizeof(Type)))
	{
		THROW_EXCEPTION("Buffer Too Small");
	}

	// TODO: Add endian type compensation here
	std::memcpy(&value, pointer, sizeof(Type));

	return sizeof(Type);
}

inline int Buffer::peek(int8_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(int16_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(int32_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(int64_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(uint8_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(uint16_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(uint32_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(uint64_t &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(float &value)
{
	return peekTemplate(value);
}

inline int Buffer::peek(double &value)
{
	return peekTemplate(value);
}

template <typename Type>
inline int Buffer::unpackTemplate(Type &value)
{
	peekTemplate<Type>(value);
	pointer += sizeof(Type);
	return sizeof(Type);
}

inline int Buffer::unpack(int8_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(int16_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(int32_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(int64_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(uint8_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(uint16_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(uint32_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(uint64_t &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(float &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(double &value)
{
	return unpackTemplate(value);
}

inline int Buffer::unpack(Transportable& object)
{
	return object.from(this);
}

template <typename Type>
inline int Buffer::packTemplate(const Type &value)
{
	if(static_cast<unsigned int>(remainingBytes()) < sizeof(Type))
	{
		THROW_EXCEPTION("Buffer Too Small: " << remainingBytes() << " for " << sizeof(Type));
	}

	// TODO: Add endian type compensation here
	std::memcpy(pointer, &value, sizeof(Type));

	pointer += sizeof(Type);

	return sizeof(Type);
}

inline int Buffer::pack(const int8_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const int16_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const int32_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const int64_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const uint8_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const uint16_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const uint32_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const uint64_t &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const float &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(const double &value)
{
	return packTemplate(value);
}

inline int Buffer::pack(Transportable& object)
{
	return object.to(this);
}

// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_BUFFER_H

