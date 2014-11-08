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

#include "openjaus/system/Buffer.h"
#include <sstream>
// Start of user code for additional includes
#include <cstdlib>
#include <cstring>
#include <iomanip>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Buffer::Buffer() :
	pointer(NULL),
	maxSize(0),
	buffer(NULL)
{
}

Buffer::Buffer(int maxSize) :
	pointer(NULL),
	maxSize(0),
	buffer(NULL)
{
	setMaxSize(maxSize);
}

Buffer::Buffer(const Buffer &clone) :
	pointer(NULL),
	maxSize(0),
	buffer(NULL)
{
	setMaxSize(clone.getMaxSize());
	memcpy(pointer, clone.buffer, clone.containedBytes());
}
// End of user code

// Start of user code for default destructor:
Buffer::~Buffer()
{
	this->free();
}
// End of user code

unsigned char * Buffer::getPointer() const
{
	// Start of user code for accessor getPointer:
	
	return pointer;
	// End of user code
}


int Buffer::getMaxSize() const
{
	// Start of user code for accessor getMaxSize:
	
	return maxSize;
	// End of user code
}

bool Buffer::setMaxSize(int maxSize)
{
	// Start of user code for accessor setMaxSize:
	if(maxSize > 0)
	{
		if(buffer)
		{
			buffer = static_cast<unsigned char *>(realloc(buffer, maxSize));
		}
		else
		{
			buffer = static_cast<unsigned char *>(calloc(maxSize, sizeof(unsigned char)));
			memset(buffer, 0, maxSize);
		}
		pointer = buffer;
		this->maxSize = maxSize;

		return true;
	}

	return false;
	// End of user code
}


unsigned char * Buffer::getBuffer() const
{
	// Start of user code for accessor getBuffer:
	
	return buffer;
	// End of user code
}



// Class Methods
int Buffer::append(Buffer &newBuffer)
{
	// Start of user code for method append:
	int overRun = newBuffer.containedBytes() - remainingBytes();
	if(overRun > 0)
	{
		int initialDataSize = containedBytes();
		setMaxSize(maxSize + overRun);
		pointer = buffer + initialDataSize;
	}

	memcpy(pointer, newBuffer.buffer, newBuffer.containedBytes());
	pointer += newBuffer.containedBytes();

	return pointer - buffer;
	// End of user code
}


void Buffer::free()
{
	// Start of user code for method free:
	if(buffer)
	{
		::free(buffer);
		buffer = NULL;
	}
	// End of user code
}


void Buffer::increment(int byteCount)
{
	// Start of user code for method increment:
	if(byteCount > 0)
	{
		pointer += byteCount;
	}
	// End of user code
}


int Buffer::remainingBytes() const
{
	// Start of user code for method remainingBytes:
	return maxSize - (pointer - buffer);
	// End of user code
}


int Buffer::containedBytes() const
{
	// Start of user code for method containedBytes:
	return pointer - buffer;
	// End of user code
}


int Buffer::clear()
{
	// Start of user code for method clear:
	std::memset(buffer, 0, maxSize);

	return maxSize;
	// End of user code
}


unsigned char * Buffer::reset()
{
	// Start of user code for method reset:
	pointer = buffer;

	return buffer;
	// End of user code
}


int Buffer::to(Buffer *dst, int byteCount)
{
	// Start of user code for method to(Buffer *dst, int byteCount):
	if(dst->remainingBytes() >= byteCount)
	{
		std::memcpy(dst->getPointer(), buffer, byteCount);
		dst->increment(byteCount);
		//increment(byteCount);
		return byteCount;
	}
	else
	{
		THROW_EXCEPTION("Insufficient space in destination buffer to copy source to.");
	}
	// End of user code
}


int Buffer::from(Buffer *src, int byteCount)
{
	// Start of user code for method from(Buffer *src, int byteCount):
	if(remainingBytes() >= byteCount)
	{
		std::memcpy(buffer, src->getPointer(), byteCount);
		src->increment(byteCount);
		//increment(byteCount);
		return byteCount;
	}
	else
	{
		THROW_EXCEPTION("Insufficient space in destination buffer to copy source to.");
	}
	// End of user code
}


int Buffer::setAllTo(unsigned char newValue)
{
	// Start of user code for method setAllTo:
	std::memset(buffer, newValue, maxSize);

	return maxSize;
	// End of user code
}


int Buffer::set(unsigned char value, int count)
{
	// Start of user code for method set:
	if(count <= remainingBytes())
	{
		std::memset(pointer, value, count);
		increment(count);
		return count;
	}
	else
	{
		THROW_EXCEPTION("Trying to set " << count << " bytes but only " << remainingBytes() << " bytes remaining in buffer.");
	}
	// End of user code
}



int Buffer::to(Buffer *dst)
{
	// Start of user code for method to:
	int length = dst->remainingBytes() < remainingBytes()? dst->remainingBytes() : remainingBytes();

	return to(dst, length);
	// End of user code
}

int Buffer::from(Buffer *src)
{
	// Start of user code for method from:
	int length = src->remainingBytes() < remainingBytes()? src->remainingBytes() : remainingBytes();

	return from(src, length);
	// End of user code
}

int Buffer::length()
{
	// Start of user code for method length:
	return remainingBytes();
	// End of user code
}


std::string Buffer::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;

	oss << "MaxSize: " << this->maxSize;
	for(int i = 0; i < this->maxSize; i++)
	{
		if(i % 32 == 0)
		{
			oss<< std::endl << std::setfill('0') << std::setw(6) << std::dec << i << ": ";
		}
		else if(i % 8 == 0)
		{
			oss << " ";
		}
		oss << std::setw(2) << std::setfill('0') << std::uppercase << std::hex << (uint16_t)(buffer[i]) << " ";
	}

	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Buffer& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods

Buffer& Buffer::operator=(const Buffer &rhs)
{
	if(this == &rhs)
	{
		return *this;
	}

	setMaxSize(rhs.getMaxSize());
	memcpy(pointer, rhs.buffer, rhs.getMaxSize());
	pointer += rhs.getMaxSize();
	return *this;
}

int Buffer::pack(std::string& source)
{
	unsigned int length = source.length();

	if(static_cast<unsigned int>(remainingBytes()) < length)
	{
		THROW_EXCEPTION("Buffer Too Small: " << remainingBytes() << " for " << length);
	}

	std::memcpy(pointer, source.data(), length);
	increment(length);
	return length;
}

int Buffer::pack(std::string& source, unsigned int length)
{
	if(static_cast<unsigned int>(remainingBytes()) < length)
	{
		THROW_EXCEPTION("Buffer Too Small: " << remainingBytes() << " for " << length);
	}

	unsigned int stringLength = source.length();
	if(stringLength < length)
	{
		std::memcpy(pointer, source.data(), stringLength);
		increment(stringLength);
		
		// Set remaining characters to NULL
		std::memset(pointer, 0, (length-stringLength));
		increment(length-stringLength);
	}
	else
	{
		std::memcpy(pointer, source.data(), length);
		increment(length);
	}

	return length;
}

int Buffer::unpack(std::string& dst, unsigned long length)
{
	if(length > static_cast<unsigned long>(remainingBytes()))
	{
		std::string tempString;
		tempString.assign(reinterpret_cast<const char *>(pointer), static_cast<size_t>(remainingBytes()));

		THROW_EXCEPTION("Buffer can't unpack string length: " << length <<
						", only " << remainingBytes() << " bytes remaining." <<
						" Partial string is: \"" << tempString << "\"");
	}

	dst.assign(reinterpret_cast<const char *>(pointer), static_cast<size_t>(length));
	increment(length);
	return length;
}

// End of user code

} // namespace system
} // namespace openjaus

