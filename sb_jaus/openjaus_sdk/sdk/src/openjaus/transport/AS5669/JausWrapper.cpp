/**
\file JausWrapper.h

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

#include "openjaus/transport/AS5669/JausWrapper.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

// Start of user code for default constructor:
JausWrapper::JausWrapper() :
	Wrapper(),
	headerNumber(0),
	headerLength(0),
	headerFlags(0)
{
}

JausWrapper::JausWrapper(const transport::Wrapper& wrapper) :
	Wrapper(),
	headerNumber(0),
	headerLength(0),
	headerFlags(0)
{
	setSequenceNumber(wrapper.getSequenceNumber());
	setDestination(wrapper.getDestination());
	setSource(wrapper.getSource());
	setAckNak(wrapper.getAckNak());
	setBroadcastFlag(wrapper.getBroadcastFlag());
	setPriority(wrapper.getPriority());
	setPayload(wrapper.getPayload());
}
// End of user code

// Start of user code for default destructor:
JausWrapper::~JausWrapper()
{
}
// End of user code

unsigned char JausWrapper::getHeaderNumber() const
{
	// Start of user code for accessor getHeaderNumber:
	
	return headerNumber;
	// End of user code
}

bool JausWrapper::setHeaderNumber(unsigned char headerNumber)
{
	// Start of user code for accessor setHeaderNumber:
	this->headerNumber = headerNumber;
	return true;
	// End of user code
}


unsigned char JausWrapper::getHeaderLength() const
{
	// Start of user code for accessor getHeaderLength:
	
	return headerLength;
	// End of user code
}

bool JausWrapper::setHeaderLength(unsigned char headerLength)
{
	// Start of user code for accessor setHeaderLength:
	this->headerLength = headerLength;
	return true;
	// End of user code
}


unsigned char JausWrapper::getHeaderFlags() const
{
	// Start of user code for accessor getHeaderFlags:
	
	return headerFlags;
	// End of user code
}

bool JausWrapper::setHeaderFlags(unsigned char headerFlags)
{
	// Start of user code for accessor setHeaderFlags:
	this->headerFlags = headerFlags;
	return true;
	// End of user code
}



// Class Methods
int JausWrapper::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int initialBufferSize = dst->remainingBytes();

	unsigned char tempByte(0);
	tempByte |= (type & 0x3F) << 2;
	tempByte |= headerFlags & 0xC0;
	dst->pack(tempByte);
	
	dst->pack(static_cast<unsigned short>(length()));

	if(headerFlags)
	{
		dst->pack(headerNumber);
		dst->pack(headerLength);
	}

	tempByte = static_cast<unsigned char>(priority) & 0x03;
	tempByte |= (static_cast<unsigned char>(broadcastFlag) & 0x03) << 2;
	tempByte |= (static_cast<unsigned char>(ackNak) & 0x03) << 4;
	tempByte |= (static_cast<unsigned char>(largeMessageFlag) & 0x03) << 6;
	dst->pack(tempByte);

	dst->pack(destination);

	dst->pack(source);

	dst->pack(*payload);

	dst->pack(sequenceNumber);

	return initialBufferSize - dst->remainingBytes();
	// End of user code
}


int JausWrapper::from(system::Buffer *src)
{
	// Start of user code for method from:
	int initialBufferSize = src->remainingBytes();

	unsigned char tempByte = 0;
	src->unpack(tempByte);
	type = static_cast<WrapperType>((tempByte >> 2) & 0x3F);
	headerFlags = tempByte & 0xC0; // Header flags are 2 most significant bits

	// dataSize = headerSize + payload->length() + footerSize;
	short dataSize;
	src->unpack(dataSize);

	short headerSize = 12; // Magic Number [12]: Minimum size of the header in bytes
	if(headerFlags)
	{
		headerSize += 2; // Magic Number [2]: Size of header compression fields in bytes
	}
	short footerSize = 2; // Magic Number [2]: Sequence Number size in bytes

	short length = dataSize - headerSize - footerSize;

	if(headerFlags)
	{
		src->unpack(headerNumber);
		src->unpack(headerLength);
	}

	src->unpack(tempByte); 
	priority = static_cast<Priority>(tempByte & 0x03);
	broadcastFlag = static_cast<BroadcastType>((tempByte >> 2)& 0x03);
	ackNak = static_cast<AckNakType>((tempByte >> 4)& 0x03);
	largeMessageFlag = static_cast<LargeMessageType>((tempByte >> 6)& 0x03);

	src->unpack(destination);

	src->unpack(source);

	if(!payload)
	{
		payload = new system::Buffer(length);
	}

	if(length)
	{
		dynamic_cast<system::Buffer *>(payload)->setMaxSize(length);
		std::memcpy(dynamic_cast<system::Buffer *>(payload)->getBuffer(), src->getPointer(), length);
		src->increment(length);
	}

	src->unpack(sequenceNumber);

	return initialBufferSize - src->remainingBytes();
	// End of user code
}


int JausWrapper::length()
{
	// Start of user code for method length:
	// dataSize = headerSize + payload->length() + footerSize;
	short headerSize = 12; // Magic Number [12]: Minimum size of the header in bytes
	if(headerFlags)
	{
		headerSize += 2; // Magic Number [2]: Size of header compression fields in bytes
	}
	short footerSize = 2; // Magic Number [2]: Sequence Number size in bytes
	
	short length = static_cast<short>(payload->length());

	return headerSize + length + footerSize;
	// End of user code
}




std::string JausWrapper::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const JausWrapper& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

