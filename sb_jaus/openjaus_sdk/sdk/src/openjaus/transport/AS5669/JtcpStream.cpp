/**
\file JtcpStream.h

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

#include "openjaus/transport/AS5669/JtcpStream.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/transport/AS5669/JausWrapper.h"
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

// Start of user code for default constructor:
JtcpStream::JtcpStream() :
	system::Buffer(),
	version(0),
	versionSent(false),
	packet(DEFAULT_SIZE)
{
	setMaxSize(DEFAULT_SIZE);
}
// End of user code

// Start of user code for default destructor:
JtcpStream::~JtcpStream()
{
}
// End of user code

unsigned char JtcpStream::getVersion() const
{
	// Start of user code for accessor getVersion:
	return version;
	// End of user code
}


bool JtcpStream::isVersionSent() const
{
	// Start of user code for accessor getVersionSent:
	return versionSent;
	// End of user code
}


const system::Packet& JtcpStream::getPacket() const
{
	// Start of user code for accessor getPacket:
	
	return packet;
	// End of user code
}


const TCPAddress& JtcpStream::getAddress() const
{
	// Start of user code for accessor getAddress:
	
	return address;
	// End of user code
}

bool JtcpStream::setAddress(const TCPAddress& address)
{
	// Start of user code for accessor setAddress:
	this->address = address;
	return true;
	// End of user code
}



// Class Methods
transport::WrapperType JtcpStream::nextWrapperType()
{
	// Start of user code for method nextWrapperType:
	transport::WrapperType result = transport::JAUS_MESSAGE;

	return result;
	// End of user code
}


transport::Wrapper* JtcpStream::popWrapper()
{
	// Start of user code for method popWrapper:
	int initialSize = containedBytes();
	if(!initialSize)
	{	// Nothing to pop from
		return NULL;
	}

	system::Buffer::reset();

	if(version == 0)
	{
		unpack(version);
		LOG_DEBUG("JtcpPacket: Version = " << static_cast<int>(version));

		if(version != STANDARD_VERSION)
		{
			THROW_EXCEPTION("JtcpStream: Data indicates unsupported version: " << version);
		}
	}

	unsigned char tempByte = 0;
	unpack(tempByte);
	WrapperType type = static_cast<WrapperType>((tempByte >> 2) & 0x3F);
	LOG_DEBUG("JtcpPacket: Wrapper type: " << type);

	switch(type)
	{
		case CONFIGURATION:
		{
			THROW_EXCEPTION("JtcpPacket: Received configuration message via TCP");
		}

		case JAUS_MESSAGE:
		{
			short length;
			unpack(length);
			LOG_DEBUG("JtcpPacket: Length = " << length);

			int parsedBytes = pointer - buffer;
			int remainingBytes = initialSize - parsedBytes;
			int remainingHeaderBytes = 11; // Number of bytes still to be parsed before payload of size "length";
			if(remainingBytes < length + remainingHeaderBytes)
			{	// No data to be parsed yet
				pointer = buffer + initialSize; // Reset buffer to initial state
				return NULL;
			}

			pointer -= 3; // Rewind buffer to beginning of JausWrapper

			// Parse JAUS Standard wrapper:
			JausWrapper *jausWrapper = new JausWrapper();
			jausWrapper->from(this);

			// Move remaining bytes to front of buffer
			parsedBytes = pointer - buffer;
			remainingBytes = initialSize - parsedBytes;
			if(remainingBytes)
			{
				LOG_DEBUG("JtcpPacket: Copying " << remainingBytes << " remaining bytes to beginning of buffer");
				memcpy(buffer, pointer, remainingBytes);
			}
			pointer = buffer + remainingBytes;

			return jausWrapper;
		}

		case OPENJAUS_MESSAGE:
		{
			// TODO: Parse OJ Wrapper
			break;
		}

	}

	return NULL;
	// End of user code
}


system::Packet& JtcpStream::packetize(transport::Wrapper &wrapper)
{
	// Start of user code for method packetize:
	packet.reset();

	// TODO: Resize packet based on length of wrapper

	if(!versionSent)
	{
		LOG_DEBUG("Sending version: ")
		versionSent = true;
		uint8_t sendVersion = STANDARD_VERSION;
		packet.pack(sendVersion);
	}

	wrapper.to(&packet);

	packet.setAddress(address.getIpAddress());
	packet.setPort(address.getPort());

	return packet;
	// End of user code
}




std::string JtcpStream::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "JtcpStream: Version: " << version;
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const JtcpStream& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

