/**
\file JudpPacket.h

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

#include "openjaus/transport/AS5669/JudpPacket.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/transport/AS5669/ConfigurationWrapper.h"
#include "openjaus/transport/AS5669/JausWrapper.h"
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

// Start of user code for default constructor:
JudpPacket::JudpPacket() :
		system::Packet(),
		version(STANDARD_VERSION),
		wrappers()
{
	setMaxSize(MAX_PACKET_SIZE);
}
// End of user code

// Start of user code for default destructor:
JudpPacket::~JudpPacket()
{
	free();
}
// End of user code

unsigned char JudpPacket::getVersion() const
{
	// Start of user code for accessor getVersion:
	
	return version;
	// End of user code
}


const std::vector< transport::Wrapper* >& JudpPacket::getWrappers() const
{
	// Start of user code for accessor getWrappers:
	
	return wrappers;
	// End of user code
}

bool JudpPacket::setWrappers(const transport::Wrapper& wrappers)
{
	// Start of user code for accessor setWrappers:
	return true;
	// End of user code
}



// Class Methods
transport::WrapperType JudpPacket::nextWrapperType()
{
	// Start of user code for method nextWrapperType:
	unsigned char tempByte = *pointer;
	return static_cast<WrapperType>((tempByte >> 2) & 0x3F);
	// End of user code
}


transport::Wrapper* JudpPacket::popWrapper()
{
	// Start of user code for method popWrapper:
	if(wrappers.size() == 0)
	{
		unpack(version);

		if(version != STANDARD_VERSION)
		{
			THROW_EXCEPTION("JudpPacket: Data indicates unsupported version: " << version);
		}
	}

	switch(nextWrapperType())
	{
		case CONFIGURATION:
		{
			ConfigurationWrapper *configWrapper = new ConfigurationWrapper();
			LOG_DEBUG("Created Config Wrapper: " << configWrapper);
			configWrapper->from(this);
			wrappers.push_back(configWrapper);
			return configWrapper;
		}

		case JAUS_MESSAGE:
		{
			// Parse UDP Packet wrapper:
			JausWrapper *jausWrapper = new JausWrapper();
			jausWrapper->from(this);
			wrappers.push_back(jausWrapper);
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


bool JudpPacket::pushWrapper(transport::Wrapper *newWrapper)
{
	// Start of user code for method pushWrapper:
	if(wrappers.size() == 0)
	{
		pack(version);
	}

	wrappers.push_back(newWrapper);

	newWrapper->to(this);

	return true;
	// End of user code
}


unsigned char * JudpPacket::reset()
{
	// Start of user code for method reset:
	wrappers.clear();

	return system::Buffer::reset();
	// End of user code
}




std::string JudpPacket::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "JudpPacket: Version: " << version;
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const JudpPacket& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

