/**
\file Wrapper.h

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

#include "openjaus/transport/Wrapper.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{

// Start of user code for default constructor:
Wrapper::Wrapper() :
		ackNak(NO_ACKNACK),
		compressible(false),
		mustArrive(false),
		sequenceNumber(0),
		type(JAUS_MESSAGE),
		priority(STANDARD),
		broadcastFlag(NO_BROADCAST),
		largeMessageFlag(SINGLE_PACKET),
		destination(),
		payload(NULL),
		source(),
		transportData(NULL)
{
}
// End of user code

// Start of user code for default destructor:
Wrapper::~Wrapper()
{
	if(payload && dynamic_cast<Wrapper*>(this->payload) != this)
	{
		delete payload;
	}

}
// End of user code

AckNakType Wrapper::getAckNak() const
{
	// Start of user code for accessor getAckNak:
	
	return ackNak;
	// End of user code
}

bool Wrapper::setAckNak(AckNakType ackNak)
{
	// Start of user code for accessor setAckNak:
	this->ackNak = ackNak;
	return true;
	// End of user code
}


bool Wrapper::isCompressible() const
{
	// Start of user code for accessor getCompressible:
	
	return compressible;
	// End of user code
}

bool Wrapper::setCompressible(bool compressible)
{
	// Start of user code for accessor setCompressible:
	this->compressible = compressible;
	return true;
	// End of user code
}


bool Wrapper::isMustArrive() const
{
	// Start of user code for accessor getMustArrive:
	
	return mustArrive;
	// End of user code
}

bool Wrapper::setMustArrive(bool mustArrive)
{
	// Start of user code for accessor setMustArrive:
	this->mustArrive = mustArrive;
	return true;
	// End of user code
}


uint16_t Wrapper::getSequenceNumber() const
{
	// Start of user code for accessor getSequenceNumber:
	
	return sequenceNumber;
	// End of user code
}

bool Wrapper::setSequenceNumber(uint16_t sequenceNumber)
{
	// Start of user code for accessor setSequenceNumber:
	this->sequenceNumber = sequenceNumber;
	return true;
	// End of user code
}


WrapperType Wrapper::getType() const
{
	// Start of user code for accessor getType:
	
	return type;
	// End of user code
}

bool Wrapper::setType(WrapperType type)
{
	// Start of user code for accessor setType:
	this->type = type;
	return true;
	// End of user code
}


Priority Wrapper::getPriority() const
{
	// Start of user code for accessor getPriority:
	
	return priority;
	// End of user code
}

bool Wrapper::setPriority(Priority priority)
{
	// Start of user code for accessor setPriority:
	this->priority = priority;
	return true;
	// End of user code
}


BroadcastType Wrapper::getBroadcastFlag() const
{
	// Start of user code for accessor getBroadcastFlag:
	
	return broadcastFlag;
	// End of user code
}

bool Wrapper::setBroadcastFlag(BroadcastType broadcastFlag)
{
	// Start of user code for accessor setBroadcastFlag:
	this->broadcastFlag = broadcastFlag;
	return true;
	// End of user code
}


LargeMessageType Wrapper::getLargeMessageFlag() const
{
	// Start of user code for accessor getLargeMessageFlag:
	
	return largeMessageFlag;
	// End of user code
}

bool Wrapper::setLargeMessageFlag(LargeMessageType largeMessageFlag)
{
	// Start of user code for accessor setLargeMessageFlag:
	this->largeMessageFlag = largeMessageFlag;
	return true;
	// End of user code
}


const Address& Wrapper::getDestination() const
{
	// Start of user code for accessor getDestination:
	
	return destination;
	// End of user code
}

bool Wrapper::setDestination(const Address& destination)
{
	// Start of user code for accessor setDestination:
	this->destination = destination;
	return true;
	// End of user code
}


system::Transportable* Wrapper::getPayload() const
{
	// Start of user code for accessor getPayload:
	
	return payload;
	// End of user code
}

bool Wrapper::setPayload(system::Transportable* payload)
{
	// Start of user code for accessor setPayload:
	if(this->payload && dynamic_cast<Wrapper*>(this->payload) != this)
	{
		delete this->payload;
	}

	if(!payload)
	{
		this->payload = NULL;
		return true;
	}

	system::Buffer *newBuffer = new system::Buffer(payload->length());
	payload->to(newBuffer);
	newBuffer->reset();

	this->payload = newBuffer;

	return true;
	// End of user code
}


const Address& Wrapper::getSource() const
{
	// Start of user code for accessor getSource:
	
	return source;
	// End of user code
}

bool Wrapper::setSource(const Address& source)
{
	// Start of user code for accessor setSource:
	this->source = source;
	return true;
	// End of user code
}


TransportData* Wrapper::getTransportData() const
{
	// Start of user code for accessor getTransportData:
	
	return transportData;
	// End of user code
}

bool Wrapper::setTransportData(TransportData* transportData)
{
	// Start of user code for accessor setTransportData:
	this->transportData = transportData;
	return true;
	// End of user code
}



// Class Methods

int Wrapper::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int result = 0;

	return result;
	// End of user code
}

int Wrapper::from(system::Buffer *src)
{
	// Start of user code for method from:
	int result = 0;

	return result;
	// End of user code
}

int Wrapper::length()
{
	// Start of user code for method length:
	int result = 0;

	return result;
	// End of user code
}

int Wrapper::prioritizedValue()
{
	// Start of user code for method prioritizedValue:
	return static_cast<int>(priority);
	// End of user code
}


std::string Wrapper::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Wrapper& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace transport
} // namespace openjaus

