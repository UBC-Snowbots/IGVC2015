/**
\file JudpLargeMessageBuffer.h

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

#include "openjaus/transport/AS5669/JudpLargeMessageBuffer.h"
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
JudpLargeMessageBuffer::JudpLargeMessageBuffer() :
	lastSeqNum(0),
	first(NULL)
{
	buffer.clear();
}
// End of user code

// Start of user code for default destructor:
JudpLargeMessageBuffer::~JudpLargeMessageBuffer()
{
	buffer.clear();
}
// End of user code

const std::map< uint16_t, transport::Wrapper * >& JudpLargeMessageBuffer::getBuffer() const
{
	// Start of user code for accessor getBuffer:
	return buffer;
	// End of user code
}


uint16_t JudpLargeMessageBuffer::getLastSeqNum() const
{
	// Start of user code for accessor getLastSeqNum:
	return lastSeqNum;
	// End of user code
}


transport::Wrapper* JudpLargeMessageBuffer::getFirst() const
{
	// Start of user code for accessor getFirst:
	return first;
	// End of user code
}



// Class Methods
bool JudpLargeMessageBuffer::addWrapper(transport::Wrapper *wrapper)
{
	// Start of user code for method addWrapper:
	switch(wrapper->getLargeMessageFlag())
	{
		case openjaus::transport::FIRST_PACKET:
		{
			if(this->buffer.size() != 0)
			{
				// Received a first and have old data... flush
				LOG_DEBUG("Received Large Message FIRST_PACKET from " << wrapper->getSource() << " with unfinished previous message parts. Flushing buffer.");
				this->reset();
			}

			this->first = wrapper;
			this->buffer[wrapper->getSequenceNumber()] = wrapper;
			this->lastSeqNum = wrapper->getSequenceNumber();
			return true;
		}

		case openjaus::transport::INNER_PACKET:
		{
			if(this->buffer.find(wrapper->getSequenceNumber()) != this->buffer.end())
			{
				// This Sequence number already exists!
				uint16_t seqnum = wrapper->getSequenceNumber();
				delete wrapper;
				this->reset();
				THROW_EXCEPTION("INNER_PACKET with seqnum: " << seqnum << " received, but packet with that seqnum already exists! Flushing buffer & discarding packet.");
				return false;
			}

			if(this->first == NULL)
			{
				// Inner with no first!
				Address address = wrapper->getSource();
				delete wrapper;
				THROW_EXCEPTION("INNER_PACKET from " << address << " received, but no FIRST_PACKET. Discarding packet.");
				return false;
			}

			uint16_t seqnum = wrapper->getSequenceNumber();
			uint16_t expected = this->lastSeqNum+1;
			if(seqnum != expected)
			{
				// Packet out of order!
				delete wrapper;
				this->reset();
				THROW_EXCEPTION("INNER_PACKET with seqnum: " << seqnum << " received, but expected packet with seqnum "<< expected << "! Flushing buffer & discarding packet.");
				return false;
			}

			this->buffer[wrapper->getSequenceNumber()] = wrapper;
			this->lastSeqNum = wrapper->getSequenceNumber();
			return true;
		}

		case openjaus::transport::LAST_PACKET:
		{
			// DO NOT USE THIS FUNCTION FOR LAST PACKET
			delete wrapper;
			THROW_EXCEPTION("Do not use JudpLargeMessageBuffer::addWrapper for LAST_PACKET. Call JudpLargeMessageBuffer::assembleWrapper instead.");
			return false;
		}

		case openjaus::transport::SINGLE_PACKET:
		default:
		{
			delete wrapper;
			return false;
		}
	}

	// End of user code
}


transport::Wrapper* JudpLargeMessageBuffer::assembleWrapper(transport::Wrapper *last)
{
	// Start of user code for method assembleWrapper:

	// Check for valid input
	if(last == NULL)
	{
		THROW_EXCEPTION("JudpLargeMessageBuffer::assembleWrapper called with NULL wrapper.");
		return NULL;
	}

	if(last->getLargeMessageFlag() != openjaus::transport::LAST_PACKET)
	{
		delete last;
		THROW_EXCEPTION("JudpLargeMessageBuffer::assembleWrapper called without LAST_PACKET wrapper.");
		return NULL;
	}

	// Test non-empty buffer
	if(this->buffer.size() == 0)
	{
		// Empty Buffer!
		delete last;
		this->reset();
		THROW_EXCEPTION("JudpLargeMessageBuffer::assembleWrapper called with empty input buffer.");
		return NULL;
	}

	// Test valid first member
	if(this->first == NULL)
	{
		delete last;
		THROW_EXCEPTION("JudpLargeMessageBuffer::assembleWrapper called without first.");
		return NULL;
	}

	// Test for all packets & calculate total data size
	uint32_t dataSize = 0;
	uint16_t i = first->getSequenceNumber();
	while(i != last->getSequenceNumber())
	{
		std::map< uint16_t, transport::Wrapper * >::iterator itr = this->buffer.find(i);
		if(itr == this->buffer.end())
		{
			// Missing part
			delete last;
			this->reset();
			THROW_EXCEPTION("JudpLargeMessageBuffer missing packet with seqnum: " << i <<". Delete packet and reset.");
			return NULL;
		}
//		LOG_DEBUG("" << i << ": " << itr->second->getPayload()->length());
		dataSize += itr->second->getPayload()->length();
		i++;
	}

	dataSize += last->getPayload()->length();
//	LOG_DEBUG("Reassemble DataSize: " << dataSize);

	transport::Wrapper *output = new transport::Wrapper();
	output->setSequenceNumber(first->getSequenceNumber());
	output->setDestination(first->getDestination());
	output->setSource(first->getSource());
	output->setAckNak(first->getAckNak());
	output->setBroadcastFlag(first->getBroadcastFlag());
	output->setPriority(first->getPriority());
	output->setLargeMessageFlag(transport::SINGLE_PACKET);

	openjaus::system::Buffer *payload = new openjaus::system::Buffer(dataSize);

	// Re-assemble packet
	i = first->getSequenceNumber();
	while(i != last->getSequenceNumber())
	{
		std::map< uint16_t, transport::Wrapper * >::iterator itr = this->buffer.find(i);
		openjaus::system::Buffer *source = dynamic_cast<system::Buffer *>(itr->second->getPayload());
		source->reset();
		memcpy(payload->getPointer(), source->getBuffer(), source->length());
		payload->increment(source->length());

//		LOG_DEBUG(payload->toString());
//		LOG_DEBUG("Appended: " << source->length() );
//		LOG_DEBUG("Payload Size: " << payload->containedBytes() );
		i++;
	}

	// Last Packet
	openjaus::system::Buffer *source = dynamic_cast<system::Buffer *>(last->getPayload());
	source->reset();
	memcpy(payload->getPointer(), source->getBuffer(), source->length());
	payload->increment(source->length());

//	LOG_DEBUG("Appended: " << source->length());
//	LOG_DEBUG("Payload Size: " << payload->containedBytes());

	payload->reset();
	output->setPayload(payload);

	// Clear this memory
	this->reset();

	// return wrapper
	return output;
	// End of user code
}




std::string JudpLargeMessageBuffer::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const JudpLargeMessageBuffer& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
void JudpLargeMessageBuffer::reset()
{
	this->buffer.clear();
	this->first = NULL;
	this->lastSeqNum = 0;
}
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

