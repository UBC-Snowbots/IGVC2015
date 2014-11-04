/**
\file Interface.h

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

#include "openjaus/transport/Interface.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Logger.h"
// End of user code

namespace openjaus
{
namespace transport
{

// Start of user code for default constructor:
Interface::Interface() :
		recvQueues(),
		sequenceNumber(0),
		recvQueue(NULL),
		sendQueue(),
		recvThread(),
		sendThread(),
		subsIf(NULL),
		nodeIf(NULL),
		address(NULL)
{
	recvThread.setThreadFunction(THREAD_METHOD(Interface,recvThreadMethod), this);
	sendThread.setThreadFunction(THREAD_METHOD(Interface,sendThreadMethod), this);
}
// End of user code

// Start of user code for default destructor:
Interface::~Interface()
{
}
// End of user code

const std::map< WrapperType, system::PriorityQueue * >& Interface::getRecvQueues() const
{
	// Start of user code for accessor getRecvQueues:
	
	return recvQueues;
	// End of user code
}


uint16_t Interface::getSequenceNumber() const
{
	// Start of user code for accessor getSequenceNumber:
	
	return sequenceNumber;
	// End of user code
}

bool Interface::setSequenceNumber(uint16_t sequenceNumber)
{
	// Start of user code for accessor setSequenceNumber:
	this->sequenceNumber = sequenceNumber;
	return true;
	// End of user code
}


system::PriorityQueue* Interface::getRecvQueue() const
{
	// Start of user code for accessor getRecvQueue:
	
	return recvQueue;
	// End of user code
}

bool Interface::setRecvQueue(system::PriorityQueue* recvQueue)
{
	// Start of user code for accessor setRecvQueue:
	this->recvQueue = recvQueue;
	return true;
	// End of user code
}


const system::PriorityQueue& Interface::getSendQueue() const
{
	// Start of user code for accessor getSendQueue:
	
	return sendQueue;
	// End of user code
}


const system::Thread& Interface::getRecvThread() const
{
	// Start of user code for accessor getRecvThread:
	
	return recvThread;
	// End of user code
}


const system::Thread& Interface::getSendThread() const
{
	// Start of user code for accessor getSendThread:
	
	return sendThread;
	// End of user code
}


system::NetworkInterface* Interface::getSubsIf() const
{
	// Start of user code for accessor getSubsIf:
	
	return subsIf;
	// End of user code
}

bool Interface::setSubsIf(system::NetworkInterface* subsIf)
{
	// Start of user code for accessor setSubsIf:
	this->subsIf = subsIf;
	return true;
	// End of user code
}


system::NetworkInterface* Interface::getNodeIf() const
{
	// Start of user code for accessor getNodeIf:
	
	return nodeIf;
	// End of user code
}

bool Interface::setNodeIf(system::NetworkInterface* nodeIf)
{
	// Start of user code for accessor setNodeIf:
	this->nodeIf = nodeIf;
	return true;
	// End of user code
}


Address* Interface::getAddress() const
{
	// Start of user code for accessor getAddress:
	
	return address;
	// End of user code
}

bool Interface::setAddress(Address* address)
{
	// Start of user code for accessor setAddress:
	this->address = address;
	return true;
	// End of user code
}



// Class Methods
bool Interface::send(Wrapper *message)
{
	// Start of user code for method send:
	this->sendQueue.push(message);

	return true;
	// End of user code
}


bool Interface::open(Address tAddress)
{
	// Start of user code for method open:
	bool result = 0;

	return result;
	// End of user code
}


bool Interface::close()
{
	// Start of user code for method close:
	bool result = 0;

	return result;
	// End of user code
}


bool Interface::broadcast(Wrapper message)
{
	// Start of user code for method broadcast:
	bool result = 0;

	return result;
	// End of user code
}


void Interface::run()
{
	// Start of user code for method run:
	recvThread.create();
	sendThread.create();
	// End of user code
}


void Interface::stop()
{
	// Start of user code for method stop:
	recvThread.join();
	sendThread.join();
	// End of user code
}


bool Interface::setRecvQueue(WrapperType type, system::PriorityQueue *queue)
{
	// Start of user code for method setRecvQueue:
	recvQueues[type] = queue;
	return true;
	// End of user code
}


bool Interface::checkAddresses(Wrapper *message)
{
	// Start of user code for method checkAddresses:
	transport::Address source = message->getSource();
	transport::Address destination = message->getDestination();

	if(source == *this->address)
	{
		// Message from myself
		LOG("Message from myself? Source: " << source.toString() << " My address: " << this->address->toString());
		return false;
	}

	if(source.getSubsystem() == transport::Address::ANY_SUBSYSTEM)
	{
		// Message from a broadcast subsystem? Invalid.
		LOG("Message received from " << source.toString() << ". ANY_SUBSYSTEM (" << transport::Address::ANY_SUBSYSTEM << ") not allowed in source address.");
		return false;
	}

	if(source.getNode() == transport::Address::ANY_NODE)
	{
		// Message from a broadcast subsystem? Invalid.
		LOG("Message received from " << source.toString() << ". ANY_NODE (" << transport::Address::ANY_NODE << ") not allowed in source address.");
		return false;
	}

	if(source.getComponent() == transport::Address::ANY_COMPONENT)
	{
		// Message from a broadcast subsystem? Invalid.
		LOG("Message received from " << source.toString() << ". ANY_COMPONENT (" << transport::Address::ANY_COMPONENT << ") not allowed in source address.");
		return false;
	}

	uint16_t subs = destination.getSubsystem();
	uint8_t node = destination.getNode();
	uint8_t cmpt = destination.getComponent();

	bool cmptValid = false;
	bool subsValid = false;
	bool nodeValid = false;

	if(cmpt == Address::ANY_COMPONENT || cmpt == this->address->getComponent())
	{
		cmptValid = true;
	}

	if(node == Address::ANY_NODE|| node == this->address->getNode())
	{
		nodeValid = true;
	}

	if(subs == Address::ANY_SUBSYSTEM || subs == this->address->getSubsystem())
	{
		subsValid = true;
	}

	if(!(cmptValid && nodeValid && subsValid))
	{
		// Message for a different JAUS Address
		LOG("Message received from " << source.toString() << " to " << destination.toString() << ". Not for me? My address is: " << this->address->toString());
		return false;
	}

	return true;
	// End of user code
}




std::string Interface::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Interface& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
void * Interface::recvThreadMethod()
{
	return NULL;
}

void * Interface::sendThreadMethod()
{
	return NULL;
}
// End of user code

} // namespace transport
} // namespace openjaus

