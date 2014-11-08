/**
\file Transport.cpp

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


#include "openjaus/core/Transport.h"
// Start of user code for additional headers:
#include "openjaus/transport/AS5669/ConfigurationWrapper.h"
#include "openjaus/transport/AS5669/JudpPacket.h"
#include "openjaus/transport/AS5669/JudpAddress.h"
#include "openjaus/transport/AS5669/JausWrapper.h"
// End of user code

namespace openjaus
{
namespace core
{

Transport::Transport() : 
	policyLoop(),
	receivingState(),
	receive()
{
	// Add Service Identification Data to implements list
	name = "Transport";
	
	model::Service *transportService = new model::Service();
	transportService->setName("Transport");
	transportService->setUri("urn:jaus:jss:core:Transport");
	transportService->setVersionMajor(1);
	transportService->setVersionMinor(0);
	this->implements->push_back(transportService);
	
	receivingState.setName("ReceivingState");
	

	policyLoop.setInterface(this);
	policyLoop.setTransportInterface(this);
	receive.addDefaultStateTransition(policyLoop);
	
	receive.setName("Receive");
	receive.addState(receivingState);
	receive.setStartingState(&receivingState);
	
	receiveThread.setStateMachine(&receive);
	runners.push_back(&receiveThread);
	
    
    
	// Start of user code for Constructor:
	systemTree = &model::SystemTree::instance();

	receive.addMessageCallback(&Transport::checkTransportPolicy, this);

	udpInterface = new transport::AS5669::JudpInterface(system::InetAddress::anyAddress(), system::InetAddress::ANY_PORT);
	udpInterface->setRecvQueue(const_cast<system::PriorityQueue*>(&receiveThread.getQueue()));
	udpInterface->setRecvQueue(transport::JAUS_MESSAGE, const_cast<system::PriorityQueue*>(&receiveThread.getQueue()));
	interfaces.push_back(udpInterface);

	tcpInterface = new transport::AS5669::JtcpInterface(system::InetAddress::anyAddress(), udpInterface->getUdpAddress()->getPort());
	tcpInterface->setRecvQueue(const_cast<system::PriorityQueue*>(&receiveThread.getQueue()));
	interfaces.push_back(tcpInterface);
	// End of user code
}

Transport::~Transport()
{
	// Start of user code for Destructor:

	// Terminate policies
	std::map< uint64_t, transport::Policy * >::iterator policyIt;
	for(policyIt = policyMap.begin(); policyIt != policyMap.end(); ++policyIt)
	{
		delete policyIt->second;
	}

	delete tcpInterface;
	delete udpInterface;
	// End of user code
}

bool Transport::enqueue(model::Trigger *trigger)
{
	// Start of user code for action enqueue(model::Trigger *trigger):
	return sendMessage(trigger);
	// End of user code
}

bool Transport::broadcastLocalEnqueue(model::Trigger *trigger)
{
	// Start of user code for action broadcastLocalEnqueue(model::Trigger *trigger):
	return broadcastToSubsystem(trigger);
	// End of user code
}

bool Transport::broadcastGlobalEnqueue(model::Trigger *trigger)
{
	// Start of user code for action broadcastGlobalEnqueue(model::Trigger *trigger):
	return broadcastToSystem(trigger);
	// End of user code
}

bool Transport::sendMessage(model::Trigger *trigger)
{
	// Start of user code for action sendMessage(model::Trigger *trigger):
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	if(!message)
	{
		THROW_EXCEPTION("Transport Service: Attempted to send non-message trigger");
	}

	transport::Wrapper* wrapper = NULL;

	// Wrap message with specific wrapper type
	switch(message->getType())
	{
		case transport::JAUS_MESSAGE:
		{
			if(!address.isValid())
			{
				LOG("Transport Service: Attempted to send message before component: " << name << ", had valid address");
				delete message;
				return false;
			}

			// Load send policy for this message
			transport::Policy *policy = policyMap[message->getDestination().getHash()];
			if(!policy)
			{
				policy = new transport::Policy();
				policyMap[message->getDestination().getHash()] = policy;
			}

			// TODO: Determine compression / encryption technique

			// Determine interface on which to send message
			if(policy->isTCPSupported())
			{
				switch(policy->getPreferredTCPUse())
				{
					case transport::LARGE_MESSAGES_ONLY:
						// TODO: Check for large message

						// break omitted on purpose, large messages case includes application required case

					case transport::APPLICATION_REQUIRED_ONLY:
						if(message->isMustArrive())
						{
							LOG_DEBUG("Transport Service: Sending message via JTCP interface");
							transport::AS5669::JausWrapper *jausWrapper = new transport::AS5669::JausWrapper();
							// TODO: Implement copy of all wrapper info: (ack/nak, etc...)
							// TODO: Check for the issue above in all transport exchanges in the code base
							jausWrapper->setSequenceNumber(message->getSequenceNumber());
							jausWrapper->setAckNak(message->getAckNak());
							jausWrapper->setBroadcastFlag(message->getBroadcastFlag());
							jausWrapper->setPriority(message->getPriority());
							jausWrapper->setDestination(message->getDestination());
							jausWrapper->setSource(address);
							jausWrapper->setPayload(message);
							delete message;
							return tcpInterface->send(jausWrapper);
						}
						break;

					default:
						break;
				}
			}

			// TODO: Large Message Processing
			// If message is to be sent via TCP then stream all messages direct through TCP
			// Else if message is to be sent via UDP then test for large message handling
			// 	- If message exceeds max UDP size then send to large message handler
			//  - Else send direct to JUDP interface

			transport::AS5669::JausWrapper *jausWrapper = new transport::AS5669::JausWrapper();
			wrapper = jausWrapper;
			break;
		}

		case transport::CONFIGURATION:
		{
			LOG_DEBUG(name << " sending configuration message: " << message);
			transport::AS5669::ConfigurationWrapper *configWrapper = new transport::AS5669::ConfigurationWrapper();
			configWrapper->setTransportData(message->getTransportData());
			wrapper = configWrapper;
			break;
		}

		default:
			THROW_EXCEPTION("Transport: attempted to send wrapper with invalid type: " << message);
			break;
	}

	wrapper->setSequenceNumber(message->getSequenceNumber());
	wrapper->setDestination(message->getDestination());
	wrapper->setSource(address);
	wrapper->setAckNak(message->getAckNak());
	wrapper->setBroadcastFlag(message->getBroadcastFlag());
	wrapper->setPriority(message->getPriority());
	wrapper->setPayload(message);
	delete message;

//	if(wrapper->getType() == transport::CONFIGURATION)
//	{
//		LOG_DEBUG(name << " Sending payload: " << ((system::Buffer*)wrapper->getPayload())->toString() )
//	}
	return udpInterface->send(wrapper);
	// End of user code
}

bool Transport::broadcastToNode(model::Trigger *trigger)
{
	// Start of user code for action broadcastToNode(model::Trigger *trigger):
	using namespace transport;
	Address broadcastAddress(address.getSubsystem(), address.getNode(), Address::ANY_COMPONENT);
	static_cast<model::Message *>(trigger)->setDestination(broadcastAddress);
	return sendMessage(trigger);
	// End of user code
}

bool Transport::broadcastToSubsystem(model::Trigger *trigger)
{
	// Start of user code for action broadcastToSubsystem(model::Trigger *trigger):
	LOG_DEBUG("JudpInterface: Subsystem Broadcast trigger: " << trigger);
	using namespace transport;
	Address broadcastAddress(address.getSubsystem(), Address::ANY_NODE, Address::ANY_COMPONENT);
	static_cast<model::Message *>(trigger)->setDestination(broadcastAddress);
	static_cast<model::Message *>(trigger)->setBroadcastFlag(LOCAL_BROADCAST);
	return sendMessage(trigger);
	// End of user code
}

bool Transport::broadcastToSystem(model::Trigger *trigger)
{
	// Start of user code for action broadcastToSystem(model::Trigger *trigger):
	using namespace transport;
	Address broadcastAddress(Address::ANY_SUBSYSTEM, Address::ANY_NODE, Address::ANY_COMPONENT);
	static_cast<model::Message *>(trigger)->setDestination(broadcastAddress);
	static_cast<model::Message *>(trigger)->setBroadcastFlag(GLOBAL_BROADCAST);
	return sendMessage(trigger);
	// End of user code
}

bool Transport::checkTransportPolicy(model::Trigger *trigger)
{
	// Start of user code for action checkTransportPolicy(model::Trigger *trigger):
	transport::Wrapper* wrapper = dynamic_cast<transport::Wrapper *>(trigger);
	if(!wrapper)
	{
		return false;
	}

	if(wrapper->getType() != transport::JAUS_MESSAGE)
	{
		return false;
	}

	model::Message *message = dynamic_cast<model::Message *>(wrapper);
	if(!message)
	{
		THROW_EXCEPTION("Transport Service: Received invalid JAUS message");
	}

	// Check policyMap
	transport::Policy *policy;
	if(policyMap[message->getSource().getHash()] == NULL)
	{
		policy = new transport::Policy();
		policyMap[message->getSource().getHash()] = policy;
	}
	else
	{
		policy = policyMap[message->getSource().getHash()];
	}

	// Check conditions to send Query
	if(	policy->isConfirmed() == false &&
		policy->getRequestCount() < 3 && // Magic Number (3): Number of times to try to establish a transport policy
		system::Time::getTime().getSeconds() > policy->getTimeout().getSeconds())
	{
		LOG_DEBUG("Transport Service: Sending QueryTransportPolicy to: " << message->getSource());

		// Send Policy Query
		core::QueryTransportPolicy *queryPolicy = new core::QueryTransportPolicy();
		queryPolicy->setDestination(message->getSource());
		sendMessage(queryPolicy);

		// Increment the request count
		policy->incrementRequestCount();

		// Configure 1 Second timeout between requests
		// This keeps the system from flooding a receiver with requests
		policy->resetTimeout();
	}
	return false;
	// End of user code
}

core::ReportTransportPolicy Transport::getReportTransportPolicy(QueryTransportPolicy *queryTransportPolicy)
{
	// Start of user code for action getReportTransportPolicy(QueryTransportPolicy *queryTransportPolicy):
	LOG_DEBUG("Transport Service: Processing QueryTransportPolicy");

	core::ReportTransportPolicy message;

	// Set policy data in message
	message.setSupportOJWrappers(true);
	message.setSupportTCP(true);
	message.setPreferenceTCP(core::PreferenceTCPEnumeration::LARGE_MESSAGES_ONLY);
	// TODO: Load preference from config file

	return message;
	// End of user code
}

bool Transport::storeTransportPolicy(ReportTransportPolicy *reportTransportPolicy)
{
	// Start of user code for action storeTransportPolicy(ReportTransportPolicy *reportTransportPolicy):
	LOG_DEBUG("Transport Service: Processing ReportTransportPolicy");

	transport::Policy *policy = policyMap[reportTransportPolicy->getSource().getHash()];
	if(!policy)
	{
		THROW_EXCEPTION("Transport Service: Received unsolicited report transport policy");
	}
	policy->setConfirmed(true);
	policy->setOpenJAUSSupported(reportTransportPolicy->getSupportOJWrappers());
	policy->setTCPSupported(reportTransportPolicy->getSupportTCP());
	policy->setPreferredTCPUse(static_cast<transport::TCPPreference>(reportTransportPolicy->getPreferenceTCP()));

	return true;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

