/**
\file JudpInterface.h

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

#include "openjaus/transport/AS5669/JudpInterface.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/transport/AddressMap.h"
#include "openjaus/transport/AS5669/ConfigurationWrapper.h"
#include "openjaus/transport/AS5669/JudpPacket.h"
#include "openjaus/transport/AS5669/JudpAddress.h"
#include "openjaus/transport/AS5669/JausWrapper.h"
#include "openjaus/transport/AS5669/TCPAddress.h"
#include "openjaus/system/Configuration.h"
#include "openjaus/system/Application.h"
#include "openjaus/system/Time.h"
#include "openjaus/system/NetworkInterface.h"
#include "openjaus/system/Logger.h"
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

// Start of user code for default constructor:
JudpInterface::JudpInterface()  :
	datagramSocket(),
	udpAddress(NULL),
	discoveryThread()
{
	discoveryThread.setThreadFunction(THREAD_METHOD(JudpInterface,discoveryThreadMethod), this);
	this->largeMessageMap.clear();
}
// End of user code

// Start of user code for default destructor:
JudpInterface::~JudpInterface()
{
	Wrapper *wrapper = dynamic_cast<Wrapper*>(sendQueue.pop());
	while(wrapper != NULL)
	{
		delete wrapper;
		wrapper = dynamic_cast<Wrapper*>(sendQueue.pop());
	}

	if(udpAddress)
	{
		delete udpAddress;
	}
}
// End of user code

const std::map< uint32_t, std::list< transport::CompressedHeader * > >& JudpInterface::getHeaderMap() const
{
	// Start of user code for accessor getHeaderMap:
	
	return headerMap;
	// End of user code
}

bool JudpInterface::setHeaderMap(std::map< uint32_t, std::list< transport::CompressedHeader * > > headerMap)
{
	// Start of user code for accessor setHeaderMap:
	this->headerMap = headerMap;
	return true;
	// End of user code
}


const system::DatagramSocket& JudpInterface::getDatagramSocket() const
{
	// Start of user code for accessor getDatagramSocket:
	
	return datagramSocket;
	// End of user code
}

JudpAddress* JudpInterface::getUdpAddress() const
{
	// Start of user code for accessor getUdpAddress:

	return udpAddress;
	// End of user code
}

bool JudpInterface::setUdpAddress(JudpAddress* udpAddress)
{
	// Start of user code for accessor setUdpAddress:
	this->udpAddress = udpAddress;
	return true;
	// End of user code
}


const system::Thread& JudpInterface::getDiscoveryThread() const
{
	// Start of user code for accessor getDiscoveryThread:
	
	return discoveryThread;
	// End of user code
}


const system::MulticastSocket& JudpInterface::getDiscoverySocket() const
{
	// Start of user code for accessor getDiscoverySocket:
	
	return discoverySocket;
	// End of user code
}



// Class Methods
void JudpInterface::run()
{
	// Start of user code for method run:
	Interface::run();

	discoveryThread.create();
	// End of user code
}


void JudpInterface::stop()
{
	// Start of user code for method stop:
	recvThread.setRunning(false);
	sendThread.setRunning(false);
	sendQueue.signalAll();

	discoveryThread.join();

	Interface::stop();
	// End of user code
}


std::list< JudpAddress * > JudpInterface::getDestinationAddresses(transport::Wrapper *wrapper)
{
	// Start of user code for method getDestinationAddresses:
	std::list<JudpAddress *> addresses;
	std::string broadcastMethod = system::Application::setting<std::string>("BroadcastMethod", "BroadcastIP");
	system::Application::comment("BroadcastMethod", "Determines how broadcast messages are sent on JUDP transport layer. Options are: BroadcastIP, Multicast or Both");

	if(subsIf && static_cast<unsigned short>(wrapper->getDestination().getSubsystem()) == Address::ANY_SUBSYSTEM)
	{
		JudpAddress *address = new JudpAddress();
		address->setIpAddress(subsIf->getBroadcast());
		address->setPort(JudpAddress::STANDARD_PORT);
		addresses.push_back(address);
	}
	else if(wrapper->getDestination().getNode() == Address::ANY_NODE)
	{
		if(broadcastMethod == "Multicast" || broadcastMethod == "Both")
		{
			JudpAddress *address = new JudpAddress();
			std::string multicastAddressStr = system::Application::setting< std::string >("MulticastAddress", "239.255.0.1");
			system::Application::comment("MulticastAddress", "For Multicast messages, this defines the address of the Multicast group to send messages. Default Value: 239.255.0.1");
			system::InetAddress multicastAddress = system::InetAddress::getByName(multicastAddressStr);
			address->setIpAddress(multicastAddress);
			address->setPort(JudpAddress::STANDARD_PORT);
			addresses.push_back(address);
		}

		if(broadcastMethod == "BroadcastIP" || broadcastMethod == "Both")
		{

			const std::vector<system::NetworkInterface*>& interfaces = datagramSocket.getInterfaces();
			// TODO: Implement in socket class getInternodeBroadcasts (loopback name needs OS abstraction)
			for(unsigned int i = 0; i < interfaces.size(); ++i)
			{
				// Exclude the subsystem interface (if one exists) and loopback (which is always cmpt interface)
				if(	(subsIf && subsIf->getName().compare(interfaces[i]->getName()) == 0) ||
					interfaces[i]->getName().compare("lo")	== 0)
				{
					continue;
				}

				JudpAddress *address = new JudpAddress();
				address->setIpAddress(interfaces[i]->getBroadcast());
				address->setPort(JudpAddress::STANDARD_PORT);
				addresses.push_back(address);
			}
		}
	}
	else if(wrapper->getDestination().getComponent() == Address::ANY_COMPONENT)
	{

		// DK: 4/30/2012: Disable Multicast for Component broadcast, causes node-level messages to be sent to external nodes
//		if(broadcastMethod == "Multicast" || broadcastMethod == "Both")
//		{
//			std::string multicastAddressStr = system::Application::setting< std::string >("MulticastAddress", "239.255.0.1");
//			system::Application::comment("MulticastAddress", "For Multicast messages, this defines the address of the Multicast group to send messages. Default Value: 239.255.0.1");
//
//			system::InetAddress multicastAddress = system::InetAddress::getByName(multicastAddressStr);
//
//			JudpAddress *address = new JudpAddress();
//			address->setIpAddress(multicastAddress);
//			address->setPort(JudpAddress::STANDARD_PORT);
//			addresses.push_back(address);
//		}

		// Send to standard port on loopback interface
		JudpAddress *address = new JudpAddress();
		address->setIpAddress(system::InetAddress::getByName("127.255.255.255"));
		address->setPort(JudpAddress::STANDARD_PORT);
		addresses.push_back(address);
	}
	else
	{
		switch(wrapper->getType())
		{
			case JAUS_MESSAGE:
			{
				// Lookup UDP address from JAUS address
				JudpAddress *address = new JudpAddress();
				if(AddressMap::getTransportData(wrapper->getDestination(), *address))
				{
					addresses.push_back(address);
				}
				else
				{
					delete address;
					THROW_EXCEPTION("JUDP Interface attempted to send message to unknown address: " << wrapper->getDestination());
				}
				break;
			}

			case CONFIGURATION:
			{
				if(!wrapper->getTransportData())
				{
					THROW_EXCEPTION("JUDP Interface: Attempted to send configuration message without transport data");
				}

				JudpAddress *address = dynamic_cast<JudpAddress *>(wrapper->getTransportData());
				if(!address)
				{
					THROW_EXCEPTION("JUDP Interface: Attempted to send configuration message without transport data");
				}
				addresses.push_back(address);
				break;
			}

			case OPENJAUS_MESSAGE:
			{
				// TODO: Populate this
				break;
			}
		}

	}

	return addresses;
	// End of user code
}




std::string JudpInterface::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const JudpInterface& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
JudpInterface::JudpInterface(system::InetAddress ipAddress, short port) :
		transport::Interface(),
		datagramSocket(),
		udpAddress(NULL),
		discoveryThread(),
		discoverySocket()
{
	discoveryThread.setThreadFunction(THREAD_METHOD(JudpInterface,discoveryThreadMethod), this);

	datagramSocket.enableBroadcast(true);
	datagramSocket.setBufferSize(131072); // 1 MB
	datagramSocket.open(ipAddress, port);
	LOG("Opened JUDP Interface: " << datagramSocket);

	udpAddress = new JudpAddress();
	udpAddress->setIpAddress(ipAddress);
	udpAddress->setPort(datagramSocket.getPort());

	discoverySocket.reuseAddress(true);
	discoverySocket.enableBroadcast(true);
	discoverySocket.open(system::InetAddress::anyAddress(), JudpAddress::STANDARD_PORT);
	LOG("Opened Discovery Interface: " << discoverySocket);
	bool multicastDiscovery = system::Application::setting<bool>("EnableMulticastDiscovery", false);
	system::Application::comment("EnableMulticastDiscovery", "If enabled, the JUDP Transport layer will join the multicast group defined by \"MulticastAddress\".");
	if(multicastDiscovery)
	{
		std::string multicastAddressStr = system::Application::setting< std::string >("MulticastAddress", "239.255.0.1");
		system::Application::comment("MulticastAddress", "For Multicast messages, this defines the address of the Multicast group to send messages. Default Value: 239.255.0.1");
		system::InetAddress multicastAddress = system::InetAddress::getByName(multicastAddressStr);
		discoverySocket.joinGroup(multicastAddress);
	}

	// Determine Would-be Default Subsystem Interface
	std::string inferfaceNames;
	system::NetworkInterface *defaultSubsIf = NULL;
	const std::vector<system::NetworkInterface*>& interfaces = datagramSocket.getInterfaces();
	for(unsigned int i = 0; i < interfaces.size(); ++i)
	{
		if(interfaces[i]->getName().compare("lo"))
		{
			defaultSubsIf = interfaces[i];
			inferfaceNames += interfaces[i]->getName() + ", ";
		}
	}
	inferfaceNames += "or None";

	// Attempt to load subsystem interface name from configuration
	std::string subsIfName = system::Application::setting("SubsystemInterface", defaultSubsIf? defaultSubsIf->getName() : "None");
	system::Application::comment("SubsystemInterface", "The network interface to use as the JAUS Subsystem Gateway. Options are: " + inferfaceNames);
	if(subsIfName.compare("None"))
	{
		for(unsigned int i = 0; i < interfaces.size(); ++i)
		{
			if(!interfaces[i]->getName().compare(subsIfName))
			{
				subsIf = interfaces[i];
				LOG("Setting Subsystem Interface: " << subsIf->getName());
				break;
			}
		}

		if(!subsIf)
		{
			LOG("Opened JUDP Interface: No Subsystem Interface: " << subsIfName);
		}
	}
	else
	{
		LOG("Opened JUDP Interface: Subsystem Interface Disabled.");
		subsIf = NULL;
	}

	// Determine Would-be Default Node Interface
	system::NetworkInterface *defaultNodeIf = NULL;
	for(unsigned int i = 0; i < interfaces.size(); ++i)
	{	// The default node interface is not the loopback or the subsystem interface
		if(	interfaces[i]->getName().compare("lo") && // is not the loopback interface
			(!subsIf || subsIf->getName().compare( interfaces[i]->getName()) ) ) // is not the subs interface
		{
			defaultNodeIf = interfaces[i];
			break;
		}
	}

	// Attempt to load node interface name from configuration
	std::string defaultNodeIfName;
	if(defaultNodeIf)
	{
		defaultNodeIfName = defaultNodeIf->getName();
	}
	else
	{
		defaultNodeIfName = "None";
	}

	std::string nodeIfName = system::Application::setting("NodeInterface", defaultNodeIfName);
	system::Application::comment("NodeInterface", "The network interface to use as the JAUS Node Interface. Options are: " + inferfaceNames);
	if(nodeIfName.compare("None"))
	{
		for(unsigned int i = 0; i < interfaces.size(); ++i)
		{
			if(!interfaces[i]->getName().compare(nodeIfName))
			{
				nodeIf = interfaces[i];
				LOG("Setting Node Interface: " << nodeIf->getName());
				break;
			}
		}

		if(!nodeIf)
		{
			LOG("Opened JUDP Interface: No Node Interface: " << nodeIfName);
		}
	}
	else
	{
		LOG("Opened JUDP Interface: Node Interface Disabled.");
	}
}

void * JudpInterface::recvThreadMethod()
{
	JudpPacket packet;

	system::Time timeout;
	timeout.setMicroseconds(250000);
	datagramSocket.setTimeout(timeout);

	while(recvThread.isRunning())
	{
		packet.reset();
		if(!datagramSocket.receive(packet))
		{	// Did not receive any packet data so try again
			continue;
		}

		try
		{
			receiveJudpPacket(packet);
		}
		catch(openjaus::system::Exception& exp)
		{
			system::Logger::log(exp);
			throw exp;
		}
	}

	packet.free();

	return NULL;
}

void * JudpInterface::sendThreadMethod()
{
	while(sendThread.isRunning())
	{
		Wrapper *wrapper = dynamic_cast<Wrapper*>(sendQueue.pop());
		if(wrapper == NULL)
		{
			sendQueue.timedWait(system::Condition::DEFAULT_WAIT_MSEC);
			continue;
		}

		sendMessage(wrapper);
		delete wrapper;
	}

	return NULL;
}

void * JudpInterface::discoveryThreadMethod()
{
	JudpPacket packet;

	system::Time timeout;
	timeout.setMicroseconds(250000);
	discoverySocket.setTimeout(timeout);

	while(discoveryThread.isRunning())
	{
		packet.reset();
		if(!discoverySocket.receive(packet))
		{	// Did not receive any packet data so try again
			continue;
		}

		// TODO: Decompress

		try
		{
			receiveJudpPacket(packet);
		}
		catch(openjaus::system::Exception& exp)
		{
			system::Logger::log(exp);
		}

	}

	packet.free();

	return NULL;
}

void JudpInterface::receiveJudpPacket(JudpPacket &packet)
{
	if(	packet.getPort() == datagramSocket.getPort() &&	datagramSocket.isLocalAddress(packet.getAddress()) )
	{	// Ignore packets sent from this component
		LOG_DEBUG("Ignoring Packet From Self: " << packet.getAddress());
		return;
	}

	// Manage received packet
	// TODO: while( packet.popWrapper() )
	Wrapper* wrapper = packet.popWrapper();

	switch(wrapper->getType())
	{
		case CONFIGURATION:
		{
			JudpAddress *udpAddress = new JudpAddress();
			udpAddress->setIpAddress(packet.getAddress());
			udpAddress->setPort(packet.getPort());
			wrapper->setTransportData(udpAddress);
			recvQueues[CONFIGURATION]->push(wrapper);
			break;
		}

		case JAUS_MESSAGE:
		{
			// Add address to map
			JudpAddress *udpAddress = new JudpAddress();
			udpAddress->setIpAddress(packet.getAddress());
			udpAddress->setPort(packet.getPort());
			AddressMap::setTransportData( wrapper->getSource(), *udpAddress);

			// Store component TCP address if it does not exist in the AddressMap
			// If it does exist, then it may be different than UDP address so
			// do not overwrite.
			TCPAddress tcpAddress;
			if(!AddressMap::getTransportData(wrapper->getSource(), tcpAddress))
			{
				tcpAddress.setIpAddress(packet.getAddress());
				tcpAddress.setPort(packet.getPort());
				AddressMap::setTransportData( wrapper->getSource(), tcpAddress);
			}

			wrapper->setTransportData(udpAddress);

			LOG_DEBUG("JudpInterface: Received JAUS Wrapper from: " << udpAddress->toString());

			if(wrapper->getLargeMessageFlag() == openjaus::transport::SINGLE_PACKET)
			{
				recvQueues[JAUS_MESSAGE]->push(wrapper);
			}
			else
			{
				handleLargeMessagePart(wrapper);
			}
			break;
		}

		case OPENJAUS_MESSAGE:
		{
			// TODO: Decompress
			break;
		}

		default:
			THROW_EXCEPTION("JudpInterface: Received invalid wrapper type from: " << packet.getAddress());
			break;
	}
}

void JudpInterface::handleLargeMessagePart(Wrapper *wrapper)
{
//	LOG_DEBUG("handleLargeMessagePart");
//	LOG_DEBUG("Type: " << wrapper->getLargeMessageFlag());
//	LOG_DEBUG("SeqNum: " << wrapper->getSequenceNumber());

	switch(wrapper->getLargeMessageFlag())
	{
		case openjaus::transport::FIRST_PACKET:
		{
			if(this->largeMessageMap.find(wrapper->getSource()) != this->largeMessageMap.end())
			{
				// Add first... this will "reset" the JudpLargeMessageBuffer internally
				this->largeMessageMap.find(wrapper->getSource())->second->addWrapper(wrapper);
			}
			else
			{
				JudpLargeMessageBuffer *lmBuffer = new JudpLargeMessageBuffer();
				this->largeMessageMap[wrapper->getSource()] = lmBuffer;
				lmBuffer->addWrapper(wrapper);
			}
			return;
		}
		break;

		case openjaus::transport::INNER_PACKET:
		{
			if(this->largeMessageMap.find(wrapper->getSource()) != this->largeMessageMap.end())
			{
				// Add inner...
				this->largeMessageMap.find(wrapper->getSource())->second->addWrapper(wrapper);
			}
			else
			{
				// Inner Packet for unknown map
				transport::Address source = wrapper->getSource();
				delete wrapper;
				THROW_EXCEPTION("Large Message INNER_PACKET received from " << source << " but no existing lmBuffer for that address! Discarding packet.");
			}
			return;
		}
		break;

		case openjaus::transport::LAST_PACKET:
		{
			if(this->largeMessageMap.find(wrapper->getSource()) != this->largeMessageMap.end())
			{
				// Last Packet... reassemble
				transport::Wrapper *output = this->largeMessageMap.find(wrapper->getSource())->second->assembleWrapper(wrapper);
				if(output != NULL)
				{
//					LOG_DEBUG("Got Reassembled Packet");
//					LOG_DEBUG(dynamic_cast<system::Buffer *>(output->getPayload())->toString());
					recvQueues[JAUS_MESSAGE]->push(output);
					return;
				}
			}
			else
			{
				// Last Packet for unknown map
				transport::Address source = wrapper->getSource();
				delete wrapper;
				THROW_EXCEPTION("Large Message LAST_PACKET received from " << source << " but no existing lmBuffer for that address! Discarding packet.");
			}
			return;
		}
		break;

		case openjaus::transport::SINGLE_PACKET:
		default:
		{
			delete wrapper;
			THROW_EXCEPTION("SINGLE or UNKNOWN packet received in LargeMessageHandler");
			return;
		}
	}
}

void JudpInterface::sendLargeMessage(Wrapper *wrapper)
{
	LOG_DEBUG("Send Large Message: " << wrapper->getPayload()->length());

	system::Buffer *data = dynamic_cast<system::Buffer *>(wrapper->getPayload());
	if(!data)
	{
		THROW_EXCEPTION("Wrapper data buffer is invalid");
	}
	data->reset();
	//LOG_DEBUG(data->toString());

	JausWrapper jWrapper(*wrapper);
	while(data->remainingBytes() > 0)
	{
		if(data->containedBytes() == 0)
		{
			dynamic_cast<system::Buffer *>(jWrapper.getPayload())->setMaxSize(JudpPacket::MAX_PAYLOAD_SIZE);
			data->increment(JudpPacket::MAX_PAYLOAD_SIZE);
			jWrapper.setLargeMessageFlag(FIRST_PACKET);
		}
		else if(data->remainingBytes() > JudpPacket::MAX_PAYLOAD_SIZE)
		{
			dynamic_cast<system::Buffer *>(jWrapper.getPayload())->from(data, JudpPacket::MAX_PAYLOAD_SIZE);
			jWrapper.setLargeMessageFlag(INNER_PACKET);
		}
		else
		{
			dynamic_cast<system::Buffer *>(jWrapper.getPayload())->setMaxSize(data->remainingBytes());
			dynamic_cast<system::Buffer *>(jWrapper.getPayload())->from(data, data->remainingBytes());
			jWrapper.setLargeMessageFlag(LAST_PACKET);
		}

//		LOG_DEBUG("Send Large Message Chunk. SeqNumber: " << jWrapper.getSequenceNumber());
//		LOG_DEBUG("Send Large Message Chunk. Size: " << dynamic_cast<system::Buffer *>(jWrapper.getPayload())->getMaxSize());
//		LOG_DEBUG(dynamic_cast<system::Buffer *>(jWrapper.getPayload())->toString());
		sendMessage(&jWrapper);
	}
}

void JudpInterface::sendMessage(Wrapper* wrapper)
{
	JudpPacket packet;
	try
	{
		if(wrapper->getPayload()->length() > JudpPacket::MAX_PAYLOAD_SIZE)
		{
			LOG_DEBUG("Large Message Detected: " << wrapper->getPayload()->length());
			sendLargeMessage(wrapper);
		}
		else
		{
			packet.reset();
			wrapper->setSequenceNumber(this->sequenceNumber++);
			packet.pushWrapper(wrapper);

			if(wrapper->getType() == transport::CONFIGURATION)
			{
				LOG_DEBUG(datagramSocket.getPort() << " Sending payload: " << ((system::Buffer*)wrapper->getPayload())->toString() )
			}

			// TODO: Compress packet data

			std::list<JudpAddress *> addresses = getDestinationAddresses(wrapper);
			for(std::list<JudpAddress *>::iterator i = addresses.begin(); i != addresses.end(); ++i)
			{
				packet.setAddress((*i)->getIpAddress());
				packet.setPort((*i)->getPort());
				datagramSocket.send(packet);
				delete *i;
			}
		}
	}
	catch(openjaus::system::Exception& exp)
	{
		system::Logger::log(exp);
	}
}
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

