/**
\file JtcpInterface.h

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

#include "openjaus/transport/AS5669/JtcpInterface.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/transport/AddressMap.h"
#include "openjaus/transport/AS5669/JtcpPacket.h"
#include "openjaus/transport/AS5669/JausWrapper.h"
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

// Start of user code for default constructor:
JtcpInterface::JtcpInterface()
{
}
// End of user code

// Start of user code for default destructor:
JtcpInterface::~JtcpInterface()
{
	std::map< uint64_t, JtcpStream * >::const_iterator i;
	for(i = streams.begin(); i !=  streams.end(); i++)
	{
		delete i->second;
	}
}
// End of user code

const std::map< uint64_t, JtcpStream * >& JtcpInterface::getStreams() const
{
	// Start of user code for accessor getStreams:
	
	return streams;
	// End of user code
}

bool JtcpInterface::setStreams(std::map< uint64_t, JtcpStream * > streams)
{
	// Start of user code for accessor setStreams:
	this->streams = streams;
	return true;
	// End of user code
}


const system::StreamServer& JtcpInterface::getServer() const
{
	// Start of user code for accessor getServer:
	
	return server;
	// End of user code
}


TCPAddress* JtcpInterface::getTcpAddress() const
{
	// Start of user code for accessor getTcpAddress:
	
	return tcpAddress;
	// End of user code
}

bool JtcpInterface::setTcpAddress(TCPAddress* tcpAddress)
{
	// Start of user code for accessor setTcpAddress:
	this->tcpAddress = tcpAddress;
	return true;
	// End of user code
}



// Class Methods
std::list< TCPAddress * > JtcpInterface::getDestinationAddresses(transport::Wrapper *wrapper)
{
	// Start of user code for method getDestinationAddresses:
	std::list< TCPAddress * > addresses;

//	if(subsIf && static_cast<unsigned short>(wrapper->getDestination().getSubsystem()) == Address::ANY_SUBSYSTEM)
//	{
//		JudpAddress *address = new JudpAddress();
//		address->setIpAddress(subsIf->getBroadcast());
//		address->setPort(JudpAddress::STANDARD_PORT);
//		addresses.push_back(address);
//	}
//	else if(wrapper->getDestination().getNode() == Address::ANY_NODE)
//	{
//		const std::vector<system::NetworkInterface*>& interfaces = datagramSocket.getInterfaces();
//		// TODO: Implement in socket class getInternodeBroadcasts (loopback name needs OS abstraction)
//		for(unsigned int i = 0; i < interfaces.size(); ++i)
//		{
//			// Exclude the subsystem interface (if one exists) and loopback (which is always cmpt interface)
//			if(	(subsIf && subsIf->getName().compare(interfaces[i]->getName()) == 0) ||
//				interfaces[i]->getName().compare("lo")	== 0)
//			{
//				continue;
//			}
//
//			JudpAddress *address = new JudpAddress();
//			address->setIpAddress(interfaces[i]->getBroadcast());
//			address->setPort(JudpAddress::STANDARD_PORT);
//			addresses.push_back(address);
//		}
//	}
//	else if(wrapper->getDestination().getComponent() == Address::ANY_COMPONENT)
//	{	// Send to standard port on loopback interface
//		JudpAddress *address = new JudpAddress();
//		address->setIpAddress(system::InetAddress::getByName("127.255.255.255"));
//		address->setPort(JudpAddress::STANDARD_PORT);
//		addresses.push_back(address);
//	}
//	else
//	{
//		switch(wrapper->getType())
//		{
//			case JAUS_MESSAGE:
//			{
//				// Lookup UDP address from JAUS address
//				JudpAddress *address = new JudpAddress();
//				if(AddressMap::getTransportData(wrapper->getDestination(), *address))
//				{
//					addresses.push_back(address);
//				}
//				else
//				{
//					delete address;
//					THROW_EXCEPTION("JUDP Interface attempted to send message to unknown address: " << wrapper->getDestination().toString());
//				}
//				break;
//			}
//
//			case CONFIGURATION:
//			{
//				if(!wrapper->getTransportData())
//				{
//					THROW_EXCEPTION("JUDP Interface: Attempted to send configuration message without transport data");
//				}
//
//				JudpAddress *address = dynamic_cast<JudpAddress *>(wrapper->getTransportData());
//				if(!address)
//				{
//					THROW_EXCEPTION("JUDP Interface: Attempted to send configuration message without transport data");
//				}
//				addresses.push_back(address);
//				break;
//			}
//
//			case OPENJAUS_MESSAGE:
//			{
//				// TODO: Populate this
//				break;
//			}
//		}
//
//	}

	return addresses;
	// End of user code
}




std::string JtcpInterface::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const JtcpInterface& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
JtcpInterface::JtcpInterface(system::InetAddress ipAddress, short port) :
		transport::Interface(),
		server(),
		tcpAddress(NULL)
{
	// Open Server on desired port
	server.open(ipAddress, port);
	LOG("Opened JTCP Interface: " << server.getListener()->toString());
}

void * JtcpInterface::recvThreadMethod()
{
	system::Time timeout;
	timeout.setMicroseconds(250000);
	server.setTimeout(timeout);

	while(recvThread.isRunning())
	{
		// 	Receive a packet from the server
		JtcpPacket packet;
		int bytesRecv = server.receive(packet);
		if(!bytesRecv)
		{	// Did not receive any packet data so try again
			continue;
		}

		LOG_DEBUG("JtcpInterface: Recv'd " << bytesRecv << " bytes");

		JtcpStream *stream;

		LOG_DEBUG("JtcpInterface: Finding stream for: " << packet.addressHash());

		std::map< uint64_t, JtcpStream * >::iterator i = streams.find(packet.addressHash());
		//  If packet source TCP addr is new then create a new buffer
		if(i == streams.end())
		{
			LOG_DEBUG("JtcpInterface: Creating new steam for port: " << packet.getPort());

			stream = new JtcpStream();
			TCPAddress newAddress;
			newAddress.setIpAddress(packet.getAddress());
			newAddress.setPort(packet.getPort());
			stream->setAddress(newAddress);
			streams[packet.addressHash()] = stream;
		}
		else
		{
			stream = i->second;
		}

		//  Add packet data to source's buffer
		packet.increment(bytesRecv);
		stream->append(packet);
		LOG_DEBUG("JtcpInterface: Stream now contains: " << stream->containedBytes() << " bytes");

		// Attempt to extract JTCP message from buffer
		Wrapper *wrapper = stream->popWrapper();
		if(!wrapper)
		{
			continue;
		}

		// Store transport address information
		TCPAddress *tcpAddress = new TCPAddress();
		tcpAddress->setIpAddress(packet.getAddress());
		tcpAddress->setPort(packet.getPort());
		wrapper->setTransportData(tcpAddress);
		AddressMap::setTransportData( wrapper->getSource(), *tcpAddress);
		LOG_DEBUG("JtcpInterface: Received JAUS Wrapper from: " << tcpAddress->toString());

		recvQueue->push(wrapper);
	}
	return NULL;
}

void * JtcpInterface::sendThreadMethod()
{
	LOG_DEBUG("JtcpInterface: Starting Send Thread");

	while(sendThread.isRunning())
	{
		Wrapper *wrapper = dynamic_cast<Wrapper*>(sendQueue.pop());
		if(wrapper == NULL)
		{
			sendQueue.timedWait(system::Condition::DEFAULT_WAIT_MSEC);
			continue;
		}

		try
		{
			TCPAddress address;
			if(!AddressMap::getTransportData(wrapper->getDestination(), address))
			{
				LOG_EXCEPTION("JtcpInterface: Attempted to send message to unknown address: " << wrapper->getDestination());
			}

			JtcpStream *stream;
			LOG_DEBUG("JtcpInterface: Finding stream for: " << address.hash());
			std::map< uint64_t, JtcpStream * >::iterator i = streams.find(address.hash());
			//  If packet source TCP addr is new then create a new buffer
			if(i == streams.end())
			{
				// Create a new socket connection to determine new port
				system::StreamSocket *socket = server.connectNew(address.getIpAddress(), address.getPort());

				stream = new JtcpStream();
				address.setIpAddress(socket->getIpAddress());
				address.setPort(socket->getPort());
				stream->setAddress(address);
				streams[socket->addressHash()] = stream;

				AddressMap::setTransportData( wrapper->getDestination(), address);
				LOG_DEBUG("JtcpInterface: Created new stream for port: " << address.getPort());
			}
			else
			{
				stream = i->second;
			}

			wrapper->setSequenceNumber(this->sequenceNumber++);
			system::Packet& packet = stream->packetize(*wrapper);

			LOG_DEBUG("JtcpInterface: Sending wrapper: " << wrapper << " to: " << address );
			server.send(packet);

			delete wrapper;
		}
		catch(openjaus::system::Exception exp)
		{
			system::Logger::log(exp);
		}
	}
	return NULL;
}
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

