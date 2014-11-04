/**
\file MulticastSocket.h

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

#include "openjaus/system/MulticastSocket.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
#include <string.h>
#include <errno.h>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
MulticastSocket::MulticastSocket() : Socket(),
	multiDescriptor(-1),
	maxDescriptor(0)
{
	// Open a socket with: Protocol Family (PF) IPv4, of Datagram Socket Type, and using UDP IP protocol explicitly
	descriptor = (int) socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(descriptor  == -1)
	{
		THROW_EXCEPTION("Could Not Open Socket: " << lastSocketError());
	}

	readInterfaces();

	// Initialize parameters needed to select on socket to read in data
	maxDescriptor = descriptor;
	FD_ZERO(&readSet);

}
// End of user code

// Start of user code for default destructor:
MulticastSocket::~MulticastSocket()
{
	if(multiDescriptor > 0)
	{
		CLOSE_SOCKET(multiDescriptor);
	}
}
// End of user code

int MulticastSocket::getMultiDescriptor() const
{
	// Start of user code for accessor getMultiDescriptor:
	
	return multiDescriptor;
	// End of user code
}


int MulticastSocket::getMaxDescriptor() const
{
	// Start of user code for accessor getMaxDescriptor:
	
	return maxDescriptor;
	// End of user code
}



// Class Methods
bool MulticastSocket::open(InetAddress ipAddress, short port)
{
	// Start of user code for method open:
	struct sockaddr_in address;
	memset(&address, 0, sizeof(address));			// Clear the data structure to zero
	address.sin_family = AF_INET;					// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	address.sin_addr.s_addr = ipAddress.getValue();	// Set Internet Socket (sin), Address to: The ipAddressString argument
	address.sin_port = htons(port);					// Set Internet Socket (sin), Port to: the port argument

	// Bind our open socket to a free port on the given interface, with our defined ipAddress
	if(bind(descriptor, (struct sockaddr *)&address, sizeof(address)))
	{
		THROW_EXCEPTION("Could Not Bind to Socket: " << lastSocketError());
	}

	socklen_t addressLength = sizeof(address);
	if(getsockname(descriptor, (struct sockaddr *)&address, &addressLength))
	{
		THROW_EXCEPTION("Could Get Socket Name: " << lastSocketError());
	}

	// Set the new port value if it is different from the one requested
	this->ipAddress.setValue(address.sin_addr.s_addr);
	this->port = ntohs(address.sin_port);

	// Tell the kernel to send multicast packets from this interface
	int ipValue = ipAddress.getValue();
	if(setsockopt(descriptor, IPPROTO_IP, IP_MULTICAST_IF, (char *)&ipValue, sizeof(ipValue)))
	{
		THROW_EXCEPTION("Could Not Set Socket to Multicast: " << lastSocketError());
	}

	return true;
	// End of user code
}


bool MulticastSocket::joinGroup(InetAddress group)
{
	// Start of user code for method joinGroup:
	struct ip_mreq multicastRequest;
	multicastRequest.imr_multiaddr.s_addr = group.getValue();
	multicastRequest.imr_interface.s_addr = ipAddress.getValue();

	int returnVal = 0;

#if defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__)
	// Open a socket with: Protocol Family (PF) IPv4, of Datagram Socket Type, and using UDP IP protocol explicitly
	multiDescriptor = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(multiDescriptor  == -1)
	{
		THROW_EXCEPTION("Could Not Open Multicast Socket: " << lastSocketError());
	}

	// Allow multiple sockets to use the same PORT number
	int on = static_cast<int>(true);
	if(setsockopt(multiDescriptor, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on)))
	{
		THROW_EXCEPTION("Could Not Reuse Socket Address: " << lastSocketError());
	}

	struct sockaddr_in address;
	memset(&address, 0, sizeof(struct sockaddr_in));		// Clear the data structure to zero
	address.sin_family = AF_INET;				// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	address.sin_addr.s_addr = system::InetAddress::anyAddress().getValue();	// Set Internet Socket (sin), Address to: The ipAddressString argument
	address.sin_port = htons(port);				// Set Internet Socket (sin), Port to: the port argument

	// Bind our open socket to a free port on the given interface, with our defined ipAddress
	if(bind(multiDescriptor, (struct sockaddr *)&address, sizeof(struct sockaddr_in)))
	{
		CLOSE_SOCKET(multiDescriptor);
		THROW_EXCEPTION("Could Not Bind to Multicast Socket: " << lastSocketError());
	}

	// Set max descriptor parameter needed to select on socket during read operation
	maxDescriptor = multiDescriptor > maxDescriptor? multiDescriptor : maxDescriptor;

	returnVal = setsockopt(multiDescriptor, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&multicastRequest, sizeof(multicastRequest));
#else
	returnVal = setsockopt(descriptor, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&multicastRequest, sizeof(multicastRequest));
#endif
	if(returnVal == -1)
	{
		THROW_EXCEPTION("Could Not Join Multicast Group: " << lastSocketError());
	}

	return true;
	// End of user code
}


int MulticastSocket::setTimeToLive(int ttl)
{
	// Start of user code for method setTimeToLive:

	// Saturation limits for time to live
	ttl = ttl > 255? 255 : ttl;
	ttl = ttl < 1? 1 : ttl;

	int returnVal = 0;
	if(multiDescriptor != -1)
	{
		returnVal = setsockopt(multiDescriptor, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl, sizeof(unsigned int));
		if(returnVal == -1)
		{
			THROW_EXCEPTION("Could Not Set Multicast Time to live: " << lastSocketError());
		}

		returnVal = setsockopt(descriptor, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl, sizeof(unsigned int));
	}
	else
	{
		returnVal = setsockopt(descriptor, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl, sizeof(unsigned int));
	}

	if(returnVal == -1)
	{
		THROW_EXCEPTION("Could Not Set Multicast Time to live: " << lastSocketError());
	}
	return returnVal;
	// End of user code
}


int MulticastSocket::setLoopback(bool enabled)
{
	// Start of user code for method setLoopback:
	unsigned int value = enabled;
	int returnVal = setsockopt(descriptor, IPPROTO_IP, IP_MULTICAST_LOOP, (char *)&value, sizeof(unsigned int));
	if(returnVal == -1)
	{
		THROW_EXCEPTION("Could Not Set Multicast Loopback: " << lastSocketError());
	}

	return returnVal;
	// End of user code
}


int MulticastSocket::send(Packet &packet)
{
	// Start of user code for method send:
	struct sockaddr_in toAddress;
	memset(&toAddress, 0, sizeof(toAddress));
	toAddress.sin_family = AF_INET;								// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	toAddress.sin_addr.s_addr = packet.getAddress().getValue();	// Set Internet Socket (sin), Address to: The packet ipAddressString
	toAddress.sin_port = htons(packet.getPort());				// Set Internet Socket (sin), Port to: the packet port

	// Returns number of bytes sent
	int bytesSent = sendto(descriptor, (char *)packet.getBuffer(), packet.containedBytes(), 0, (struct sockaddr *)&toAddress, sizeof(toAddress));
	if(bytesSent == -1)
	{
		THROW_EXCEPTION("Could Not Send Data: " << lastSocketError());
	}

	return bytesSent;
	// End of user code
}


int MulticastSocket::receive(Packet &packet)
{
	// Start of user code for method receive:

	// If no file descriptors are ready for reading
	if(	!FD_ISSET(descriptor, &readSet) && (multiDescriptor == -1 || !FD_ISSET(multiDescriptor, &readSet)))
	{
		// Reset readSet just in case any other bits are set
		FD_ZERO(&readSet);

		// Setup readSet to instruct select to listen to the unicast descriptor
		FD_SET((unsigned int)descriptor, &readSet);

		if(multiDescriptor != -1)
		{
			// Setup readSet to instruct select to listen to the multicast descriptor
			FD_SET((unsigned int)multiDescriptor, &readSet);
		}

		// Setup timeout for select
		struct timeval *timeoutPtr = NULL;
		if(!blocking)
		{
			struct timeval timeoutVal;
			timeoutVal.tv_sec = timeout.getSeconds();
			timeoutVal.tv_usec = timeout.getMicroseconds();
			timeoutPtr = &timeoutVal;
		}

		int count = select(maxDescriptor + 1, &readSet, NULL, NULL, timeoutPtr);
		if(count < 0)
		{
			THROW_EXCEPTION("Could Not Wait For Data: " << lastSocketError());
		}

		if(count == 0) // Timeout expired
		{
			return 0;
		}
	}

	// Zero from address
	struct sockaddr_in fromAddress;
	socklen_t fromAddressLength = sizeof(fromAddress);
	memset(&fromAddress, 0, fromAddressLength);

	int bytesReceived = 0;

	// Check to see which socket is ready for reading and read that socket one at a time
	if(FD_ISSET(descriptor, &readSet))
	{
		FD_CLR((unsigned int)descriptor, &readSet);
		bytesReceived = recvfrom(descriptor, (char *)packet.getBuffer(), packet.getMaxSize(), 0, (struct sockaddr*)&fromAddress, &fromAddressLength);
	}
	else if(FD_ISSET(multiDescriptor, &readSet))
	{
		FD_CLR((unsigned int)multiDescriptor, &readSet);
		bytesReceived = recvfrom(multiDescriptor, (char *)packet.getBuffer(), packet.getMaxSize(), 0, (struct sockaddr*)&fromAddress, &fromAddressLength);
	}

	if(bytesReceived < 0)
	{
		THROW_EXCEPTION("Could Not Receive Data: " << lastSocketError());
	}

	packet.setPort(ntohs(fromAddress.sin_port));
	InetAddress fromIpAddress(fromAddress.sin_addr.s_addr);
	packet.setAddress(fromIpAddress);
	return bytesReceived;
	// End of user code
}


bool MulticastSocket::reuseAddress(bool enabled)
{
	// Start of user code for method reuseAddress:
	int on = static_cast<int>(enabled);

	// Allow multiple sockets to use the same PORT number
	if(setsockopt(descriptor, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on)))
	{
		THROW_EXCEPTION("Could Not Reuse Socket Address: " << lastSocketError());
	}

	return true;
	// End of user code
}


bool MulticastSocket::enableBroadcast(bool enabled)
{
	// Start of user code for method enableBroadcast:
	int on = static_cast<int>(enabled);

	// Allow multiple sockets to use the same PORT number
	if(setsockopt(descriptor, SOL_SOCKET, SO_BROADCAST, (char *)&on, sizeof(on)))
	{
		THROW_EXCEPTION("Could Not Set Broadcast: " << lastSocketError());
	}

	return true;
	// End of user code
}




std::string MulticastSocket::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "udp://" << ipAddress << ":" << static_cast<unsigned short>(port);
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const MulticastSocket& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
MulticastSocket::MulticastSocket(InetAddress ipAddress, short port) :
		Socket()
{
	// Open a socket with: Protocol Family (PF) IPv4, of Datagram Socket Type, and using UDP IP protocol explicitly
	descriptor = (int) socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(descriptor  == -1)
	{
		THROW_EXCEPTION("Could Not Open Socket: " << lastSocketError());
	}

	readInterfaces();

	// Initialize parameters needed to select on socket to read in data
	maxDescriptor = descriptor;
	FD_ZERO(&readSet);

	open(ipAddress, port);
}

// End of user code

} // namespace system
} // namespace openjaus

