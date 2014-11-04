/**
\file DatagramSocket.h

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

#include "openjaus/system/DatagramSocket.h"
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
DatagramSocket::DatagramSocket()
{
	// Open a socket with: Protocol Family (PF) IPv4, of Datagram Socket Type, and using UDP IP protocol explicitly
	descriptor = (int) socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(descriptor == -1)
	{
		THROW_EXCEPTION("Could Not Create Socket: " << lastSocketError());
	}
	readInterfaces();
}
// End of user code

// Start of user code for default destructor:
DatagramSocket::~DatagramSocket()
{
}
// End of user code


// Class Methods
bool DatagramSocket::open(InetAddress ipAddress, short port)
{
	// Start of user code for method open:
	struct sockaddr_in address;
	memset(&address, 0, sizeof(address));					// Clear the data structure to zero
	address.sin_family = AF_INET;							// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	address.sin_addr.s_addr = ipAddress.getValue();			// Set Internet Socket (sin), Address to: The ipAddressString argument
	address.sin_port = htons(port);							// Set Internet Socket (sin), Port to: the port argument

	// Bind our open socket to a free port on the localhost, with our defined ipAddress
	if(bind(descriptor, (struct sockaddr *)&address, sizeof(address)))
	{
		THROW_EXCEPTION("Could Not Bind to Socket: " << lastSocketError());
	}

	socklen_t addressLength = sizeof(address);
	if(getsockname(descriptor, (struct sockaddr *)&address, &addressLength))
	{
		THROW_EXCEPTION("Could Not Get Socket Name: " << lastSocketError());
	}

	this->ipAddress.setValue(address.sin_addr.s_addr);
	this->port = ntohs(address.sin_port);

	return true;
	// End of user code
}


int DatagramSocket::send(Packet &packet)
{
	// Start of user code for method send:
	struct sockaddr_in toAddress;
	memset(&toAddress, 0, sizeof(toAddress));
	toAddress.sin_family = AF_INET;									// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	toAddress.sin_addr.s_addr = packet.getAddress().getValue();	// Set Internet Socket (sin), Address to: The packet ipAddressString
	toAddress.sin_port = htons(packet.getPort());						// Set Internet Socket (sin), Port to: the packet port

	// Returns number of bytes sent
	int bytesSent = sendto(descriptor, (char *)packet.getBuffer(), packet.containedBytes(), 0, (struct sockaddr *)&toAddress, sizeof(toAddress));
	if(bytesSent == -1)
	{
		THROW_EXCEPTION("Could Not Send Datagram: " << lastSocketError());
	}

	return bytesSent;
	// End of user code
}


int DatagramSocket::receive(Packet &packet)
{
	// Start of user code for method receive:
	struct timeval *timeoutPtr = NULL;
	if(!blocking)
	{
		struct timeval timeoutVal;
		timeoutVal.tv_sec = timeout.getSeconds();
		timeoutVal.tv_usec = timeout.getMicroseconds();
		timeoutPtr = &timeoutVal;
	}

	fd_set readSet;
	FD_ZERO(&readSet);
	FD_SET((unsigned int)descriptor, &readSet);

	int selectReturnVal = select(descriptor + 1, &readSet, NULL, NULL, timeoutPtr);
	if(selectReturnVal < 0)
	{
		THROW_EXCEPTION("Could Not Receive Datagram: " << lastSocketError());
	}

	if(!FD_ISSET(descriptor, &readSet))
	{
		// Assume timed out
		return 0;
	}

	struct sockaddr_in fromAddress;
	socklen_t fromAddressLength = sizeof(fromAddress);
	int bytesReceived = recvfrom(descriptor, (char *)packet.getBuffer(), packet.getMaxSize(), 0, (struct sockaddr*)&fromAddress, &fromAddressLength);
	if(bytesReceived == -1)
	{
		THROW_EXCEPTION("Could Not Receive Datagram: " << lastSocketError());
	}

	packet.setPort(ntohs(fromAddress.sin_port));
	InetAddress fromIpAddress(fromAddress.sin_addr.s_addr);
	packet.setAddress(fromIpAddress);

	return bytesReceived;
	// End of user code
}


bool DatagramSocket::reuseAddress(bool enabled)
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


bool DatagramSocket::enableBroadcast(bool enabled)
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


bool DatagramSocket::setBufferSize(uint32_t size)
{
	// Start of user code for method setBufferSize:
	if(setsockopt(descriptor, SOL_SOCKET, SO_SNDBUF, (char*)&size, sizeof(size)))
	{
		THROW_EXCEPTION("Could Not Set Send Buffer: " << lastSocketError());
	}

	if(setsockopt(descriptor, SOL_SOCKET, SO_RCVBUF, (char*)&size, sizeof(size)))
	{
		THROW_EXCEPTION("Could Not Set Recv Buffer: " << lastSocketError());
	}

	return true;
	// End of user code
}




std::string DatagramSocket::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "udp://" << ipAddress << ":" << static_cast<unsigned short>(port);
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const DatagramSocket& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
DatagramSocket::DatagramSocket(InetAddress ipAddress, short port) :
		Socket()
{
	// Open a socket with: Protocol Family (PF) IPv4, of Datagram Socket Type, and using UDP IP protocol explicitly
	descriptor = (int) socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(descriptor == -1)
	{
		THROW_EXCEPTION("Could Not Create Socket: " << lastSocketError());
	}

	readInterfaces();

	open(ipAddress, port);
}

// End of user code

} // namespace system
} // namespace openjaus

