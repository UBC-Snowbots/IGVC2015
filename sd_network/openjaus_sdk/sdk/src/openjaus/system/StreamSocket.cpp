/**
\file StreamSocket.h

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

#include "openjaus/system/StreamSocket.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
StreamSocket::StreamSocket() : Socket()
{
}
// End of user code

// Start of user code for default destructor:
StreamSocket::~StreamSocket()
{
}
// End of user code


// Class Methods
StreamSocket* StreamSocket::accept()
{
	// Start of user code for method accept:
	StreamSocket *newSocket = new StreamSocket();
	struct sockaddr_in fromAddress;
	socklen_t fromAddressLength = sizeof(fromAddress);

	newSocket->descriptor = ::accept(descriptor, (struct sockaddr*)&fromAddress, &fromAddressLength);
	if(newSocket->descriptor == -1)
	{
		THROW_EXCEPTION("Could Not Accept Connection: " << lastSocketError());
	}

	newSocket->port = ntohs(fromAddress.sin_port);
	newSocket->ipAddress.setValue(fromAddress.sin_addr.s_addr);

	LOG_DEBUG("StreamSocket: Accepted new socket: " << newSocket->descriptor << ", from: " << newSocket->ipAddress << ":" << newSocket->port);

	return newSocket;
	// End of user code
}


bool StreamSocket::listenOn(InetAddress ipAddress, short port)
{
	// Start of user code for method listenOn:
	// Open a socket with: Protocol Family (PF) IPv4, of Stream Socket Type, and an implicit protocol
	descriptor = (int) socket(PF_INET, SOCK_STREAM, 0);
	if(descriptor == -1)
	{
		THROW_EXCEPTION("Could Not Open Socket: " << lastSocketError());
	}

	struct sockaddr_in address;
	memset(&address, 0, sizeof(address));			// Clear the data structure to zero
	address.sin_family = AF_INET;					// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	address.sin_addr.s_addr = ipAddress.getValue();	// Set Internet Socket (sin), Address to: The ipAddressString argument
	address.sin_port = htons(port);					// Set Internet Socket (sin), Port to: the port argument

	// Bind our open socket to a free port on the localhost, with our defined ipAddress
	if(bind(descriptor, (struct sockaddr *)&address, sizeof(address)))
	{
		THROW_EXCEPTION("Could Not Bind To Socket: " << lastSocketError());
	}

	socklen_t addressLength = sizeof(address);
	if(getsockname(descriptor, (struct sockaddr *)&address, &addressLength))
	{
		THROW_EXCEPTION("Could Not Get Socket Name: " << lastSocketError());
	}

	this->ipAddress = ipAddress;
	this->port = ntohs(address.sin_port);

	if(::listen(descriptor, SOMAXCONN))
	{
		THROW_EXCEPTION("Could Not Listen On Socket: " << lastSocketError());
	}

	return true;
	// End of user code
}


bool StreamSocket::connectTo(InetAddress address, short port)
{
	// Start of user code for method connectTo:
//	if(!blocking)
//	{
//		// Set socket to non-blocking
//		int fileStatusFlags = fcntl(descriptor, F_GETFL, NULL);
//		if( fileStatusFlags < 0 )
//		{
//			THROW_EXCEPTION("Could Not Get Socket Flags: " << lastSocketError());
//		}
//
//		fileStatusFlags |= O_NONBLOCK;
//
//		if( fcntl(descriptor, F_SETFL, fileStatusFlags) < 0)
//		{
//			THROW_EXCEPTION("Could Not Set Socket Flags: " << lastSocketError());
//		}
//	}

	// Open a socket with: Protocol Family (PF) IPv4, of Stream Socket Type, and an implicit protocol
	descriptor = (int) socket(PF_INET, SOCK_STREAM, 0);
	if(descriptor == -1)
	{
		THROW_EXCEPTION("Could Not Open Socket: " << lastSocketError());
	}

	struct sockaddr_in sockAddress;
	memset(&sockAddress, 0, sizeof(sockAddress));
	sockAddress.sin_family = AF_INET;					// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	sockAddress.sin_addr.s_addr = address.getValue();	// Set Internet Socket (sin), Address to: Dest ipAddressString
	sockAddress.sin_port = htons(port);				// Set Internet Socket (sin), Port to: Dest port

	if(connect(descriptor, (struct sockaddr *)&sockAddress, sizeof(sockAddress)) == 0)
	{
		socklen_t addressLength = sizeof(sockAddress);
		if(getsockname(descriptor, (struct sockaddr *)&sockAddress, &addressLength))
		{
			THROW_EXCEPTION("Could Not Get Socket Name: " << lastSocketError());
		}

		this->ipAddress = InetAddress(sockAddress.sin_addr.s_addr);
		this->port = ntohs(sockAddress.sin_port);
		return true;
	}

#ifdef WIN32
	if(errno != WSAEINPROGRESS)
	{
		THROW_EXCEPTION("Connect to socket: A blocking Windows Sockets call is in progress");
	}
#else
	if(errno != EINPROGRESS)
	{
		THROW_EXCEPTION("Connect to socket: " << lastSocketError());
	}
#endif

	struct timeval timeoutVal;
	timeoutVal.tv_sec = timeout.getSeconds();
	timeoutVal.tv_usec = timeout.getMicroseconds();

	fd_set writeSet;
	FD_ZERO(&writeSet);
	FD_SET((unsigned int)descriptor, &writeSet);

	// Wait until timeout while connecting is in progress
	int socketCount = select(descriptor+1, NULL, &writeSet, NULL, &timeoutVal);
	if(socketCount < 0 && errno != EINTR)
	{
		THROW_EXCEPTION("Could Not Wait For Connect: " << lastSocketError());
	}

	if(socketCount < 1)
	{
		// Connect attempt timed out, just exit gracefully
		return false;
	}

	// Socket selected for action or error has occurred, check for error before returning success
	int socketError = 0;
	socklen_t optionLength = sizeof(socketError);
	if(getsockopt(descriptor, SOL_SOCKET, SO_ERROR, (char*)&socketError, &optionLength) < 0 || socketError)
	{
		THROW_EXCEPTION("Could Not Get Socket Error: " << lastSocketError());
	}

	this->ipAddress = address;
	this->port = port;

	return true;
	// End of user code
}


int StreamSocket::send(Packet &packet)
{
	// Start of user code for method send:
	int bytesSent = ::send(descriptor, (char *)packet.getBuffer(), packet.containedBytes(), 0);
	if(bytesSent == -1)
	{
		THROW_EXCEPTION("Could Not Send Data: " << lastSocketError());
	}
	return bytesSent;
	// End of user code
}


int StreamSocket::receive(Packet &packet)
{
	// Start of user code for method receive:
	int bytesRecv = recv(descriptor, (char *)packet.getBuffer(), packet.getMaxSize(), 0);
	if(bytesRecv == -1)
	{
		THROW_EXCEPTION("Could Not Receive Data: " << lastSocketError());
	}

	packet.setAddress(ipAddress);
	packet.setPort(port);

	return bytesRecv;
	// End of user code
}


bool StreamSocket::reuseAddress(bool enabled)
{
	// Start of user code for method reuseAddress:
	bool result = 0;

	return result;
	// End of user code
}




std::string StreamSocket::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "tcp://" << ipAddress << ":" << static_cast<unsigned short>(port);
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const StreamSocket& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
StreamSocket::StreamSocket(InetAddress ipAddress, short port) :
		Socket()
{

}

// End of user code

} // namespace system
} // namespace openjaus

