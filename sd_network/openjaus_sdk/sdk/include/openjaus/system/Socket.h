/**
\file Socket.h

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
#ifndef SYSTEM_SOCKET_H
#define SYSTEM_SOCKET_H

#include "openjaus/system/InetAddress.h"
#include "openjaus/system/Time.h"
#include "openjaus/system/NetworkInterface.h"
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes

#include "openjaus/system/Packet.h"

#ifdef WIN32
	#include <winsock2.h>
	#include <ws2tcpip.h>
    #ifndef socklen_t
        typedef int socklen_t;
    #endif
	#define CLOSE_SOCKET closesocket
	#include <Iphlpapi.h>
	typedef unsigned int size_t;
	#pragma comment(lib, "Ws2_32.lib")
	#pragma comment(lib,"Iphlpapi.lib")

	inline std::string errorString(int errorNum)
	{
		// Retrieve the system error message for the last-error code
		LPTSTR lpMsgBuf;
		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			errorNum,
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR) &lpMsgBuf,
			0, NULL );
		std::string resultString(lpMsgBuf);
		LocalFree(lpMsgBuf);
		return resultString;
	}

	inline std::string lastSocketError()
	{
		return errorString(WSAGetLastError());
	}
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <unistd.h>
	#include <sys/socket.h>
	#include <netdb.h>
	#include <arpa/inet.h>
	#include <sys/time.h>
	#include <netinet/in.h>
	#include <sys/types.h>
	#include <sys/ioctl.h>
	#include <linux/if.h>
	#include <errno.h>
	#define CLOSE_SOCKET close

	inline std::string lastSocketError()
	{
		return strerror(errno);
	}

#else
	#error "No Socket implementation defined for this platform."
#endif
// End of user code

namespace openjaus
{
namespace system
{
class InetAddress;
class Time;
class NetworkInterface;

/// \class Socket Socket.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Socket 
{
public:
	Socket(); 
	virtual ~Socket();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of blocking.
	bool isBlocking() const;


	/// Accessor to get the value of port.
	uint16_t getPort() const;


	/// Accessor to get the value of descriptor.
	int getDescriptor() const;


	/// Accessor to get the value of ipAddress.
	const InetAddress& getIpAddress() const;


	/// Accessor to get the value of timeout.
	const Time& getTimeout() const;

	/// Accessor to set value of timeout.
	/// \param timeout The value of the new timeout.
	bool setTimeout(const Time& timeout);

	/// Accessor to get the value of interfaces.
	const std::vector< NetworkInterface* >& getInterfaces() const;


	/// Operation readInterfaces.
	 void readInterfaces();

	/// Operation isLocalAddress.
	/// \param ipAddress 
	 bool isLocalAddress(const InetAddress &ipAddress);

	/// Operation addressHash.
	 uint64_t addressHash();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Socket& object);

protected:
	// Member attributes & references
	bool blocking;
	uint16_t port;
	int descriptor;
	InetAddress ipAddress;
	Time timeout;
	std::vector< NetworkInterface* > interfaces;

// Start of user code for additional member data
	static int descriptorCount;

	static void initialize();
	static void deinitialize();

	virtual int send(Packet &) = 0;
	virtual int receive(Packet &) = 0;
// End of user code

}; // class Socket

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_SOCKET_H

