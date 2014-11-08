/**
\file StreamServer.h

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
#ifndef SYSTEM_STREAMSERVER_H
#define SYSTEM_STREAMSERVER_H

#include "openjaus/system/InetAddress.h"
#include "openjaus/system/Packet.h"
#include "openjaus/system/StreamSocket.h"
#include <map>
#include "openjaus/system/Time.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#ifdef WIN32
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#pragma comment(lib, "Ws2_32.lib")
#elif defined(__linux) || defined(linux) || defined(__linux__)
	#include <sys/epoll.h>
#else
	#error "No Socket implementation defined for this platform."
#endif
// End of user code

namespace openjaus
{
namespace system
{
class InetAddress;
class Packet;
class StreamSocket;
class Time;

/// \class StreamServer StreamServer.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT StreamServer 
{
public:
	StreamServer(); 
	virtual ~StreamServer();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of socketMap.
	const std::map< uint64_t, StreamSocket * >& getSocketMap() const;

	/// Accessor to set value of socketMap.
	/// \param socketMap The value of the new socketMap.
	bool setSocketMap(std::map< uint64_t, StreamSocket * > socketMap);

	/// Accessor to get the value of listener.
	StreamSocket* getListener() const;

	/// Accessor to set value of listener.
	/// \param listener The value of the new listener.
	bool setListener(StreamSocket* listener);

	/// Accessor to get the value of timeout.
	const Time& getTimeout() const;

	/// Accessor to set value of timeout.
	/// \param timeout The value of the new timeout.
	bool setTimeout(const Time& timeout);

	/// Operation open.
	/// \param ipAddress 
	/// \param port 
	 bool open(InetAddress ipAddress, short port);

	/// Operation send.
	/// \param packet 
	 int send(Packet &packet);

	/// Operation receive.
	/// \param packet 
	 int receive(Packet &packet);

	/// Operation reuseAddress.
	/// \param enabled 
	 bool reuseAddress(bool enabled);


	/// \param address 
	/// \param port 
	 StreamSocket* connectNew(InetAddress address, short port);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const StreamServer& object);

protected:
	// Member attributes & references
	std::map< uint64_t, StreamSocket * > socketMap;
	StreamSocket *listener;
	Time timeout;

// Start of user code for additional member data
#ifdef WIN32
	// TODO: This currently limits the number of events to 64.
	// Use overlapped completion ports to support more events
	// with better performance.
	int eventTotal;
	WSAEVENT events[WSA_MAXIMUM_WAIT_EVENTS];
	int socketHash[WSA_MAXIMUM_WAIT_EVENTS];
#else
	int epoll;
	struct epoll_event *events;
#endif
	int pendingEventCount;
	int nextEvent;

// End of user code

}; // class StreamServer

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_STREAMSERVER_H

