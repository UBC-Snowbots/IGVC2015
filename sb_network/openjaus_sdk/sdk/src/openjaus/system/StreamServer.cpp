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

#include "openjaus/system/StreamServer.h"
#include <sstream>
// Start of user code for additional includes
#ifdef WIN32
#elif defined(__linux) || defined(linux) || defined(__linux__)
	#include <sys/epoll.h>
	#include <stdlib.h>
	#include <errno.h>
	#include <fcntl.h>
	#define EPOLL_QUEUE_LEN	128
	#define MAX_EVENT_COUNT	16
#else
	#error "No Socket implementation defined for this platform."
#endif
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
StreamServer::StreamServer() :
		listener(NULL)
{
#ifdef WIN32
	eventTotal = 0;
#else
	// Create system event notifier
	epoll = epoll_create(EPOLL_QUEUE_LEN);
	if(epoll == -1)
	{
		THROW_EXCEPTION("Could Not Create System Event Notifier: " << strerror(errno));
	}

	events = (struct epoll_event *)malloc(sizeof (struct epoll_event) * MAX_EVENT_COUNT);
#endif
	pendingEventCount = 0;
	nextEvent = 0;
}
// End of user code

// Start of user code for default destructor:
StreamServer::~StreamServer()
{
	std::map< uint64_t, StreamSocket * >::const_iterator i;
	for(i = socketMap.begin(); i !=  socketMap.end(); i++)
	{
		delete i->second;
	}

#ifdef WIN32
#else
	close(epoll);
	free(events);

	if(listener)
	{
		delete listener;
		listener = NULL;
	}
#endif

}
// End of user code

const std::map< uint64_t, StreamSocket * >& StreamServer::getSocketMap() const
{
	// Start of user code for accessor getSocketMap:
	
	return socketMap;
	// End of user code
}

bool StreamServer::setSocketMap(std::map< uint64_t, StreamSocket * > socketMap)
{
	// Start of user code for accessor setSocketMap:
	this->socketMap = socketMap;
	return true;
	// End of user code
}


StreamSocket* StreamServer::getListener() const
{
	// Start of user code for accessor getListener:
	
	return listener;
	// End of user code
}

bool StreamServer::setListener(StreamSocket* listener)
{
	// Start of user code for accessor setListener:
	this->listener = listener;
	return true;
	// End of user code
}


const Time& StreamServer::getTimeout() const
{
	// Start of user code for accessor getTimeout:
	
	return timeout;
	// End of user code
}

bool StreamServer::setTimeout(const Time& timeout)
{
	// Start of user code for accessor setTimeout:
	this->timeout = timeout;
	return true;
	// End of user code
}



// Class Methods
bool StreamServer::open(InetAddress ipAddress, short port)
{
	// Start of user code for method open:
	listener = new StreamSocket();
	listener->listenOn(ipAddress, port);

#ifdef WIN32
	events[eventTotal] = WSACreateEvent();
	if(events[eventTotal] == WSA_INVALID_EVENT)
	{
		THROW_EXCEPTION("StreamServer: WSACreateEvent() failed with error: " << lastSocketError());
	}

	if(WSAEventSelect(listener->getDescriptor(), events[eventTotal], FD_ACCEPT|FD_CLOSE) == SOCKET_ERROR)
	{
		THROW_EXCEPTION("StreamServer: WSAEventSelect() failed with error: " << lastSocketError());
	}
	socketHash[eventTotal] = listener->addressHash();
	socketMap[socketHash[eventTotal]] = listener;
	eventTotal++;
#else
	// Set epoll to notify us when there is a connection request on listener
	struct epoll_event ev;
	ev.events = EPOLLIN | EPOLLPRI;
	ev.data.ptr = listener;
	if(epoll_ctl(epoll, EPOLL_CTL_ADD, listener->getDescriptor(), &ev))
	{
		THROW_EXCEPTION("Could Not Add Listener Socket to Event Notifier: " << strerror(errno));
	}
#endif

	return true;
	// End of user code
}


int StreamServer::send(Packet &packet)
{
	// Start of user code for method send:

	// If we have a socket connected to dest address then retrieve it from local map
	StreamSocket *socket = socketMap[packet.addressHash()];
	if(!socket)
	{	// else create and connect a new socket
		socket = connectNew(packet.getAddress(), packet.getPort());
	}

	return socket->send(packet);
	// End of user code
}


int StreamServer::receive(Packet &packet)
{
	// Start of user code for method receive:
	int timeOut_ms = 1000.0 * timeout.inSec();

#ifdef WIN32
	int eventIndex = WSAWaitForMultipleEvents(eventTotal, events, FALSE, timeOut_ms, FALSE);
	if(eventIndex == WSA_WAIT_FAILED)
	{
		THROW_EXCEPTION("StreamServer: WSAWaitForMultipleEvents() failed with error: " << lastSocketError());
	}
	if(eventIndex == WSA_WAIT_TIMEOUT)
	{
		return 0;
	}
	eventIndex -= WSA_WAIT_EVENT_0;

	StreamSocket *socket = socketMap[socketHash[eventIndex]];
	if(!socket)
	{
		THROW_EXCEPTION("StreamServer: Event notification occured for invalid socket");
	}

	WSANETWORKEVENTS networkEvents;
	WSAEnumNetworkEvents(socket->getDescriptor(), events[eventIndex], &networkEvents);

	if(networkEvents.lNetworkEvents & FD_ACCEPT)
	{
		StreamSocket *newSocket = listener->accept();

		events[eventTotal] = WSACreateEvent();
		if(events[eventTotal] == WSA_INVALID_EVENT)
		{
			THROW_EXCEPTION("StreamServer: WSACreateEvent() failed with error: " << lastSocketError());
		}

		if(WSAEventSelect(newSocket->getDescriptor(), events[eventTotal], FD_READ|FD_CLOSE) == SOCKET_ERROR)
		{
			THROW_EXCEPTION("StreamServer: WSAEventSelect() failed with error: " << lastSocketError());
		}
		socketHash[eventTotal] = newSocket->addressHash();
		socketMap[newSocket->addressHash()] = newSocket;
		eventTotal++;

		return 0;
	}

	// Handle FD_CLOSE Hang-up case
	if(networkEvents.lNetworkEvents & FD_CLOSE)
	{
		LOG_DEBUG("StreamServer: Socket Hung Up: " << socket->toString());
		socketMap.erase(socketHash[eventTotal]);
		delete socket;
		return 0;
	}

	if(networkEvents.lNetworkEvents & FD_READ)
	{
		int bytesReceived = socket->receive(packet);
		if(!bytesReceived)
		{
			LOG_DEBUG("StreamServer: Socket Hung Up: " << socket->toString());
			LOG_DEBUG("StreamServer: Deleting Socket: " << socket->addressHash());
			socketMap.erase(socket->addressHash());
			delete socket;
		}
		return bytesReceived;
	}

#else
	if(!pendingEventCount)
	{
		nextEvent = 0;
		pendingEventCount = epoll_wait(epoll, events, MAX_EVENT_COUNT, timeOut_ms);
		if(pendingEventCount < 0)
		{
			if(errno == EINTR)
			{	// Timeout occurred
				pendingEventCount = 0;
				return 0;
			}

			THROW_EXCEPTION("Error Waiting for System Event: " << strerror(errno));
		}
		if(!pendingEventCount)
		{
			return 0;
		}
	}

	if(events[nextEvent].data.ptr == listener)
	{
		nextEvent++;
		pendingEventCount--;

		StreamSocket *newSocket = listener->accept();

		// Add socket to epoll control
		struct epoll_event ev;
		ev.events = EPOLLIN | EPOLLPRI;
		ev.data.ptr = newSocket;
		if(epoll_ctl(epoll, EPOLL_CTL_ADD, newSocket->getDescriptor(), &ev))
		{
			THROW_EXCEPTION("StreamServer: Could Not Add New Socket to Event Notifier: " << epoll << ", " << strerror(errno));
		}

		// Add newSocket to local storage map
		socketMap[newSocket->addressHash()] = newSocket;

		if(!pendingEventCount)
		{
			return 0;
		}
	}

	// TODO: Handle EPOLL error case

	// Handle EPOLL Hang-up case
	if(events[nextEvent].events & EPOLLHUP || events[nextEvent].events & EPOLLERR)
	{
		StreamSocket *socket = static_cast<StreamSocket *>(events[nextEvent].data.ptr);
		LOG_DEBUG("StreamServer: Socket Hung Up: " << socket->toString());

		socketMap.erase(socket->addressHash());
		delete socket;

		nextEvent++;
		pendingEventCount--;
	}

	if(events[nextEvent].events & EPOLLIN)
	{
		StreamSocket *socket = static_cast<StreamSocket *>(events[nextEvent].data.ptr);

		nextEvent++;
		pendingEventCount--;
		int bytesReceived = socket->receive(packet);
		if(!bytesReceived)
		{
			LOG_DEBUG("StreamServer: Socket Hung Up: " << socket->toString());
			LOG_DEBUG("StreamServer: Deleting Socket: " << socket->addressHash());
			socketMap.erase(socket->addressHash());
			delete socket;
		}
		return bytesReceived;
	}
#endif
	return 0;
	// End of user code
}


bool StreamServer::reuseAddress(bool enabled)
{
	// Start of user code for method reuseAddress:
	bool result = 0;

	return result;
	// End of user code
}


StreamSocket* StreamServer::connectNew(InetAddress address, short port)
{
	// Start of user code for method connectNew:
	LOG_DEBUG("StreamServer: Attempting to connect new socket at: " << address << ":" << static_cast<unsigned short>(port));
	StreamSocket *socket = new StreamSocket();
	if(!socket->connectTo(address, port))
	{
		THROW_EXCEPTION("Could Not Connect New Socket");
	}

	LOG_DEBUG("StreamServer: Connected new socket: " << socket->toString());

#ifdef WIN32
	events[eventTotal] = WSACreateEvent();
	if(events[eventTotal] == WSA_INVALID_EVENT)
	{
		THROW_EXCEPTION("StreamServer: WSACreateEvent() failed with error: " << lastSocketError());
	}

	if(WSAEventSelect(socket->getDescriptor(), events[eventTotal], FD_READ|FD_CLOSE) == SOCKET_ERROR)
	{
		THROW_EXCEPTION("StreamServer: WSAEventSelect() failed with error: " << lastSocketError());
	}
	socketHash[eventTotal] = socket->addressHash();
	eventTotal++;
#else
	// Add new socket to epoll control
	struct epoll_event ev;
	ev.events = EPOLLIN | EPOLLPRI;
	ev.data.ptr = socket;
	if(epoll_ctl(epoll, EPOLL_CTL_ADD, socket->getDescriptor(), &ev))
	{
		THROW_EXCEPTION("Could Not Add New Socket to Event Notifier: " << strerror(errno));
	}
#endif

	// Add socket to local map
	socketMap[socket->addressHash()] = socket;

	return socket;
	// End of user code
}




std::string StreamServer::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const StreamServer& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace system
} // namespace openjaus

