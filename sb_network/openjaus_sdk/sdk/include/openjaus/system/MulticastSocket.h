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
#ifndef SYSTEM_MULTICASTSOCKET_H
#define SYSTEM_MULTICASTSOCKET_H

#include "openjaus/system/InetAddress.h"
#include "openjaus/system/Packet.h"
#include "openjaus/system/Socket.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace system
{
class InetAddress;
class Packet;

/// \class MulticastSocket MulticastSocket.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT MulticastSocket : public Socket
{
public:
	MulticastSocket(); 
	virtual ~MulticastSocket();
	// Start of user code for additional constructors
	MulticastSocket(InetAddress ipAddress, short port);
	// End of user code
	/// Accessor to get the value of multiDescriptor.
	int getMultiDescriptor() const;


	/// Accessor to get the value of maxDescriptor.
	int getMaxDescriptor() const;


	/// Operation open.
	/// \param ipAddress 
	/// \param port 
	 bool open(InetAddress ipAddress, short port);

	/// Operation joinGroup.
	/// \param group 
	 bool joinGroup(InetAddress group);

	/// Operation setTimeToLive.
	/// \param ttl 
	 int setTimeToLive(int ttl);

	/// Operation setLoopback.
	/// \param enabled 
	 int setLoopback(bool enabled);

	/// Operation send.
	/// \param packet 
	 int send(Packet &packet);

	/// Operation receive.
	/// \param packet 
	 int receive(Packet &packet);

	/// Operation reuseAddress.
	/// \param enabled 
	 bool reuseAddress(bool enabled);

	/// Operation enableBroadcast.
	/// \param enabled 
	 bool enableBroadcast(bool enabled);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const MulticastSocket& object);

protected:
	// Member attributes & references
	int multiDescriptor;
	int maxDescriptor;

// Start of user code for additional member data
	fd_set readSet;

// End of user code

}; // class MulticastSocket

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_MULTICASTSOCKET_H

