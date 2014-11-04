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
#ifndef AS5669_JTCPINTERFACE_H
#define AS5669_JTCPINTERFACE_H

#include <list>
#include "openjaus/transport/AS5669/TCPAddress.h"
#include "openjaus/transport/Wrapper.h"
#include "openjaus/system/StreamServer.h"
#include <map>
#include "openjaus/transport/AS5669/JtcpStream.h"
#include "openjaus/transport/Interface.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{
class Wrapper;
namespace AS5669
{
class TCPAddress;
class JtcpStream;

/// \class JtcpInterface JtcpInterface.h

class OPENJAUS_EXPORT JtcpInterface : public transport::Interface
{
public:
	JtcpInterface(); 
	virtual ~JtcpInterface();
	// Start of user code for additional constructors
	JtcpInterface(system::InetAddress ipAddress, short port);
	// End of user code
	/// Accessor to get the value of streams.
	const std::map< uint64_t, JtcpStream * >& getStreams() const;

	/// Accessor to set value of streams.
	/// \param streams The value of the new streams.
	bool setStreams(std::map< uint64_t, JtcpStream * > streams);

	/// Accessor to get the value of server.
	const system::StreamServer& getServer() const;


	/// Accessor to get the value of tcpAddress.
	TCPAddress* getTcpAddress() const;

	/// Accessor to set value of tcpAddress.
	/// \param tcpAddress The value of the new tcpAddress.
	bool setTcpAddress(TCPAddress* tcpAddress);

	/// Operation getDestinationAddresses.
	/// \param wrapper 
	 std::list< TCPAddress * > getDestinationAddresses(transport::Wrapper *wrapper);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const JtcpInterface& object);

protected:
	// Member attributes & references
	std::map< uint64_t, JtcpStream * > streams;
	system::StreamServer server;
	TCPAddress *tcpAddress;

// Start of user code for additional member data
	virtual void *recvThreadMethod();
	virtual void *sendThreadMethod();

// End of user code

}; // class JtcpInterface

// Start of user code for inline functions
// End of user code



} // namespace AS5669
} // namespace transport
} // namespace openjaus

#endif // AS5669_JTCPINTERFACE_H

