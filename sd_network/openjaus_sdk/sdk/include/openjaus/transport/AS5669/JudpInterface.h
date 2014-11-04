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
#ifndef AS5669_JUDPINTERFACE_H
#define AS5669_JUDPINTERFACE_H

#include <list>
#include "openjaus/transport/AS5669/JudpAddress.h"
#include "openjaus/transport/Wrapper.h"
#include "openjaus/system/DatagramSocket.h"
#include "openjaus/system/Thread.h"
#include "openjaus/system/MulticastSocket.h"
#include <map>
#include "openjaus/transport/CompressedHeader.h"
#include "openjaus/transport/Interface.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "openjaus/transport/AS5669/JudpPacket.h"
#include "openjaus/transport/AS5669/JudpLargeMessageBuffer.h"
// End of user code

namespace openjaus
{
namespace transport
{
class Wrapper;
class CompressedHeader;
namespace AS5669
{
class JudpAddress;

/// \class JudpInterface JudpInterface.h

class OPENJAUS_EXPORT JudpInterface : public transport::Interface
{
public:
	JudpInterface(); 
	virtual ~JudpInterface();
	// Start of user code for additional constructors
	JudpInterface(system::InetAddress ipAddress, short port);
	// End of user code
	/// Accessor to get the value of headerMap.
	const std::map< uint32_t, std::list< transport::CompressedHeader * > >& getHeaderMap() const;

	/// Accessor to set value of headerMap.
	/// \param headerMap The value of the new headerMap.
	bool setHeaderMap(std::map< uint32_t, std::list< transport::CompressedHeader * > > headerMap);

	/// Accessor to get the value of datagramSocket.
	const system::DatagramSocket& getDatagramSocket() const;


	/// Accessor to get the value of udpAddress.
	JudpAddress* getUdpAddress() const;

	/// Accessor to set value of udpAddress.
	/// \param udpAddress The value of the new udpAddress.
	bool setUdpAddress(JudpAddress* udpAddress);

	/// Accessor to get the value of discoveryThread.
	const system::Thread& getDiscoveryThread() const;


	/// Accessor to get the value of discoverySocket.
	const system::MulticastSocket& getDiscoverySocket() const;



	virtual void run();


	virtual void stop();

	/// Operation getDestinationAddresses.
	/// \param wrapper 
	 std::list< JudpAddress * > getDestinationAddresses(transport::Wrapper *wrapper);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const JudpInterface& object);

protected:
	// Member attributes & references
	std::map< uint32_t, std::list< transport::CompressedHeader * > > headerMap;
	system::DatagramSocket datagramSocket;
	JudpAddress *udpAddress;
	system::Thread discoveryThread;
	system::MulticastSocket discoverySocket;

// Start of user code for additional member data
	virtual void *recvThreadMethod();
	virtual void *sendThreadMethod();
	void *discoveryThreadMethod();

	void receiveJudpPacket(JudpPacket &packet);
	void handleLargeMessagePart(Wrapper *wrapper);
	void sendLargeMessage(Wrapper *wrapper);
	void sendMessage(Wrapper* wrapper);

	std::map< transport::Address, JudpLargeMessageBuffer *> largeMessageMap;
// End of user code

}; // class JudpInterface

// Start of user code for inline functions
// End of user code



} // namespace AS5669
} // namespace transport
} // namespace openjaus

#endif // AS5669_JUDPINTERFACE_H

