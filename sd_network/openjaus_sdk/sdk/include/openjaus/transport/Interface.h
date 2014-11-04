/**
\file Interface.h

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
#ifndef TRANSPORT_INTERFACE_H
#define TRANSPORT_INTERFACE_H

#include "openjaus/transport/Wrapper.h"
#include "openjaus/transport/Address.h"
#include "openjaus/transport/WrapperType.h"
#include "openjaus/system/PriorityQueue.h"
#include "openjaus/system/Thread.h"
#include <map>
#include "openjaus/system/NetworkInterface.h"
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
class Address;

/// \class Interface Interface.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Interface 
{
public:
	Interface(); 
	virtual ~Interface();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of recvQueues.
	const std::map< WrapperType, system::PriorityQueue * >& getRecvQueues() const;


	/// Accessor to get the value of sequenceNumber.
	uint16_t getSequenceNumber() const;

	/// Accessor to set value of sequenceNumber.
	/// \param sequenceNumber The value of the new sequenceNumber.
	bool setSequenceNumber(uint16_t sequenceNumber);

	/// Accessor to get the value of recvQueue.
	system::PriorityQueue* getRecvQueue() const;

	/// Accessor to set value of recvQueue.
	/// \param recvQueue The value of the new recvQueue.
	bool setRecvQueue(system::PriorityQueue* recvQueue);

	/// Accessor to get the value of sendQueue.
	const system::PriorityQueue& getSendQueue() const;


	/// Accessor to get the value of recvThread.
	const system::Thread& getRecvThread() const;


	/// Accessor to get the value of sendThread.
	const system::Thread& getSendThread() const;


	/// Accessor to get the value of subsIf.
	system::NetworkInterface* getSubsIf() const;

	/// Accessor to set value of subsIf.
	/// \param subsIf The value of the new subsIf.
	bool setSubsIf(system::NetworkInterface* subsIf);

	/// Accessor to get the value of nodeIf.
	system::NetworkInterface* getNodeIf() const;

	/// Accessor to set value of nodeIf.
	/// \param nodeIf The value of the new nodeIf.
	bool setNodeIf(system::NetworkInterface* nodeIf);

	/// Accessor to get the value of address.
	Address* getAddress() const;

	/// Accessor to set value of address.
	/// \param address The value of the new address.
	bool setAddress(Address* address);

	/// Operation send.
	/// \param message 
	 bool send(Wrapper *message);

	/// Operation open.
	/// \param tAddress 
	 bool open(Address tAddress);

	/// Operation close.
	 bool close();

	/// Operation broadcast.
	/// \param message 
	 bool broadcast(Wrapper message);


	virtual void run();


	virtual void stop();

	/// Operation setRecvQueue.
	/// \param type 
	/// \param queue 
	 bool setRecvQueue(WrapperType type, system::PriorityQueue *queue);

	/// Operation checkAddresses.
	/// \param message 
	 bool checkAddresses(Wrapper *message);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Interface& object);

protected:
	// Member attributes & references
	std::map< WrapperType, system::PriorityQueue * > recvQueues;
	uint16_t sequenceNumber;
	system::PriorityQueue *recvQueue;
	system::PriorityQueue sendQueue;
	system::Thread recvThread;
	system::Thread sendThread;
	system::NetworkInterface *subsIf;
	system::NetworkInterface *nodeIf;
	Address *address;

// Start of user code for additional member data
	virtual void *recvThreadMethod();
	virtual void *sendThreadMethod();
// End of user code

}; // class Interface

// Start of user code for inline functions
// End of user code



} // namespace transport
} // namespace openjaus

#endif // TRANSPORT_INTERFACE_H

