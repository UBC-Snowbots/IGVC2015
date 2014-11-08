/**
\file Transport.h

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

#ifndef TRANSPORT_COMPONENT_H
#define TRANSPORT_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/TransportInterface.h"
#include "openjaus/core/Transitions/PolicyLoop.h"
#include "openjaus/core/Triggers/Receive.h"
#include "openjaus/core/Triggers/Send.h"
#include "openjaus/core/Triggers/QueryTransportPolicy.h"
#include "openjaus/core/Triggers/ReportTransportPolicy.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace core
{

/// \class Transport Transport.h
/// \brief %Transport Component implements the urn:jaus:jss:core:Transport services.
/// The %Transport component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%Transport Service</dt>
/// <dd><p>
/// The transport service acts as an interface to the JAUS transport layer. It models an abstract bi-directional
/// communication channel (input queue and output queue) whose primary function is to provide the capability of sending
/// messages to a single destination endpoint or broadcasting messages to all endpoints in the system, and to receive a
/// message from any source endpoint. It also provides the capability to prioritize the delivery of sent messages.\nThis
/// service establishes a communication endpoint whose address is defined by a triple {SubsystemID, NodeID, ComponentID}
/// as specified by the Send and Receive internal events. Other services that need to utilize the communication channel
/// provided by the transport service must inherit from the transport service.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:Transport<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT Transport : public model::Component, public core::TransportInterface
{

public:
	Transport();
	virtual ~Transport();

	/// Convert the destination address into an unsigned integer such that the ComponentID maps to the least significant byte, NodeID to the next least significant byte and SubsystemID maps onto the remaining two bytes of the integer. Package the message as specified by the transport layer specification and send it to its destination as per the specified priority.
    virtual bool enqueue(model::Trigger *trigger);

	/// Package the message as specified by the transport layer specification and send it to all endpoints in the local subsystem.
    virtual bool broadcastLocalEnqueue(model::Trigger *trigger);

	/// Package the message as specified by the transport layer specification and send it to all endpoints on all subsystems.
    virtual bool broadcastGlobalEnqueue(model::Trigger *trigger);

	virtual bool sendMessage(model::Trigger *trigger);

	/// Broadcasts message to all components within the local node
    virtual bool broadcastToNode(model::Trigger *trigger);

	/// Broadcasts a given message to all nodes in the local subsystem (equivalent to broadcast local enqueue)
    virtual bool broadcastToSubsystem(model::Trigger *trigger);

	/// Broadcasts the message to all subsystems on the JAUS network (equivalent to broadcast global enqueue)
    virtual bool broadcastToSystem(model::Trigger *trigger);

	virtual bool checkTransportPolicy(model::Trigger *trigger);

	/// \brief Send action for ReportTransportPolicy with input message QueryTransportPolicy.
	/// Send action for ReportTransportPolicy with input message QueryTransportPolicy.
	/// \param[in] queryTransportPolicy - Input Trigger.
	/// \return ReportTransportPolicy Output Message.
	virtual ReportTransportPolicy getReportTransportPolicy(QueryTransportPolicy *queryTransportPolicy);

	/// \brief StoreTransportPolicy action with input ReportTransportPolicy.
	/// StoreTransportPolicy action with input ReportTransportPolicy.
	/// \param[in]  reportTransportPolicy - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool storeTransportPolicy(ReportTransportPolicy *reportTransportPolicy);


	// Start of user code for additional methods:
	template <class MessageType, class CallbackClass>
	void addMessageCallback(bool(CallbackClass::*callback)(MessageType &messageRef), CallbackClass* object)
	{
		receive.addMessageCallback<MessageType, CallbackClass>(callback, object);
	}

	template <class CallbackClass>
	void addMessageCallback(bool(CallbackClass::*callback)(model::Trigger *trigger), CallbackClass* object)
	{
		receive.addMessageCallback<CallbackClass>(callback, object);
	}

	template <class MessageType>
	void addMessageCallback(bool(*callback)(MessageType &messageRef))
	{
		receive.addMessageCallback<MessageType>(callback);
	}

	// End of user code

protected:
	PolicyLoop policyLoop;
	model::State receivingState;

	model::StateMachine receive;

	model::StateMachineRunner receiveThread;
	// Start of user code for additional members:
	transport::AS5669::JudpInterface *udpInterface;
	transport::AS5669::JtcpInterface *tcpInterface;
	std::map< uint64_t, transport::Policy * > policyMap;
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // TRANSPORT_H
