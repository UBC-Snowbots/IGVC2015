/**
\file Base.h

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

#ifndef BASE_COMPONENT_H
#define BASE_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Events.h"
#include "openjaus/core/DiscoveryInterface.h"
#include "openjaus/core/Transitions/DiscoveryLoopback.h"
#include "openjaus/core/Triggers/RegisterServices.h"
#include "openjaus/core/Triggers/QueryIdentification.h"
#include "openjaus/core/Triggers/QueryConfiguration.h"
#include "openjaus/core/Triggers/QuerySubsystemList.h"
#include "openjaus/core/Triggers/QueryServices.h"
#include "openjaus/core/Triggers/QueryServiceList.h"
#include "openjaus/core/Triggers/ReportIdentification.h"
#include "openjaus/core/Triggers/ReportConfiguration.h"
#include "openjaus/core/Triggers/ReportSubsystemList.h"
#include "openjaus/core/Triggers/ReportServices.h"
#include "openjaus/core/Triggers/ReportServiceList.h"
#include "openjaus/core/LivenessInterface.h"
#include "openjaus/core/Transitions/HeartbeatLoop.h"
#include "openjaus/core/Triggers/QueryHeartbeatPulse.h"
#include "openjaus/core/Triggers/ReportHeartbeatPulse.h"
#include "openjaus/core/ConfigurationInterface.h"
#include "openjaus/core/Transitions/DefaultConfigurationLoop.h"
#include "openjaus/core/Triggers/QueryJausAddress.h"
#include "openjaus/core/Triggers/ReportJausAddress.h"
#include "openjaus/core/AccessControlInterface.h"
#include "openjaus/core/Transitions/NotControlledLoopback.h"
#include "openjaus/core/Transitions/AcceptControlTransition.h"
#include "openjaus/core/Transitions/ControlledLoopback.h"
#include "openjaus/core/Transitions/ReleaseControlTransition.h"
#include "openjaus/core/Transitions/DefaultStateLoop.h"
#include "openjaus/core/Triggers/RequestControl.h"
#include "openjaus/core/Triggers/ReleaseControl.h"
#include "openjaus/core/Triggers/QueryControl.h"
#include "openjaus/core/Triggers/QueryAuthority.h"
#include "openjaus/core/Triggers/SetAuthority.h"
#include "openjaus/core/Triggers/QueryTimeout.h"
#include "openjaus/core/Triggers/ReportControl.h"
#include "openjaus/core/Triggers/RejectControl.h"
#include "openjaus/core/Triggers/ConfirmControl.h"
#include "openjaus/core/Triggers/ReportAuthority.h"
#include "openjaus/core/Triggers/ReportTimeout.h"
#include "openjaus/core/TimeInterface.h"
#include "openjaus/core/Transitions/TimeDefaultLoop.h"
#include "openjaus/core/Transitions/TimeControlledLoop.h"
#include "openjaus/core/Triggers/SetTime.h"
#include "openjaus/core/Triggers/QueryTime.h"
#include "openjaus/core/Triggers/ReportTime.h"
// Start of user code for additional headers:
#include <map>
// End of user code

namespace openjaus
{
namespace core
{

/// \class Base Base.h
/// \brief %Base Component implements the urn:jaus:jss:core:Discovery, urn:jaus:jss:core:Liveness, urn:openjaus:core:Configuration, urn:jaus:jss:core:AccessControl, urn:jaus:jss:core:Time services.
/// The %Base component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%Discovery Service</dt>
/// <dd><p>
/// The process of discovery is conducted at both the node level and the subsystem level.\nThis service supports the
/// discovery of both legacy components defined in the JAUS Reference Architecture versions 3.2+, and new components.
/// The Component IDs of legacy components were fixed at specification time (Primitive Driver = 33 for example) and
/// could contain only one service beyond the core service support. New components may use any component ID that is
/// outside the range of IDs that have been allocated to legacy component definitions. New components can also contain
/// two or more services beyond the core service support. 
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:Discovery<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Events</dd>
/// </dl></dd>
/// <dt>%Liveness Service</dt>
/// <dd><p>
/// This service provides a means to maintain connection liveness between communicating components.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:Liveness<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Events</dd>
/// </dl></dd>
/// <dt>%Configuration Service</dt>
/// <dd><p>
/// The Configuration Service provides runtime configuration of JAUS addresses using a discovery process which uses
/// non-JAUS messages sent over JUDP.
/// </p><br/><br/>
/// <b>URI:</b> urn:openjaus:core:Configuration<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Events</dd>
/// </dl></dd>
/// <dt>%AccessControl Service</dt>
/// <dd><p>
/// The Access Control service offers a basic interface for acquiring preemptable exclusive control to one or more
/// related services that utilize this function. Once the exclusive control is established, the related services shall
/// only execute commands originating from the controlling component. The authority code parameter of this service is
/// used for preemption and is to be set equal to that of its controlling client. This service always grants control to
/// the highest authority client that is requesting exclusive control. Commands from all other clients are ignored
/// unless from a client with higher authority.\nThis service maintains two values, a default value and a current value
/// of a field called authority code. The default value is the value that the service is pre-configured with. Access is
/// provided to clients based on the value of their authority code in comparison to the current value of this
/// service.\nState transitions between the Available and NotAvailable nested states are behaviors that are deferred to
/// service definitions that derive from this service.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:AccessControl<br/><br/>
/// <b>Version:</b> 1.1<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Events</dd>
/// </dl></dd>
/// <dt>%Time Service</dt>
/// <dd><p>
/// The Time Service allows clients to query and set the system time for the component. Note that exclusive control is
/// required to set the time, but is not required to query it. 
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:Time<br/><br/>
/// <b>Version:</b> 1.1<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:AccessControl</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT Base : public core::Events, public core::DiscoveryInterface, public core::LivenessInterface, public core::ConfigurationInterface, public core::AccessControlInterface, public core::TimeInterface
{

public:
	Base();
	virtual ~Base();

	/// \brief PublishServices action with input RegisterServices.
	/// PublishServices action with input RegisterServices.
	/// \param[in]  registerServices - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool publishServices(RegisterServices *registerServices);

	/// \brief Processes incoming report identification message and saves the name in the system tree
	/// Processes incoming report identification message and saves the name in the system tree
	/// \param[in]  reportIdentification - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool storeIdentification(ReportIdentification *reportIdentification);

	/// \brief Processes incoming report service list message and saves the contents in the system tree
	/// Processes incoming report service list message and saves the contents in the system tree
	/// \param[in]  reportServiceList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool storeServiceList(ReportServiceList *reportServiceList);

	/// \brief Processes incoming report configuration message and saves the contents in the system tree
	/// Processes incoming report configuration message and saves the contents in the system tree
	/// \param[in]  reportConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool storeConfiguration(ReportConfiguration *reportConfiguration);

	/// \brief Takes a report identification message and determines if a Query Configuration message should be generated as a response. Sends the response if needed.
	/// Takes a report identification message and determines if a Query Configuration message should be generated as a response. Sends the response if needed.
	/// \param[in]  reportIdentification - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool queryConfiguration(ReportIdentification *reportIdentification);

	/// \brief Send a Report Identification message to the component that sent the query.
	/// Send a Report Identification message to the component that sent the query.
	/// \param[in] queryIdentification - Input Trigger.
	/// \return ReportIdentification Output Message.
	virtual ReportIdentification getReportIdentification(QueryIdentification *queryIdentification);

	/// \brief Send a Report Configuration message to the component that sent the query.
	/// Send a Report Configuration message to the component that sent the query.
	/// \param[in] queryConfiguration - Input Trigger.
	/// \return ReportConfiguration Output Message.
	virtual ReportConfiguration getReportConfiguration(QueryConfiguration *queryConfiguration);

	/// \brief Send a Report Subsystems List message to the component that sent the query.
	/// Send a Report Subsystems List message to the component that sent the query.
	/// \param[in] querySubsystemList - Input Trigger.
	/// \return ReportSubsystemList Output Message.
	virtual ReportSubsystemList getReportSubsystemList(QuerySubsystemList *querySubsystemList);

	/// \brief Send a Report Services message to the component that sent the query.
	/// Send a Report Services message to the component that sent the query.
	/// \param[in] queryServices - Input Trigger.
	/// \return ReportServices Output Message.
	virtual ReportServices getReportServices(QueryServices *queryServices);

	/// \brief Send a Report Service List message to the component that sent the query
	/// Send a Report Service List message to the component that sent the query
	/// \param[in] queryServiceList - Input Trigger.
	/// \return ReportServiceList Output Message.
	virtual ReportServiceList getReportServiceList(QueryServiceList *queryServiceList);

	/// \brief Send a Register Services message to newly identified component.
	/// Send a Register Services message to newly identified component.
	/// \param[in] reportIdentification - Input Trigger.
	/// \return RegisterServices Output Message.
	virtual RegisterServices getRegisterServices(ReportIdentification *reportIdentification);

	/// \brief Send a Query Service List message to newly identified components.
	/// Send a Query Service List message to newly identified components.
	/// \param[in] reportConfiguration - Input Trigger.
	/// \return QueryServiceList Output Message.
	virtual QueryServiceList getQueryServiceList(ReportConfiguration *reportConfiguration);

	/// \brief Send a Report Heartbeat Pulse message to the component that sent the query
	/// Send a Report Heartbeat Pulse message to the component that sent the query
	/// \param[in] queryHeartbeatPulse - Input Trigger.
	/// \return ReportHeartbeatPulse Output Message.
	virtual ReportHeartbeatPulse getReportHeartbeatPulse(QueryHeartbeatPulse *queryHeartbeatPulse);

	/// \brief Processes ReportHeatbeat Messages to see if a new JAUS entity exists
	/// Processes ReportHeatbeat Messages to see if a new JAUS entity exists
	/// \param[in]  reportHeartbeatPulse - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool processHeartbeat(ReportHeartbeatPulse *reportHeartbeatPulse);

	/// \brief SendJausAddress action with input QueryJausAddress.
	/// SendJausAddress action with input QueryJausAddress.
	/// \param[in]  queryJausAddress - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendJausAddress(QueryJausAddress *queryJausAddress);

	/// \brief SetThisJausAddress action with input ReportJausAddress.
	/// SetThisJausAddress action with input ReportJausAddress.
	/// \param[in]  reportJausAddress - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setThisJausAddress(ReportJausAddress *reportJausAddress);

	virtual void sendRejectControlReleased();

	virtual void init();

	/// \brief Send a Report Control message with the specified control value
	/// Send a Report Control message with the specified control value
	/// \param[in] queryControl - Input Trigger.
	/// \return ReportControl Output Message.
	virtual ReportControl getReportControl(QueryControl *queryControl);

	/// \brief Send a Report Authority message to querying client reporting the current authority value of this service
	/// Send a Report Authority message to querying client reporting the current authority value of this service
	/// \param[in] queryAuthority - Input Trigger.
	/// \return ReportAuthority Output Message.
	virtual ReportAuthority getReportAuthority(QueryAuthority *queryAuthority);

	/// \brief Send action for ReportTimeout with input message SendReportTimeout.
	/// \brief Send a Report Timeout message specifying the timeout period of this service
	/// Send a Report Timeout message specifying the timeout period of this service
	/// \param[in]  sendReportTimeout - Input Trigger.
	/// \return ReportTimeout Output Message.
    virtual core::ReportTimeout getReportTimeout(model::Trigger *trigger);

	/// \brief Set the current authority value of this service to the specified authority
	/// Set the current authority value of this service to the specified authority
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setAuthority(RequestControl *requestControl);
    
	/// \brief Set the current authority value of this service to the specified authority
	/// Set the current authority value of this service to the specified authority
	/// \param[in]  setAuthority - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setAuthority(SetAuthority *setAuthority);

	/// \brief Reset the timer
	/// Reset the timer
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool resetTimer(RequestControl *requestControl);

	/// \brief Send a confirm control message with the specified response code to requesting client
	/// Send a confirm control message with the specified response code to requesting client
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendConfirmControlNotAvailable(RequestControl *requestControl);

	/// \brief Send a confirm control message with the specified response code to requesting client
	/// Send a confirm control message with the specified response code to requesting client
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendConfirmControlInsufficientAuthority(RequestControl *requestControl);

	/// \brief Send a confirm control message with the specified response code to requesting client
	/// Send a confirm control message with the specified response code to requesting client
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendConfirmControlAccepted(RequestControl *requestControl);

	/// \brief SendRejectControlToController action with input RequestControl.
	/// SendRejectControlToController action with input RequestControl.
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendRejectControlToController(RequestControl *requestControl);

	/// \brief SendRejectControlNotAvailable action with input ReleaseControl.
	/// SendRejectControlNotAvailable action with input ReleaseControl.
	/// \param[in]  releaseControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendRejectControlNotAvailable(ReleaseControl *releaseControl);

	/// \brief SendRejectControlReleased action with input ReleaseControl.
	/// SendRejectControlReleased action with input ReleaseControl.
	/// \param[in]  releaseControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool sendRejectControlReleased(ReleaseControl *releaseControl);

	/// \brief StoreAddress action with input RequestControl.
	/// StoreAddress action with input RequestControl.
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool storeAddress(RequestControl *requestControl);

	/// \brief Modifies list of controlled components based on confirm or reject messages
	/// Modifies list of controlled components based on confirm or reject messages
	/// \param[in]  confirmControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateControlledList(ConfirmControl *confirmControl);
    
	/// \brief Modifies list of controlled components based on confirm or reject messages
	/// Modifies list of controlled components based on confirm or reject messages
	/// \param[in]  rejectControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateControlledList(RejectControl *rejectControl);

	/// \brief Set the time to the specified time.
	/// Set the time to the specified time.
	/// \param[in]  setTime - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setTime(SetTime *setTime);

	/// \brief Send a Report Time message.
	/// Send a Report Time message.
	/// \param[in] queryTime - Input Trigger.
	/// \return ReportTime Output Message.
	virtual ReportTime getReportTime(QueryTime *queryTime);


	/// \brief True if the default authority code of this service is greater than the authority code of the client service that triggered the corresponding transition
	/// True if the default authority code of this service is greater than the authority code of the client service that triggered the corresponding transition
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isDefaultAuthorityGreater(RequestControl *requestControl);


	/// \brief True if the current authority value of this service is less than the authority code of the client service that triggered the corresponding transition
	/// True if the current authority value of this service is less than the authority code of the client service that triggered the corresponding transition
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCurrentAuthorityLess(RequestControl *requestControl);


	/// \brief True if the value of the authority code received from the client is less than or equal to the current authority value of this service, but greater than or equal to the receiving component�s default authority
	/// True if the value of the authority code received from the client is less than or equal to the current authority value of this service, but greater than or equal to the receiving component�s default authority
	/// \param[in]  setAuthority - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isAuthorityValid(SetAuthority *setAuthority);


	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(RequestControl *requestControl);

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  setAuthority - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(SetAuthority *setAuthority);

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  releaseControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(ReleaseControl *releaseControl);


	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  setTime - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(SetTime *setTime);


	// Start of user code for additional methods:
	void setComponentId(system::Timer *timer);
	void setNodeId(system::Timer *timer);
	void setSubsystemId(system::Timer *timer);
	void sendQueryIdentifications(system::Timer *timer);
	void sendHeartbeats(system::Timer *timer);
	void run();
	void stop();
	// End of user code

protected:
	DiscoveryLoopback discoveryLoopback;


	HeartbeatLoop heartbeatLoop;


	DefaultConfigurationLoop defaultConfigurationLoop;


	NotControlledLoopback notControlledLoopback;
	AcceptControlTransition acceptControlTransition;
	ControlledLoopback controlledLoopback;
	ReleaseControlTransition releaseControlTransition;
	DefaultStateLoop defaultStateLoop;
	model::State notControlled;
	model::State controlled;

	model::StateMachine accessStateMachine;

	TimeDefaultLoop timeDefaultLoop;
	TimeControlledLoop timeControlledLoop;


	// Start of user code for additional members:

	transport::Address controllerAddress;
	uint8_t controllerAuthority;
	uint8_t defaultAuthority;
	std::vector<transport::Address> controlledComponents;
	std::map<int, void (*)(const model::ControlResponse& response) > controlResponseCallbacks;

	system::Timer *timer;
	system::Timer *heartbeatTimer;
	short hbSequenceNumber;

	void *configurationThreadMethod();
	core::RSLSubsystemRecord reportServiceListGetSubsystem(core::QSLSubsystemRecord& querySubsRec);
	core::ServicesNodeRecord reportServiceListGetNode(uint16 subsId, core::QSLNodeRecord& queryNodeRec);
	core::ServicesComponentRecord reportServiceListGetComponent(uint16 subsId, uint8 nodeId, core::QSLComponentRecord &queryCmptRec);
	void addSelfToSystemTree();

public:
	void requestControl(transport::Address destination, void (*responseCallback)(const model::ControlResponse& response) = NULL);
	void releaseControl(transport::Address destination, void (*responseCallback)(const model::ControlResponse& response) = NULL);


	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // BASE_H
