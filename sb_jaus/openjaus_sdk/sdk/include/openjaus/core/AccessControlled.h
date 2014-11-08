/**
\file AccessControlled.h

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

#ifndef ACCESSCONTROLLED_COMPONENT_H
#define ACCESSCONTROLLED_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Events.h"
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
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace core
{

/// \class AccessControlled AccessControlled.h
/// \brief %AccessControlled Component implements the urn:jaus:jss:core:AccessControl services.
/// The %AccessControlled component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
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
/// </dl>
class OPENJAUS_EXPORT AccessControlled : public core::Events, public core::AccessControlInterface
{

public:
	AccessControlled();
	virtual ~AccessControlled();

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


	/// \brief True if the value of the authority code received from the client is less than or equal to the current authority value of this service, but greater than or equal to the receiving component’s default authority
	/// True if the value of the authority code received from the client is less than or equal to the current authority value of this service, but greater than or equal to the receiving component’s default authority
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


	// Start of user code for additional methods:
	// End of user code

protected:
	NotControlledLoopback notControlledLoopback;
	AcceptControlTransition acceptControlTransition;
	ControlledLoopback controlledLoopback;
	ReleaseControlTransition releaseControlTransition;
	DefaultStateLoop defaultStateLoop;
	model::State notControlled;
	model::State controlled;

	model::StateMachine accessStateMachine;

	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // ACCESSCONTROLLED_H
