/**
\file AccessControl.h

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

#ifndef ACCESSCONTROL_SERVICE_INTERFACE_H
#define ACCESSCONTROL_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/EventsInterface.h"
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
namespace openjaus
{
namespace core
{

/// \class AccessControlInterface AccessControlInterface.h
/// \brief Provides an abstract interface for the %AccessControl service. 
/// <p>
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
/// <b>URI:</b> %urn:jaus:jss:core:AccessControl<br/><br/>
/// <b>Version:</b> 1.1<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Events</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT AccessControlInterface
{
public:
	virtual ~AccessControlInterface(){};
	
	virtual void sendRejectControlReleased() = 0;
	virtual void init() = 0;
	/// \brief Send a Report Control message with the specified control value
	/// Send a Report Control message with the specified control value
	/// \param[in] queryControl - Input Trigger.
	/// \return ReportControl Output Message.
	virtual ReportControl getReportControl(QueryControl *queryControl) = 0;

	/// \brief Send a Report Authority message to querying client reporting the current authority value of this service
	/// Send a Report Authority message to querying client reporting the current authority value of this service
	/// \param[in] queryAuthority - Input Trigger.
	/// \return ReportAuthority Output Message.
	virtual ReportAuthority getReportAuthority(QueryAuthority *queryAuthority) = 0;

	/// \brief Send a Report Timeout message specifying the timeout period of this service
	/// Send a Report Timeout message specifying the timeout period of this service
	/// \param[in] trigger - Input Trigger.
	/// \return ReportTimeout Output Message.
	virtual ReportTimeout getReportTimeout(model::Trigger *trigger) = 0;

	/// \brief Set the current authority value of this service to the specified authority
	/// Set the current authority value of this service to the specified authority
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setAuthority(RequestControl *requestControl) = 0;

	/// \brief Set the current authority value of this service to the specified authority
	/// Set the current authority value of this service to the specified authority
	/// \param[in]  setAuthority - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setAuthority(SetAuthority *setAuthority) = 0;

	/// \brief Reset the timer
	/// Reset the timer
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool resetTimer(RequestControl *requestControl) = 0;

	/// \brief Send a confirm control message with the specified response code to requesting client
	/// Send a confirm control message with the specified response code to requesting client
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool sendConfirmControlNotAvailable(RequestControl *requestControl) = 0;

	/// \brief Send a confirm control message with the specified response code to requesting client
	/// Send a confirm control message with the specified response code to requesting client
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool sendConfirmControlInsufficientAuthority(RequestControl *requestControl) = 0;

	/// \brief Send a confirm control message with the specified response code to requesting client
	/// Send a confirm control message with the specified response code to requesting client
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool sendConfirmControlAccepted(RequestControl *requestControl) = 0;

	/// \brief SendRejectControlToController action with input RequestControl.
	/// SendRejectControlToController action with input RequestControl.
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool sendRejectControlToController(RequestControl *requestControl) = 0;

	/// \brief SendRejectControlNotAvailable action with input ReleaseControl.
	/// SendRejectControlNotAvailable action with input ReleaseControl.
	/// \param[in]  releaseControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool sendRejectControlNotAvailable(ReleaseControl *releaseControl) = 0;

	/// \brief SendRejectControlReleased action with input ReleaseControl.
	/// SendRejectControlReleased action with input ReleaseControl.
	/// \param[in]  releaseControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool sendRejectControlReleased(ReleaseControl *releaseControl) = 0;

	/// \brief StoreAddress action with input RequestControl.
	/// StoreAddress action with input RequestControl.
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool storeAddress(RequestControl *requestControl) = 0;

	/// \brief Modifies list of controlled components based on confirm or reject messages
	/// Modifies list of controlled components based on confirm or reject messages
	/// \param[in]  confirmControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateControlledList(ConfirmControl *confirmControl) = 0;

	/// \brief Modifies list of controlled components based on confirm or reject messages
	/// Modifies list of controlled components based on confirm or reject messages
	/// \param[in]  rejectControl - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateControlledList(RejectControl *rejectControl) = 0;

	/// \brief True if the default authority code of this service is greater than the authority code of the client service that triggered the corresponding transition
	/// True if the default authority code of this service is greater than the authority code of the client service that triggered the corresponding transition
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isDefaultAuthorityGreater(RequestControl *requestControl) = 0;

	/// \brief True if the current authority value of this service is less than the authority code of the client service that triggered the corresponding transition
	/// True if the current authority value of this service is less than the authority code of the client service that triggered the corresponding transition
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCurrentAuthorityLess(RequestControl *requestControl) = 0;

	/// \brief True if the value of the authority code received from the client is less than or equal to the current authority value of this service, but greater than or equal to the receiving component’s default authority
	/// True if the value of the authority code received from the client is less than or equal to the current authority value of this service, but greater than or equal to the receiving component’s default authority
	/// \param[in]  setAuthority - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isAuthorityValid(SetAuthority *setAuthority) = 0;

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  requestControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(RequestControl *requestControl) = 0;

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  setAuthority - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(SetAuthority *setAuthority) = 0;

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  releaseControl - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(ReleaseControl *releaseControl) = 0;

};

} // namespace core
} // namespace openjaus

#endif // ACCESSCONTROL_SERVICE_INTERFACE_H
