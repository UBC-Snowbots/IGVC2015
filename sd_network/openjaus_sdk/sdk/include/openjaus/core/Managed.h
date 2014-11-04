/**
\file Managed.h

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

#ifndef MANAGED_COMPONENT_H
#define MANAGED_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/core/ManagementInterface.h"
#include "openjaus/core/Transitions/ToReady.h"
#include "openjaus/core/Transitions/Pause.h"
#include "openjaus/core/Transitions/ResetTransition.h"
#include "openjaus/core/Transitions/ShutdownTransition.h"
#include "openjaus/core/Transitions/PushToEmergency.h"
#include "openjaus/core/Transitions/PopFromEmergency.h"
#include "openjaus/core/Transitions/ManagementLoopback.h"
#include "openjaus/core/Transitions/InitializedTransition.h"
#include "openjaus/core/Triggers/Shutdown.h"
#include "openjaus/core/Triggers/Standby.h"
#include "openjaus/core/Triggers/Resume.h"
#include "openjaus/core/Triggers/Reset.h"
#include "openjaus/core/Triggers/SetEmergency.h"
#include "openjaus/core/Triggers/ClearEmergency.h"
#include "openjaus/core/Triggers/QueryStatus.h"
#include "openjaus/core/Triggers/ReportStatus.h"
#include "openjaus/core/Triggers/Initialized.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace core
{

/// \class Managed Managed.h
/// \brief %Managed Component implements the urn:jaus:jss:core:Management services.
/// The %Managed component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%Management Service</dt>
/// <dd><p>
/// The Management Service provides a state machine for component life-cycle management to help clients understand how
/// the component will react to commands and queries.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:Management<br/><br/>
/// <b>Version:</b> 1.1<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:AccessControl</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT Managed : public core::Base, public core::ManagementInterface
{

public:
	Managed();
	virtual ~Managed();

	/// \brief StoreID action with input SetEmergency.
	/// StoreID action with input SetEmergency.
	/// \param[in]  setEmergency - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool storeID(SetEmergency *setEmergency);

	/// \brief DeleteID action with input ClearEmergency.
	/// DeleteID action with input ClearEmergency.
	/// \param[in]  clearEmergency - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool deleteID(ClearEmergency *clearEmergency);

	/// \brief Send action for ReportStatus with input message QueryStatus.
	/// Send action for ReportStatus with input message QueryStatus.
	/// \param[in] queryStatus - Input Trigger.
	/// \return ReportStatus Output Message.
	virtual ReportStatus getReportStatus(QueryStatus *queryStatus);


	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  reset - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(Reset *reset);

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  shutdown - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(Shutdown *shutdown);


	/// \brief isIDStored condition.
	/// isIDStored condition.
	/// \param[in]  clearEmergency - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isIDStored(ClearEmergency *clearEmergency);


	// Start of user code for additional methods:
	void initialized();
	// End of user code

protected:
	ToReady toReady;
	Pause pause;
	ResetTransition resetTransition;
	ShutdownTransition shutdownTransition;
	PushToEmergency pushToEmergency;
	PopFromEmergency popFromEmergency;
	ManagementLoopback managementLoopback;
	InitializedTransition initializedTransition;
	model::State standby;
	model::State ready;
	model::State init;
	model::State shutdown;
	model::State emergency;

	model::StateMachine standbyReady;

	// Start of user code for additional members:
	std::vector< transport::Address > storedIds;
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // MANAGED_H
