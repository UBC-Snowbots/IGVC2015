/**
\file Management.h

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

#ifndef MANAGEMENT_SERVICE_INTERFACE_H
#define MANAGEMENT_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/AccessControlInterface.h"
#include "openjaus/core/Triggers/Shutdown.h"
#include "openjaus/core/Triggers/Standby.h"
#include "openjaus/core/Triggers/Resume.h"
#include "openjaus/core/Triggers/Reset.h"
#include "openjaus/core/Triggers/SetEmergency.h"
#include "openjaus/core/Triggers/ClearEmergency.h"
#include "openjaus/core/Triggers/QueryStatus.h"
#include "openjaus/core/Triggers/ReportStatus.h"
#include "openjaus/core/Triggers/Initialized.h"
namespace openjaus
{
namespace core
{

/// \class ManagementInterface ManagementInterface.h
/// \brief Provides an abstract interface for the %Management service. 
/// <p>
/// The Management Service provides a state machine for component life-cycle management to help clients understand how
/// the component will react to commands and queries.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:core:Management<br/><br/>
/// <b>Version:</b> 1.1<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:AccessControl</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT ManagementInterface
{
public:
	virtual ~ManagementInterface(){};
	
	/// \brief StoreID action with input SetEmergency.
	/// StoreID action with input SetEmergency.
	/// \param[in]  setEmergency - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool storeID(SetEmergency *setEmergency) = 0;

	/// \brief DeleteID action with input ClearEmergency.
	/// DeleteID action with input ClearEmergency.
	/// \param[in]  clearEmergency - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool deleteID(ClearEmergency *clearEmergency) = 0;

	/// \brief Send action for ReportStatus with input message QueryStatus.
	/// Send action for ReportStatus with input message QueryStatus.
	/// \param[in] queryStatus - Input Trigger.
	/// \return ReportStatus Output Message.
	virtual ReportStatus getReportStatus(QueryStatus *queryStatus) = 0;

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  reset - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(Reset *reset) = 0;

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  shutdown - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(Shutdown *shutdown) = 0;

	/// \brief isIDStored condition.
	/// isIDStored condition.
	/// \param[in]  clearEmergency - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isIDStored(ClearEmergency *clearEmergency) = 0;

};

} // namespace core
} // namespace openjaus

#endif // MANAGEMENT_SERVICE_INTERFACE_H
