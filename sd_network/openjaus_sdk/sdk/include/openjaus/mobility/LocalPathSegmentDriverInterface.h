/**
\file LocalPathSegmentDriver.h

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

#ifndef LOCALPATHSEGMENTDRIVER_SERVICE_INTERFACE_H
#define LOCALPATHSEGMENTDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/mobility/ListManagerInterface.h"
#include "openjaus/mobility/Triggers/QueryLocalPathSegment.h"
#include "openjaus/mobility/Triggers/ReportLocalPathSegment.h"
#include "openjaus/mobility/Triggers/SetLocalPathSegment.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/ExecuteList.h"
#include "openjaus/mobility/Triggers/QueryActiveElement.h"
#include "openjaus/mobility/Triggers/ReportActiveElement.h"
namespace openjaus
{
namespace mobility
{

/// \class LocalPathSegmentDriverInterface LocalPathSegmentDriverInterface.h
/// \brief Provides an abstract interface for the %LocalPathSegmentDriver service. 
/// <p>
/// The function of the Local Path Segment Driver is to perform closed loop control of position and velocity along a
/// path where the path is defined in a generic manner. The Local Path Segment Driver differs from the Waypoint Drivers
/// in that the exact path between “s” is strictly defined. A path segment will be defined by specifying the
/// three-dimensional coordinates of three points, P0, P1, and P2 together with one scalar weighting value w1 as
/// documented in the JAUS Mobility Service Set Specification.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:LocalPathSegmentDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:mobility:ListManager</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT LocalPathSegmentDriverInterface
{
public:
	virtual ~LocalPathSegmentDriverInterface(){};
	
	/// \brief SetLpsdElement action with input SetElement.
	/// SetLpsdElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setLpsdElement(SetElement *setElement) = 0;

	/// \brief ExecuteLocalPathSegmentList action with input ExecuteList.
	/// ExecuteLocalPathSegmentList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool executeLocalPathSegmentList(ExecuteList *executeList) = 0;

	/// \brief ModifyLpsdTravelSpeed action with input ExecuteList.
	/// ModifyLpsdTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool modifyLpsdTravelSpeed(ExecuteList *executeList) = 0;

	/// \brief Send action for ReportLocalPathSegment with input message QueryLocalPathSegment.
	/// Send action for ReportLocalPathSegment with input message QueryLocalPathSegment.
	/// \param[in] queryLocalPathSegment - Input Trigger.
	/// \return ReportLocalPathSegment Output Message.
	virtual ReportLocalPathSegment getReportLocalPathSegment(QueryLocalPathSegment *queryLocalPathSegment) = 0;

	/// \brief Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// \param[in] queryTravelSpeed - Input Trigger.
	/// \return ReportTravelSpeed Output Message.
	virtual ReportTravelSpeed getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed) = 0;

	/// \brief Send action for ReportActiveElement with input message QueryActiveElement.
	/// Send action for ReportActiveElement with input message QueryActiveElement.
	/// \param[in] queryActiveElement - Input Trigger.
	/// \return ReportActiveElement Output Message.
	virtual ReportActiveElement getReportActiveElement(QueryActiveElement *queryActiveElement) = 0;

	/// \brief Send action for ConfirmElementRequest with input message SetElement.
	/// Send action for ConfirmElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(SetElement *setElement) = 0;

	/// \brief Send action for RejectElementRequest with input message SetElement.
	/// Send action for RejectElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(SetElement *setElement) = 0;

	virtual void resetLpsdTravelSpeed() = 0;
	/// \brief isControllingLpsdClient condition.
	/// isControllingLpsdClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLpsdClient(SetElement *setElement) = 0;

	/// \brief isControllingLpsdClient condition.
	/// isControllingLpsdClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLpsdClient(ExecuteList *executeList) = 0;

	/// \brief lpsdSegmentExists condition.
	/// lpsdSegmentExists condition.
	/// \param[in]  queryLocalPathSegment - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lpsdSegmentExists(QueryLocalPathSegment *queryLocalPathSegment) = 0;

	/// \brief lpsdElementExists condition.
	/// lpsdElementExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lpsdElementExists(ExecuteList *executeList) = 0;

	/// \brief isValidLpsdElementRequest condition.
	/// isValidLpsdElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidLpsdElementRequest(SetElement *setElement) = 0;

	/// \brief isLpsdElementSupported condition.
	/// isLpsdElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isLpsdElementSupported(SetElement *setElement) = 0;

	/// \brief lpsdElementSpecified condition.
	/// lpsdElementSpecified condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lpsdElementSpecified(ExecuteList *executeList) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // LOCALPATHSEGMENTDRIVER_SERVICE_INTERFACE_H
