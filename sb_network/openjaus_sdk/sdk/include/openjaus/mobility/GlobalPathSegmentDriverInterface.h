/**
\file GlobalPathSegmentDriver.h

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

#ifndef GLOBALPATHSEGMENTDRIVER_SERVICE_INTERFACE_H
#define GLOBALPATHSEGMENTDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/mobility/ListManagerInterface.h"
#include "openjaus/mobility/Triggers/QueryGlobalPathSegment.h"
#include "openjaus/mobility/Triggers/ReportGlobalPathSegment.h"
#include "openjaus/mobility/Triggers/SetGlobalPathSegment.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/ExecuteList.h"
#include "openjaus/mobility/Triggers/QueryActiveElement.h"
#include "openjaus/mobility/Triggers/ReportActiveElement.h"
namespace openjaus
{
namespace mobility
{

/// \class GlobalPathSegmentDriverInterface GlobalPathSegmentDriverInterface.h
/// \brief Provides an abstract interface for the %GlobalPathSegmentDriver service. 
/// <p>
/// The function of the Global Path Segment Driver is to perform closed loop control of position and velocity along a
/// path where the path is defined in a generic manner. The Global Path Segment Driver differs from the Waypoint Drivers
/// in that the exact path between “waypoints” is strictly defined. A path segment will be defined by specifying the
/// three-dimensional coordinates of three points, P0, P1, and P2 together with one scalar weighting value w1 as
/// documented in the JAUS Mobility Service Set Specification.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:GlobalPathSegmentDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:mobility:ListManager</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT GlobalPathSegmentDriverInterface
{
public:
	virtual ~GlobalPathSegmentDriverInterface(){};
	
	/// \brief SetGpsdElement action with input SetElement.
	/// SetGpsdElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setGpsdElement(SetElement *setElement) = 0;

	/// \brief ExecuteGlobalPathSegmentList action with input ExecuteList.
	/// ExecuteGlobalPathSegmentList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool executeGlobalPathSegmentList(ExecuteList *executeList) = 0;

	/// \brief ModifyGpsdTravelSpeed action with input ExecuteList.
	/// ModifyGpsdTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool modifyGpsdTravelSpeed(ExecuteList *executeList) = 0;

	/// \brief Send action for ReportGlobalPathSegment with input message QueryGlobalPathSegment.
	/// Send action for ReportGlobalPathSegment with input message QueryGlobalPathSegment.
	/// \param[in] queryGlobalPathSegment - Input Trigger.
	/// \return ReportGlobalPathSegment Output Message.
	virtual ReportGlobalPathSegment getReportGlobalPathSegment(QueryGlobalPathSegment *queryGlobalPathSegment) = 0;

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

	virtual void resetGpsdTravelSpeed() = 0;
	/// \brief isControllingGpsdClient condition.
	/// isControllingGpsdClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGpsdClient(SetElement *setElement) = 0;

	/// \brief isControllingGpsdClient condition.
	/// isControllingGpsdClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGpsdClient(ExecuteList *executeList) = 0;

	/// \brief gpsdSegmentExists condition.
	/// gpsdSegmentExists condition.
	/// \param[in]  queryGlobalPathSegment - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gpsdSegmentExists(QueryGlobalPathSegment *queryGlobalPathSegment) = 0;

	/// \brief gpsdElementExists condition.
	/// gpsdElementExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gpsdElementExists(ExecuteList *executeList) = 0;

	/// \brief isValidGpsdElementRequest condition.
	/// isValidGpsdElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidGpsdElementRequest(SetElement *setElement) = 0;

	/// \brief isGpsdElementSupported condition.
	/// isGpsdElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isGpsdElementSupported(SetElement *setElement) = 0;

	/// \brief gpsdElementSpecified condition.
	/// gpsdElementSpecified condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gpsdElementSpecified(ExecuteList *executeList) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // GLOBALPATHSEGMENTDRIVER_SERVICE_INTERFACE_H
