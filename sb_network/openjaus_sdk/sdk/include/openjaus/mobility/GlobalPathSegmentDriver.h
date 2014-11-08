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

#ifndef GLOBALPATHSEGMENTDRIVER_COMPONENT_H
#define GLOBALPATHSEGMENTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/mobility/ListManaged.h"
#include "openjaus/mobility/GlobalPathSegmentDriverInterface.h"
#include "openjaus/mobility/Transitions/GpsdDefaultLoop.h"
#include "openjaus/mobility/Transitions/GpsdControlledLoop.h"
#include "openjaus/mobility/Transitions/GpsdReadyLoop.h"
#include "openjaus/mobility/Triggers/QueryGlobalPathSegment.h"
#include "openjaus/mobility/Triggers/ReportGlobalPathSegment.h"
#include "openjaus/mobility/Triggers/SetGlobalPathSegment.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/ExecuteList.h"
#include "openjaus/mobility/Triggers/QueryActiveElement.h"
#include "openjaus/mobility/Triggers/ReportActiveElement.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class GlobalPathSegmentDriver GlobalPathSegmentDriver.h
/// \brief %GlobalPathSegmentDriver Component implements the urn:jaus:jss:mobility:GlobalPathSegmentDriver services.
/// The %GlobalPathSegmentDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%GlobalPathSegmentDriver Service</dt>
/// <dd><p>
/// The function of the Global Path Segment Driver is to perform closed loop control of position and velocity along a
/// path where the path is defined in a generic manner. The Global Path Segment Driver differs from the Waypoint Drivers
/// in that the exact path between “waypoints” is strictly defined. A path segment will be defined by specifying the
/// three-dimensional coordinates of three points, P0, P1, and P2 together with one scalar weighting value w1 as
/// documented in the JAUS Mobility Service Set Specification.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:GlobalPathSegmentDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:mobility:ListManager</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT GlobalPathSegmentDriver : public mobility::ListManaged, public mobility::GlobalPathSegmentDriverInterface
{

public:
	GlobalPathSegmentDriver();
	virtual ~GlobalPathSegmentDriver();

	/// \brief SetGpsdElement action with input SetElement.
	/// SetGpsdElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setGpsdElement(SetElement *setElement);

	/// \brief ExecuteGlobalPathSegmentList action with input ExecuteList.
	/// ExecuteGlobalPathSegmentList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool executeGlobalPathSegmentList(ExecuteList *executeList);

	/// \brief ModifyGpsdTravelSpeed action with input ExecuteList.
	/// ModifyGpsdTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool modifyGpsdTravelSpeed(ExecuteList *executeList);

	/// \brief Send action for ReportGlobalPathSegment with input message QueryGlobalPathSegment.
	/// Send action for ReportGlobalPathSegment with input message QueryGlobalPathSegment.
	/// \param[in] queryGlobalPathSegment - Input Trigger.
	/// \return ReportGlobalPathSegment Output Message.
	virtual ReportGlobalPathSegment getReportGlobalPathSegment(QueryGlobalPathSegment *queryGlobalPathSegment);

	/// \brief Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// \param[in] queryTravelSpeed - Input Trigger.
	/// \return ReportTravelSpeed Output Message.
	virtual ReportTravelSpeed getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed);

	/// \brief Send action for ReportActiveElement with input message QueryActiveElement.
	/// Send action for ReportActiveElement with input message QueryActiveElement.
	/// \param[in] queryActiveElement - Input Trigger.
	/// \return ReportActiveElement Output Message.
	virtual ReportActiveElement getReportActiveElement(QueryActiveElement *queryActiveElement);

	/// \brief Send action for ConfirmElementRequest with input message SetElement.
	/// Send action for ConfirmElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(SetElement *setElement);

	/// \brief Send action for RejectElementRequest with input message SetElement.
	/// Send action for RejectElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(SetElement *setElement);

	virtual void resetGpsdTravelSpeed();


	/// \brief isControllingGpsdClient condition.
	/// isControllingGpsdClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGpsdClient(SetElement *setElement);

	/// \brief isControllingGpsdClient condition.
	/// isControllingGpsdClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGpsdClient(ExecuteList *executeList);


	/// \brief gpsdSegmentExists condition.
	/// gpsdSegmentExists condition.
	/// \param[in]  queryGlobalPathSegment - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gpsdSegmentExists(QueryGlobalPathSegment *queryGlobalPathSegment);


	/// \brief gpsdElementExists condition.
	/// gpsdElementExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gpsdElementExists(ExecuteList *executeList);


	/// \brief isValidGpsdElementRequest condition.
	/// isValidGpsdElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidGpsdElementRequest(SetElement *setElement);


	/// \brief isGpsdElementSupported condition.
	/// isGpsdElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isGpsdElementSupported(SetElement *setElement);


	/// \brief gpsdElementSpecified condition.
	/// gpsdElementSpecified condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gpsdElementSpecified(ExecuteList *executeList);


	// Start of user code for additional methods:
	// End of user code

protected:
	GpsdDefaultLoop gpsdDefaultLoop;
	GpsdControlledLoop gpsdControlledLoop;
	GpsdReadyLoop gpsdReadyLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // GLOBALPATHSEGMENTDRIVER_H
