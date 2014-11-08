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

#ifndef LOCALPATHSEGMENTDRIVER_COMPONENT_H
#define LOCALPATHSEGMENTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/mobility/ListManaged.h"
#include "openjaus/mobility/LocalPathSegmentDriverInterface.h"
#include "openjaus/mobility/Transitions/LpsdDefaultLoop.h"
#include "openjaus/mobility/Transitions/LpsdControlledLoop.h"
#include "openjaus/mobility/Transitions/LpsdReadyLoop.h"
#include "openjaus/mobility/Triggers/QueryLocalPathSegment.h"
#include "openjaus/mobility/Triggers/ReportLocalPathSegment.h"
#include "openjaus/mobility/Triggers/SetLocalPathSegment.h"
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

/// \class LocalPathSegmentDriver LocalPathSegmentDriver.h
/// \brief %LocalPathSegmentDriver Component implements the urn:jaus:jss:mobility:LocalPathSegmentDriver services.
/// The %LocalPathSegmentDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%LocalPathSegmentDriver Service</dt>
/// <dd><p>
/// The function of the Local Path Segment Driver is to perform closed loop control of position and velocity along a
/// path where the path is defined in a generic manner. The Local Path Segment Driver differs from the Waypoint Drivers
/// in that the exact path between “s” is strictly defined. A path segment will be defined by specifying the
/// three-dimensional coordinates of three points, P0, P1, and P2 together with one scalar weighting value w1 as
/// documented in the JAUS Mobility Service Set Specification.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:LocalPathSegmentDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:mobility:ListManager</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT LocalPathSegmentDriver : public mobility::ListManaged, public mobility::LocalPathSegmentDriverInterface
{

public:
	LocalPathSegmentDriver();
	virtual ~LocalPathSegmentDriver();

	/// \brief SetLpsdElement action with input SetElement.
	/// SetLpsdElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setLpsdElement(SetElement *setElement);

	/// \brief ExecuteLocalPathSegmentList action with input ExecuteList.
	/// ExecuteLocalPathSegmentList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool executeLocalPathSegmentList(ExecuteList *executeList);

	/// \brief ModifyLpsdTravelSpeed action with input ExecuteList.
	/// ModifyLpsdTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool modifyLpsdTravelSpeed(ExecuteList *executeList);

	/// \brief Send action for ReportLocalPathSegment with input message QueryLocalPathSegment.
	/// Send action for ReportLocalPathSegment with input message QueryLocalPathSegment.
	/// \param[in] queryLocalPathSegment - Input Trigger.
	/// \return ReportLocalPathSegment Output Message.
	virtual ReportLocalPathSegment getReportLocalPathSegment(QueryLocalPathSegment *queryLocalPathSegment);

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

	virtual void resetLpsdTravelSpeed();


	/// \brief isControllingLpsdClient condition.
	/// isControllingLpsdClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLpsdClient(SetElement *setElement);

	/// \brief isControllingLpsdClient condition.
	/// isControllingLpsdClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLpsdClient(ExecuteList *executeList);


	/// \brief lpsdSegmentExists condition.
	/// lpsdSegmentExists condition.
	/// \param[in]  queryLocalPathSegment - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lpsdSegmentExists(QueryLocalPathSegment *queryLocalPathSegment);


	/// \brief lpsdElementExists condition.
	/// lpsdElementExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lpsdElementExists(ExecuteList *executeList);


	/// \brief isValidLpsdElementRequest condition.
	/// isValidLpsdElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidLpsdElementRequest(SetElement *setElement);


	/// \brief isLpsdElementSupported condition.
	/// isLpsdElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isLpsdElementSupported(SetElement *setElement);


	/// \brief lpsdElementSpecified condition.
	/// lpsdElementSpecified condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lpsdElementSpecified(ExecuteList *executeList);


	// Start of user code for additional methods:
	// End of user code

protected:
	LpsdDefaultLoop lpsdDefaultLoop;
	LpsdControlledLoop lpsdControlledLoop;
	LpsdReadyLoop lpsdReadyLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // LOCALPATHSEGMENTDRIVER_H
