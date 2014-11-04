/**
\file PrimitivePanTilt.h

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

#ifndef PRIMITIVEPANTILT_COMPONENT_H
#define PRIMITIVEPANTILT_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/manipulator/PrimitivePanTiltInterface.h"
#include "openjaus/manipulator/Transitions/PrimitivePanTiltDefaultLoop.h"
#include "openjaus/manipulator/Transitions/PrimitivePanTiltControlledLoop.h"
#include "openjaus/manipulator/Triggers/QueryPanTiltSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryPanTiltJointEffort.h"
#include "openjaus/manipulator/Triggers/SetPanTiltJointEffort.h"
#include "openjaus/manipulator/Triggers/ReportPanTiltJointEffort.h"
#include "openjaus/manipulator/Triggers/ReportPanTiltSpecifications.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class PrimitivePanTilt PrimitivePanTilt.h
/// \brief %PrimitivePanTilt Component implements the urn:jaus:jss:manipulator:PrimitivePanTilt services.
/// The %PrimitivePanTilt component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%PrimitivePanTilt Service</dt>
/// <dd><p>
/// The Primitive Pan Tilt Service is the low level interface to a pan tilt mechanism. The Report Pan Tilt Specification
/// Message returns the minimum and maximum allowable value and the maximum velocity for each of the two joints as well
/// as the position and orientation of the pan tilt base coordinate system relative to the vehicle coordinate system. 
/// Motion of the pan tilt mechanism is accomplished via the Set Pan Tilt Joint Effort message.  In this message, each
/// actuator is commanded to move with a percentage of maximum effort.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:PrimitivePanTilt<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT PrimitivePanTilt : public core::Managed, public manipulator::PrimitivePanTiltInterface
{

public:
	PrimitivePanTilt();
	virtual ~PrimitivePanTilt();

	/// \brief Set the joint motion efforts for the two joints of the pan tilt mechanism
	/// Set the joint motion efforts for the two joints of the pan tilt mechanism
	/// \param[in]  setPanTiltJointEffort - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setPanTiltJointEffort(SetPanTiltJointEffort *setPanTiltJointEffort);

	/// \brief Send a report Pan Tilt specifications message
	/// Send a report Pan Tilt specifications message
	/// \param[in] queryPanTiltSpecifications - Input Trigger.
	/// \return ReportPanTiltSpecifications Output Message.
	virtual ReportPanTiltSpecifications getReportPanTiltSpecifications(QueryPanTiltSpecifications *queryPanTiltSpecifications);

	/// \brief Send a report Pan Tilt joint effort message
	/// Send a report Pan Tilt joint effort message
	/// \param[in] queryPanTiltJointEffort - Input Trigger.
	/// \return ReportPanTiltJointEffort Output Message.
	virtual ReportPanTiltJointEffort getReportPanTiltJointEffort(QueryPanTiltJointEffort *queryPanTiltJointEffort);


	/// \brief isControllingPrimitivePanTiltClient condition.
	/// isControllingPrimitivePanTiltClient condition.
	/// \param[in]  setPanTiltJointEffort - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingPrimitivePanTiltClient(SetPanTiltJointEffort *setPanTiltJointEffort);


	// Start of user code for additional methods:
	// End of user code

protected:
	PrimitivePanTiltDefaultLoop primitivePanTiltDefaultLoop;
	PrimitivePanTiltControlledLoop primitivePanTiltControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // PRIMITIVEPANTILT_H
