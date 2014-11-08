/**
\file VelocityStateSensor.h

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

#ifndef VELOCITYSTATESENSOR_COMPONENT_H
#define VELOCITYSTATESENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Events.h"
#include "openjaus/mobility/VelocityStateSensorInterface.h"
#include "openjaus/mobility/Transitions/VelocityStateDefaultLoop.h"
#include "openjaus/mobility/Triggers/QueryVelocityState.h"
#include "openjaus/mobility/Triggers/ReportVelocityState.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class VelocityStateSensor VelocityStateSensor.h
/// \brief %VelocityStateSensor Component implements the urn:jaus:jss:mobility:VelocityStateSensor services.
/// The %VelocityStateSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%VelocityStateSensor Service</dt>
/// <dd><p>
/// The Velocity State Sensor has the responsibility of reporting the instantaneous velocity of the platform.  The
/// velocity state of a rigid body is defined as the set of parameters that are necessary to calculate the velocity of
/// any point in that rigid body.  Six parameters are required to specify a velocity state of a rigid body in terms of
/// some fixed reference coordinate system.  The first three parameters represent the velocity components of a point in
/// the rigid body that is coincident with the origin of the fixed reference.  The second three components represent the
///  instantaneous angular velocity components.  It is possible to represent the six velocity state parameters as a
/// screw, about which the rigid body is rotating and translating along at that instant. The reference frame for the
/// velocity state sensor component is selected as a fixed coordinate system that at this instant is co-located with and
/// aligned with the vehicle or system coordinate system.  Thus the message data ‘velocity x’, ‘velocity y’, and
/// ‘velocity z’ represents the current velocity of the subsystem’s control point at this instant.  For example if
/// ‘velocity x’ has a value of 3 m/sec and ‘velocity y’ and ‘velocity z’ are zero, then the vehicle is moving in the
/// forward direction at a velocity of 3 m/sec.  The message data ‘omega x’, ‘omega  y’, and ‘omega z’ represent the
/// actual rate of change of orientation or angular velocity of the vehicle about its coordinate axes.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:VelocityStateSensor<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Events</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT VelocityStateSensor : public core::Events, public mobility::VelocityStateSensorInterface
{

public:
	VelocityStateSensor();
	virtual ~VelocityStateSensor();

	/// \brief Send action for ReportVelocityState with input message QueryVelocityState.
	/// Send action for ReportVelocityState with input message QueryVelocityState.
	/// \param[in] queryVelocityState - Input Trigger.
	/// \return ReportVelocityState Output Message.
	virtual ReportVelocityState getReportVelocityState(QueryVelocityState *queryVelocityState);


	// Start of user code for additional methods:
	// End of user code

protected:
	VelocityStateDefaultLoop velocityStateDefaultLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // VELOCITYSTATESENSOR_H
