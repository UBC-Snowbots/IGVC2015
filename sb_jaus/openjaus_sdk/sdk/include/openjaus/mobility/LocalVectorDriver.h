/**
\file LocalVectorDriver.h

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

#ifndef LOCALVECTORDRIVER_COMPONENT_H
#define LOCALVECTORDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/mobility/LocalVectorDriverInterface.h"
#include "openjaus/mobility/Transitions/LocalVectorDefaultLoop.h"
#include "openjaus/mobility/Transitions/LocalVectorReadyLoop.h"
#include "openjaus/mobility/Triggers/SetLocalVector.h"
#include "openjaus/mobility/Triggers/QueryLocalVector.h"
#include "openjaus/mobility/Triggers/ReportLocalVector.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class LocalVectorDriver LocalVectorDriver.h
/// \brief %LocalVectorDriver Component implements the urn:jaus:jss:mobility:LocalVectorDriver services.
/// The %LocalVectorDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%LocalVectorDriver Service</dt>
/// <dd><p>
/// The Local Vector Driver performs closed loop control of the desired local heading, pitch, roll and speed of a mobile
/// platform. The Local Vector Driver is very similar in function to the Global Vector Driver, the difference being that
/// the desired heading is defined in terms of a local coordinate system as opposed to the global coordinate system. The
/// Local Vector Driver takes as input four pieces of information, i.e. the desired heading, pitch and roll of the
/// platform as measured with respect to a local coordinate system and the desired speed of the platform. The desired
/// heading angle is defined in a right hand sense about the Z axis of the local coordinate system where zero degrees
/// defines a heading that is parallel to the X axis of the local coordinate system. The pitch is the angle about the
/// Y-axis and the roll is the desired angle about the X-axis. The Local Vector Driver also receives data from the Local
/// Pose Sensor and the Velocity State Sensor component. This information allows the Local Vector Driver to perform
/// closed loop control on both the platform’s local orientation and speed.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:LocalVectorDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Management</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT LocalVectorDriver : public core::Managed, public mobility::LocalVectorDriverInterface
{

public:
	LocalVectorDriver();
	virtual ~LocalVectorDriver();

	/// \brief SetLocalVector action with input SetLocalVector.
	/// SetLocalVector action with input SetLocalVector.
	/// \param[in]  setLocalVector - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setLocalVector(SetLocalVector *setLocalVector);

	/// \brief Send action for ReportLocalVector with input message QueryLocalVector.
	/// Send action for ReportLocalVector with input message QueryLocalVector.
	/// \param[in] queryLocalVector - Input Trigger.
	/// \return ReportLocalVector Output Message.
	virtual ReportLocalVector getReportLocalVector(QueryLocalVector *queryLocalVector);


	/// \brief isControllingLvdClient condition.
	/// isControllingLvdClient condition.
	/// \param[in]  setLocalVector - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLvdClient(SetLocalVector *setLocalVector);


	// Start of user code for additional methods:
	// End of user code

protected:
	LocalVectorDefaultLoop localVectorDefaultLoop;
	LocalVectorReadyLoop localVectorReadyLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // LOCALVECTORDRIVER_H
