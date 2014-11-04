/**
\file VelocityStateDriver.h

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

#ifndef VELOCITYSTATEDRIVER_COMPONENT_H
#define VELOCITYSTATEDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/mobility/VelocityStateDriverInterface.h"
#include "openjaus/mobility/Transitions/VsdDefaultLoop.h"
#include "openjaus/mobility/Transitions/VsdControlledLoop.h"
#include "openjaus/mobility/Transitions/VsdReadyLoop.h"
#include "openjaus/mobility/Triggers/SetVelocityCommand.h"
#include "openjaus/mobility/Triggers/SetAccelerationLimit.h"
#include "openjaus/mobility/Triggers/QueryVelocityCommand.h"
#include "openjaus/mobility/Triggers/QueryAccelerationLimit.h"
#include "openjaus/mobility/Triggers/ReportVelocityCommand.h"
#include "openjaus/mobility/Triggers/ReportAccelerationLimit.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class VelocityStateDriver VelocityStateDriver.h
/// \brief %VelocityStateDriver Component implements the urn:jaus:jss:mobility:VelocityStateDriver services.
/// The %VelocityStateDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%VelocityStateDriver Service</dt>
/// <dd><p>
/// The Velocity State Driver allows for low level control of platform mobility. This service does not imply any
/// particular platform type such as tracked or wheeled, but describes mobility in six degrees of freedom using velocity
/// commands relative to the vehicle’s coordinate system. The function of the Velocity State Driver service is to
/// control the desired linear and angular velocity of a mobile platform. The Velocity State Driver takes the desired
/// velocity as measured with respect to the vehicle coordinate system.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:VelocityStateDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Management</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT VelocityStateDriver : public core::Managed, public mobility::VelocityStateDriverInterface
{

public:
	VelocityStateDriver();
	virtual ~VelocityStateDriver();

	virtual void resetCmdVelocityToDefault();

	/// \brief Send action for ReportVelocityCommand with input message QueryVelocityCommand.
	/// Send action for ReportVelocityCommand with input message QueryVelocityCommand.
	/// \param[in] queryVelocityCommand - Input Trigger.
	/// \return ReportVelocityCommand Output Message.
	virtual ReportVelocityCommand getReportVelocityCommand(QueryVelocityCommand *queryVelocityCommand);

	/// \brief Send action for ReportAccelerationLimit with input message QueryAccelerationLimit.
	/// Send action for ReportAccelerationLimit with input message QueryAccelerationLimit.
	/// \param[in] queryAccelerationLimit - Input Trigger.
	/// \return ReportAccelerationLimit Output Message.
	virtual ReportAccelerationLimit getReportAccelerationLimit(QueryAccelerationLimit *queryAccelerationLimit);

	/// \brief SetCurrentVelocityCommand action with input SetVelocityCommand.
	/// SetCurrentVelocityCommand action with input SetVelocityCommand.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setCurrentVelocityCommand(SetVelocityCommand *setVelocityCommand);

	/// \brief SetVelocityLimit action with input SetVelocityCommand.
	/// SetVelocityLimit action with input SetVelocityCommand.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setVelocityLimit(SetVelocityCommand *setVelocityCommand);

	/// \brief SetAccelerationLimit action with input SetAccelerationLimit.
	/// SetAccelerationLimit action with input SetAccelerationLimit.
	/// \param[in]  setAccelerationLimit - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setAccelerationLimit(SetAccelerationLimit *setAccelerationLimit);


	/// \brief isControllingVsdClient condition.
	/// isControllingVsdClient condition.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVsdClient(SetVelocityCommand *setVelocityCommand);

	/// \brief isControllingVsdClient condition.
	/// isControllingVsdClient condition.
	/// \param[in]  setAccelerationLimit - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVsdClient(SetAccelerationLimit *setAccelerationLimit);


	/// \brief isCurrentVelocityCmd condition.
	/// isCurrentVelocityCmd condition.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCurrentVelocityCmd(SetVelocityCommand *setVelocityCommand);


	// Start of user code for additional methods:
	// End of user code

protected:
	VsdDefaultLoop vsdDefaultLoop;
	VsdControlledLoop vsdControlledLoop;
	VsdReadyLoop vsdReadyLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // VELOCITYSTATEDRIVER_H
