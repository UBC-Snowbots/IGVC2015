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

#ifndef VELOCITYSTATEDRIVER_SERVICE_INTERFACE_H
#define VELOCITYSTATEDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/ManagementInterface.h"
#include "openjaus/mobility/Triggers/SetVelocityCommand.h"
#include "openjaus/mobility/Triggers/SetAccelerationLimit.h"
#include "openjaus/mobility/Triggers/QueryVelocityCommand.h"
#include "openjaus/mobility/Triggers/QueryAccelerationLimit.h"
#include "openjaus/mobility/Triggers/ReportVelocityCommand.h"
#include "openjaus/mobility/Triggers/ReportAccelerationLimit.h"
namespace openjaus
{
namespace mobility
{

/// \class VelocityStateDriverInterface VelocityStateDriverInterface.h
/// \brief Provides an abstract interface for the %VelocityStateDriver service. 
/// <p>
/// The Velocity State Driver allows for low level control of platform mobility. This service does not imply any
/// particular platform type such as tracked or wheeled, but describes mobility in six degrees of freedom using velocity
/// commands relative to the vehicle’s coordinate system. The function of the Velocity State Driver service is to
/// control the desired linear and angular velocity of a mobile platform. The Velocity State Driver takes the desired
/// velocity as measured with respect to the vehicle coordinate system.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:VelocityStateDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Management</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT VelocityStateDriverInterface
{
public:
	virtual ~VelocityStateDriverInterface(){};
	
	virtual void resetCmdVelocityToDefault() = 0;
	/// \brief Send action for ReportVelocityCommand with input message QueryVelocityCommand.
	/// Send action for ReportVelocityCommand with input message QueryVelocityCommand.
	/// \param[in] queryVelocityCommand - Input Trigger.
	/// \return ReportVelocityCommand Output Message.
	virtual ReportVelocityCommand getReportVelocityCommand(QueryVelocityCommand *queryVelocityCommand) = 0;

	/// \brief Send action for ReportAccelerationLimit with input message QueryAccelerationLimit.
	/// Send action for ReportAccelerationLimit with input message QueryAccelerationLimit.
	/// \param[in] queryAccelerationLimit - Input Trigger.
	/// \return ReportAccelerationLimit Output Message.
	virtual ReportAccelerationLimit getReportAccelerationLimit(QueryAccelerationLimit *queryAccelerationLimit) = 0;

	/// \brief SetCurrentVelocityCommand action with input SetVelocityCommand.
	/// SetCurrentVelocityCommand action with input SetVelocityCommand.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setCurrentVelocityCommand(SetVelocityCommand *setVelocityCommand) = 0;

	/// \brief SetVelocityLimit action with input SetVelocityCommand.
	/// SetVelocityLimit action with input SetVelocityCommand.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setVelocityLimit(SetVelocityCommand *setVelocityCommand) = 0;

	/// \brief SetAccelerationLimit action with input SetAccelerationLimit.
	/// SetAccelerationLimit action with input SetAccelerationLimit.
	/// \param[in]  setAccelerationLimit - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setAccelerationLimit(SetAccelerationLimit *setAccelerationLimit) = 0;

	/// \brief isControllingVsdClient condition.
	/// isControllingVsdClient condition.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVsdClient(SetVelocityCommand *setVelocityCommand) = 0;

	/// \brief isControllingVsdClient condition.
	/// isControllingVsdClient condition.
	/// \param[in]  setAccelerationLimit - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVsdClient(SetAccelerationLimit *setAccelerationLimit) = 0;

	/// \brief isCurrentVelocityCmd condition.
	/// isCurrentVelocityCmd condition.
	/// \param[in]  setVelocityCommand - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCurrentVelocityCmd(SetVelocityCommand *setVelocityCommand) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // VELOCITYSTATEDRIVER_SERVICE_INTERFACE_H
