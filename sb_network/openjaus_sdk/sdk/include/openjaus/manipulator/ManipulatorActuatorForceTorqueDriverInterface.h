/**
\file ManipulatorActuatorForceTorqueDriver.h

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

#ifndef MANIPULATORACTUATORFORCETORQUEDRIVER_SERVICE_INTERFACE_H
#define MANIPULATORACTUATORFORCETORQUEDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryCommandedActuatorForceTorque.h"
#include "openjaus/manipulator/Triggers/SetActuatorForceTorque.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportCommandedActuatorForceTorque.h"
namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorActuatorForceTorqueDriverInterface ManipulatorActuatorForceTorqueDriverInterface.h
/// \brief Provides an abstract interface for the %ManipulatorActuatorForceTorqueDriver service. 
/// <p>
/// The function of the Actuator Force/Torque Driver is to perform closed-loop force control (for a prismatic actuator)
/// and closed-loop torque control (for a revolute actuator).
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:manipulator:ManipulatorActuatorForceTorqueDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT ManipulatorActuatorForceTorqueDriverInterface
{
public:
	virtual ~ManipulatorActuatorForceTorqueDriverInterface(){};
	
	/// \brief Set the desired actuator forces and torques
	/// Set the desired actuator forces and torques
	/// \param[in]  setActuatorForceTorque - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setActuatorForceTorque(SetActuatorForceTorque *setActuatorForceTorque) = 0;

	/// \brief Send a report manipulator specs message
	/// Send a report manipulator specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications) = 0;

	/// \brief Send a Report Commanded Actuator Force Torque message
	/// Send a Report Commanded Actuator Force Torque message
	/// \param[in] queryCommandedActuatorForceTorque - Input Trigger.
	/// \return ReportCommandedActuatorForceTorque Output Message.
	virtual ReportCommandedActuatorForceTorque getReportCommandedActuatorForceTorque(QueryCommandedActuatorForceTorque *queryCommandedActuatorForceTorque) = 0;

	/// \brief isControllingActuatorForceTorqueClient condition.
	/// isControllingActuatorForceTorqueClient condition.
	/// \param[in]  setActuatorForceTorque - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingActuatorForceTorqueClient(SetActuatorForceTorque *setActuatorForceTorque) = 0;

};

} // namespace manipulator
} // namespace openjaus

#endif // MANIPULATORACTUATORFORCETORQUEDRIVER_SERVICE_INTERFACE_H
