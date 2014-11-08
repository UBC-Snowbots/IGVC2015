/**
\file ManipulatorJointForceTorqueSensor.h

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

#ifndef MANIPULATORJOINTFORCETORQUESENSOR_SERVICE_INTERFACE_H
#define MANIPULATORJOINTFORCETORQUESENSOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/QueryJointForceTorque.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportJointForceTorque.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorJointForceTorqueSensorInterface ManipulatorJointForceTorqueSensorInterface.h
/// \brief Provides an abstract interface for the %ManipulatorJointForceTorqueSensor service. 
/// <p>
/// The function of the Joint Force/Torque Sensor is to report the values of instantaneous torques (for revolute joints)
/// and forces (for prismatic joints) that are applied at the individual joints of the manipulator kinematic model when
/// queried.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:manipulator:ManipulatorJointForceTorqueSensor<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT ManipulatorJointForceTorqueSensorInterface
{
public:
	virtual ~ManipulatorJointForceTorqueSensorInterface(){};
	
	/// \brief Send Report Joint Force Torque message to the service that sent the query
	/// Send Report Joint Force Torque message to the service that sent the query
	/// \param[in] queryJointForceTorque - Input Trigger.
	/// \return ReportJointForceTorque Output Message.
	virtual ReportJointForceTorque getReportJointForceTorque(QueryJointForceTorque *queryJointForceTorque) = 0;

	/// \brief Send a Report Manipulator Specs message
	/// Send a Report Manipulator Specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications) = 0;

};

} // namespace manipulator
} // namespace openjaus

#endif // MANIPULATORJOINTFORCETORQUESENSOR_SERVICE_INTERFACE_H
