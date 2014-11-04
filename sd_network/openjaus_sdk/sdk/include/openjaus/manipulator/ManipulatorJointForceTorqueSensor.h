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

#ifndef MANIPULATORJOINTFORCETORQUESENSOR_COMPONENT_H
#define MANIPULATORJOINTFORCETORQUESENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/manipulator/ManipulatorJointForceTorqueSensorInterface.h"
#include "openjaus/manipulator/Transitions/JointForceTorqueSensorDefaultLoop.h"
#include "openjaus/manipulator/Triggers/QueryJointForceTorque.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportJointForceTorque.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorJointForceTorqueSensor ManipulatorJointForceTorqueSensor.h
/// \brief %ManipulatorJointForceTorqueSensor Component implements the urn:jaus:jss:manipulator:ManipulatorJointForceTorqueSensor services.
/// The %ManipulatorJointForceTorqueSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%ManipulatorJointForceTorqueSensor Service</dt>
/// <dd><p>
/// The function of the Joint Force/Torque Sensor is to report the values of instantaneous torques (for revolute joints)
/// and forces (for prismatic joints) that are applied at the individual joints of the manipulator kinematic model when
/// queried.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:ManipulatorJointForceTorqueSensor<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT ManipulatorJointForceTorqueSensor : public core::Base, public manipulator::ManipulatorJointForceTorqueSensorInterface
{

public:
	ManipulatorJointForceTorqueSensor();
	virtual ~ManipulatorJointForceTorqueSensor();

	/// \brief Send Report Joint Force Torque message to the service that sent the query
	/// Send Report Joint Force Torque message to the service that sent the query
	/// \param[in] queryJointForceTorque - Input Trigger.
	/// \return ReportJointForceTorque Output Message.
	virtual ReportJointForceTorque getReportJointForceTorque(QueryJointForceTorque *queryJointForceTorque);

	/// \brief Send a Report Manipulator Specs message
	/// Send a Report Manipulator Specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications);


	// Start of user code for additional methods:
	// End of user code

protected:
	JointForceTorqueSensorDefaultLoop jointForceTorqueSensorDefaultLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // MANIPULATORJOINTFORCETORQUESENSOR_H
