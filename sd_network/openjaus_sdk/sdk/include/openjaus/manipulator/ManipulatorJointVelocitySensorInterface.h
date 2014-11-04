/**
\file ManipulatorJointVelocitySensor.h

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

#ifndef MANIPULATORJOINTVELOCITYSENSOR_SERVICE_INTERFACE_H
#define MANIPULATORJOINTVELOCITYSENSOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/QueryJointVelocity.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportJointVelocity.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorJointVelocitySensorInterface ManipulatorJointVelocitySensorInterface.h
/// \brief Provides an abstract interface for the %ManipulatorJointVelocitySensor service. 
/// <p>
/// The function of the Joint Velocity Sensor Service is to report the values of manipulator joint velocities when
/// queried.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:manipulator:ManipulatorJointVelocitySensor<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT ManipulatorJointVelocitySensorInterface
{
public:
	virtual ~ManipulatorJointVelocitySensorInterface(){};
	
	/// \brief Send Report Joint Velocity message to the service that sent the query
	/// Send Report Joint Velocity message to the service that sent the query
	/// \param[in] queryJointVelocity - Input Trigger.
	/// \return ReportJointVelocity Output Message.
	virtual ReportJointVelocity getReportJointVelocity(QueryJointVelocity *queryJointVelocity) = 0;

	/// \brief Send a Report Manipulator Specs message
	/// Send a Report Manipulator Specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications) = 0;

};

} // namespace manipulator
} // namespace openjaus

#endif // MANIPULATORJOINTVELOCITYSENSOR_SERVICE_INTERFACE_H
