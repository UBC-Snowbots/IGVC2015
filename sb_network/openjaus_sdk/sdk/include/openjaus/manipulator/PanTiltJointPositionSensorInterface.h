/**
\file PanTiltJointPositionSensor.h

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

#ifndef PANTILTJOINTPOSITIONSENSOR_SERVICE_INTERFACE_H
#define PANTILTJOINTPOSITIONSENSOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/QueryPanTiltJointPosition.h"
#include "openjaus/manipulator/Triggers/QueryPanTiltSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportPanTiltJointPosition.h"
#include "openjaus/manipulator/Triggers/ReportPanTiltSpecifications.h"
namespace openjaus
{
namespace manipulator
{

/// \class PanTiltJointPositionSensorInterface PanTiltJointPositionSensorInterface.h
/// \brief Provides an abstract interface for the %PanTiltJointPositionSensor service. 
/// <p>
/// The function of the Pan Tilt Joint Position Sensor Service is to report the values of the two joint angles of the
/// pan tilt mechanism when queried.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:manipulator:PanTiltJointPositionSensor<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT PanTiltJointPositionSensorInterface
{
public:
	virtual ~PanTiltJointPositionSensorInterface(){};
	
	/// \brief Send Report Pan Tilt Joint Position message to the service that sent the query
	/// Send Report Pan Tilt Joint Position message to the service that sent the query
	/// \param[in] queryPanTiltJointPosition - Input Trigger.
	/// \return ReportPanTiltJointPosition Output Message.
	virtual ReportPanTiltJointPosition getReportPanTiltJointPosition(QueryPanTiltJointPosition *queryPanTiltJointPosition) = 0;

	/// \brief Send a Report Pan Tilt Specifications message
	/// Send a Report Pan Tilt Specifications message
	/// \param[in] queryPanTiltSpecifications - Input Trigger.
	/// \return ReportPanTiltSpecifications Output Message.
	virtual ReportPanTiltSpecifications getReportPanTiltSpecifications(QueryPanTiltSpecifications *queryPanTiltSpecifications) = 0;

};

} // namespace manipulator
} // namespace openjaus

#endif // PANTILTJOINTPOSITIONSENSOR_SERVICE_INTERFACE_H
