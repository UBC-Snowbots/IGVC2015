/**
\file GlobalPoseSensor.h

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

#ifndef GLOBALPOSESENSOR_SERVICE_INTERFACE_H
#define GLOBALPOSESENSOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/AccessControlInterface.h"
#include "openjaus/mobility/Triggers/SetGlobalPose.h"
#include "openjaus/mobility/Triggers/SetGeomagneticProperty.h"
#include "openjaus/mobility/Triggers/QueryGlobalPose.h"
#include "openjaus/mobility/Triggers/QueryGeomagneticProperty.h"
#include "openjaus/mobility/Triggers/ReportGlobalPose.h"
#include "openjaus/mobility/Triggers/ReportGeomagneticProperty.h"
namespace openjaus
{
namespace mobility
{

/// \class GlobalPoseSensorInterface GlobalPoseSensorInterface.h
/// \brief Provides an abstract interface for the %GlobalPoseSensor service. 
/// <p>
/// The function of the Global Pose Sensor is to determine the global position and orientation of the platform. The
/// Report Global Pose message provides the position and orientation of the platform. The position of the platform is
/// given in latitude, longitude, and elevation, in accordance with the WGS 84 standard.  Platform orientation is as
/// defined in Section 4 of the JAUS Mobility Service Set Specification.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:GlobalPoseSensor<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:AccessControl</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT GlobalPoseSensorInterface
{
public:
	virtual ~GlobalPoseSensorInterface(){};
	
	/// \brief Send Report Global Pose message to the component that sent the query.
	/// Send Report Global Pose message to the component that sent the query.
	/// \param[in] queryGlobalPose - Input Trigger.
	/// \return ReportGlobalPose Output Message.
	virtual ReportGlobalPose getReportGlobalPose(QueryGlobalPose *queryGlobalPose) = 0;

	/// \brief Send action for ReportGeomagneticProperty with input message QueryGeomagneticProperty.
	/// Send action for ReportGeomagneticProperty with input message QueryGeomagneticProperty.
	/// \param[in] queryGeomagneticProperty - Input Trigger.
	/// \return ReportGeomagneticProperty Output Message.
	virtual ReportGeomagneticProperty getReportGeomagneticProperty(QueryGeomagneticProperty *queryGeomagneticProperty) = 0;

	/// \brief UpdateGlobalPose action with input SetGlobalPose.
	/// UpdateGlobalPose action with input SetGlobalPose.
	/// \param[in]  setGlobalPose - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateGlobalPose(SetGlobalPose *setGlobalPose) = 0;

	/// \brief UpdateGeomagneticProperty action with input SetGeomagneticProperty.
	/// UpdateGeomagneticProperty action with input SetGeomagneticProperty.
	/// \param[in]  setGeomagneticProperty - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateGeomagneticProperty(SetGeomagneticProperty *setGeomagneticProperty) = 0;

	/// \brief isControllingGposClient condition.
	/// isControllingGposClient condition.
	/// \param[in]  setGlobalPose - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGposClient(SetGlobalPose *setGlobalPose) = 0;

	/// \brief isControllingGposClient condition.
	/// isControllingGposClient condition.
	/// \param[in]  setGeomagneticProperty - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGposClient(SetGeomagneticProperty *setGeomagneticProperty) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // GLOBALPOSESENSOR_SERVICE_INTERFACE_H
