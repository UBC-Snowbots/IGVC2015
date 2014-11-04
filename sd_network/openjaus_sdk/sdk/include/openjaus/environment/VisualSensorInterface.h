/**
\file VisualSensor.h

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

#ifndef VISUALSENSOR_SERVICE_INTERFACE_H
#define VISUALSENSOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/AccessControlInterface.h"
#include "openjaus/environment/Triggers/QueryVisualSensorGeometricProperties.h"
#include "openjaus/environment/Triggers/QueryVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryVisualSensorCapabilities.h"
#include "openjaus/environment/Triggers/SetVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/ConfirmVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportVisualSensorGeometricProperties.h"
#include "openjaus/environment/Triggers/ReportVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportVisualSensorCapabilities.h"
namespace openjaus
{
namespace environment
{

/// \class VisualSensorInterface VisualSensorInterface.h
/// \brief Provides an abstract interface for the %VisualSensor service. 
/// <p>
/// This service provides access to the basic capabilities and configuration of a visual sensor, allowing the
/// controlling component to set the visual sensor to a particular operational profile. The Query Sensor Geometric
/// Properties message can be used to determine the geometric relationship between the sensor and the vehicle coordinate
/// system. Three possible coordinate responses are possible; (a) the service does not know the sensor’s position, (b)
/// the sensor coordinate system is fixed with respect to the vehicle and (c) the sensor is attached to some
/// manipulator. These cases are supported by the Report Sensor Geometric Properties message and are described therein.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:environmentSensing:VisualSensor<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:AccessControl</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT VisualSensorInterface
{
public:
	virtual ~VisualSensorInterface(){};
	
	/// \brief Send a ReportVisualSensorGeometricProperties message
	/// Send a ReportVisualSensorGeometricProperties message
	/// \param[in] queryVisualSensorGeometricProperties - Input Trigger.
	/// \return ReportVisualSensorGeometricProperties Output Message.
	virtual ReportVisualSensorGeometricProperties getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties) = 0;

	/// \brief Send sendConfirmVisualSensorConfiguration message
	/// Send sendConfirmVisualSensorConfiguration message
	/// \param[in] setVisualSensorConfiguration - Input Trigger.
	/// \return ConfirmVisualSensorConfiguration Output Message.
	virtual ConfirmVisualSensorConfiguration getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration) = 0;

	/// \brief Send a ReportVisualSensorConfiguration message
	/// Send a ReportVisualSensorConfiguration message
	/// \param[in] queryVisualSensorConfiguration - Input Trigger.
	/// \return ReportVisualSensorConfiguration Output Message.
	virtual ReportVisualSensorConfiguration getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration) = 0;

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setVisualSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration) = 0;

	/// \brief Send a ReportVisualSensorCapabilities message
	/// Send a ReportVisualSensorCapabilities message
	/// \param[in] queryVisualSensorCapabilities - Input Trigger.
	/// \return ReportVisualSensorCapabilities Output Message.
	virtual ReportVisualSensorCapabilities getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities) = 0;

	/// \brief isControllingVisualSensorClient condition.
	/// isControllingVisualSensorClient condition.
	/// \param[in]  setVisualSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration) = 0;

};

} // namespace environment
} // namespace openjaus

#endif // VISUALSENSOR_SERVICE_INTERFACE_H
