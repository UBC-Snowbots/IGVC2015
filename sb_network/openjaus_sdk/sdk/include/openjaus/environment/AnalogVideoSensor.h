/**
\file AnalogVideoSensor.h

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

#ifndef ANALOGVIDEOSENSOR_COMPONENT_H
#define ANALOGVIDEOSENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/environment/AnalogVideoInterface.h"
#include "openjaus/environment/Transitions/AnalogVideoDefaultLoop.h"
#include "openjaus/environment/Transitions/AnalogVideoControlledLoop.h"
#include "openjaus/environment/Triggers/SetAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryAnalogVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/ConfirmAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportAnalogVideoSensorCapabilities.h"
#include "openjaus/environment/VisualSensorInterface.h"
#include "openjaus/environment/Transitions/VisualSensorDefaultLoop.h"
#include "openjaus/environment/Transitions/VisualSensorControlledLoop.h"
#include "openjaus/environment/Triggers/QueryVisualSensorGeometricProperties.h"
#include "openjaus/environment/Triggers/QueryVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryVisualSensorCapabilities.h"
#include "openjaus/environment/Triggers/SetVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/ConfirmVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportVisualSensorGeometricProperties.h"
#include "openjaus/environment/Triggers/ReportVisualSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportVisualSensorCapabilities.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace environment
{

/// \class AnalogVideoSensor AnalogVideoSensor.h
/// \brief %AnalogVideoSensor Component implements the urn:jaus:jss:environmentSensing:AnalogVideo, urn:jaus:jss:environmentSensing:VisualSensor services.
/// The %AnalogVideoSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%AnalogVideo Service</dt>
/// <dd><p>
/// This service provides access to the capabilities and configuration of the analog visual sensor, allowing the
/// controlling component to set the visual sensor to a particular operational profile. The actual transmission of the
/// video stream is outside the scope of this service.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:environmentSensing:AnalogVideo<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:environmentSensing:VisualSensor</dd>
/// </dl></dd>
/// <dt>%VisualSensor Service</dt>
/// <dd><p>
/// This service provides access to the basic capabilities and configuration of a visual sensor, allowing the
/// controlling component to set the visual sensor to a particular operational profile. The Query Sensor Geometric
/// Properties message can be used to determine the geometric relationship between the sensor and the vehicle coordinate
/// system. Three possible coordinate responses are possible; (a) the service does not know the sensor’s position, (b)
/// the sensor coordinate system is fixed with respect to the vehicle and (c) the sensor is attached to some
/// manipulator. These cases are supported by the Report Sensor Geometric Properties message and are described therein.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:environmentSensing:VisualSensor<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:AccessControl</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT AnalogVideoSensor : public core::Base, public environment::AnalogVideoInterface, public environment::VisualSensorInterface
{

public:
	AnalogVideoSensor();
	virtual ~AnalogVideoSensor();

	/// \brief Send a ReportAnalogVideoSensorCapabilities message
	/// Send a ReportAnalogVideoSensorCapabilities message
	/// \param[in] queryAnalogVideoSensorCapabilities - Input Trigger.
	/// \return ReportAnalogVideoSensorCapabilities Output Message.
	virtual ReportAnalogVideoSensorCapabilities getReportAnalogVideoSensorCapabilities(QueryAnalogVideoSensorCapabilities *queryAnalogVideoSensorCapabilities);

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setAnalogVideoSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration);

	/// \brief Send sendConfirmAnalogVideoSensorConfiguration message
	/// Send sendConfirmAnalogVideoSensorConfiguration message
	/// \param[in] setAnalogVideoSensorConfiguration - Input Trigger.
	/// \return ConfirmAnalogVideoSensorConfiguration Output Message.
	virtual ConfirmAnalogVideoSensorConfiguration getConfirmAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration);

	/// \brief Send a ReportAnalogVideoSensorConfiguration message
	/// Send a ReportAnalogVideoSensorConfiguration message
	/// \param[in] queryAnalogVideoSensorConfiguration - Input Trigger.
	/// \return ReportAnalogVideoSensorConfiguration Output Message.
	virtual ReportAnalogVideoSensorConfiguration getReportAnalogVideoSensorConfiguration(QueryAnalogVideoSensorConfiguration *queryAnalogVideoSensorConfiguration);

	/// \brief Send a ReportVisualSensorGeometricProperties message
	/// Send a ReportVisualSensorGeometricProperties message
	/// \param[in] queryVisualSensorGeometricProperties - Input Trigger.
	/// \return ReportVisualSensorGeometricProperties Output Message.
	virtual ReportVisualSensorGeometricProperties getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties);

	/// \brief Send sendConfirmVisualSensorConfiguration message
	/// Send sendConfirmVisualSensorConfiguration message
	/// \param[in] setVisualSensorConfiguration - Input Trigger.
	/// \return ConfirmVisualSensorConfiguration Output Message.
	virtual ConfirmVisualSensorConfiguration getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration);

	/// \brief Send a ReportVisualSensorConfiguration message
	/// Send a ReportVisualSensorConfiguration message
	/// \param[in] queryVisualSensorConfiguration - Input Trigger.
	/// \return ReportVisualSensorConfiguration Output Message.
	virtual ReportVisualSensorConfiguration getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration);

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setVisualSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration);

	/// \brief Send a ReportVisualSensorCapabilities message
	/// Send a ReportVisualSensorCapabilities message
	/// \param[in] queryVisualSensorCapabilities - Input Trigger.
	/// \return ReportVisualSensorCapabilities Output Message.
	virtual ReportVisualSensorCapabilities getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities);


	/// \brief isControllingAnalogVideoClient condition.
	/// isControllingAnalogVideoClient condition.
	/// \param[in]  setAnalogVideoSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingAnalogVideoClient(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration);


	/// \brief isControllingVisualSensorClient condition.
	/// isControllingVisualSensorClient condition.
	/// \param[in]  setVisualSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration);


	// Start of user code for additional methods:
	// End of user code

protected:
	AnalogVideoDefaultLoop analogVideoDefaultLoop;
	AnalogVideoControlledLoop analogVideoControlledLoop;


	VisualSensorDefaultLoop visualSensorDefaultLoop;
	VisualSensorControlledLoop visualSensorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // ANALOGVIDEOSENSOR_H
