/**
\file DigitalVideoSensor.h

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

#ifndef DIGITALVIDEOSENSOR_COMPONENT_H
#define DIGITALVIDEOSENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/environment/DigitalVideoInterface.h"
#include "openjaus/environment/Transitions/DigitalVideoDefaultLoop.h"
#include "openjaus/environment/Transitions/DigitalVideoControlledLoop.h"
#include "openjaus/environment/Triggers/ControlDigitalVideoSensorStream.h"
#include "openjaus/environment/Triggers/SetDigitalVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryDigitalVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryDigitalVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/ConfirmDigitalVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportDigitalVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/ReportDigitalVideoSensorConfiguration.h"
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

/// \class DigitalVideoSensor DigitalVideoSensor.h
/// \brief %DigitalVideoSensor Component implements the urn:jaus:jss:environmentSensing:DigitalVideo, urn:jaus:jss:environmentSensing:VisualSensor services.
/// The %DigitalVideoSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%DigitalVideo Service</dt>
/// <dd><p>
/// This service provides access to the capabilities and configuration of the digital visual sensor, allowing the
/// controlling component to set the visual sensor to a particular operational profile. The actual transmission of the
/// video stream is outside the scope of this service. The ability to start, stop and pause the video stream is provided
/// in the message protocol. There may also be mechanisms in the chosen video transmission protocol to control the video
/// stream. In such situations, the messages defined herein are redundant and either mechanism may be used by sensor's
/// client.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:environmentSensing:DigitalVideo<br/><br/>
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
class OPENJAUS_EXPORT DigitalVideoSensor : public core::Base, public environment::DigitalVideoInterface, public environment::VisualSensorInterface
{

public:
	DigitalVideoSensor();
	virtual ~DigitalVideoSensor();

	/// \brief Modify the video stream according to the specified message.
	/// Modify the video stream according to the specified message.
	/// \param[in]  controlDigitalVideoSensorStream - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool modifyDigitalVideoSensorStream(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream);

	/// \brief Send sendConfirmDigitalVideoSensorConfiguration message
	/// Send sendConfirmDigitalVideoSensorConfiguration message
	/// \param[in] setDigitalVideoSensorConfiguration - Input Trigger.
	/// \return ConfirmDigitalVideoSensorConfiguration Output Message.
	virtual ConfirmDigitalVideoSensorConfiguration getConfirmDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration);

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setDigitalVideoSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration);

	/// \brief Send a ReportDigitalVideoSensorConfiguration message
	/// Send a ReportDigitalVideoSensorConfiguration message
	/// \param[in] queryDigitalVideoSensorConfiguration - Input Trigger.
	/// \return ReportDigitalVideoSensorConfiguration Output Message.
	virtual ReportDigitalVideoSensorConfiguration getReportDigitalVideoSensorConfiguration(QueryDigitalVideoSensorConfiguration *queryDigitalVideoSensorConfiguration);

	/// \brief Send a ReportDigitalVideoSensorCapabilities message
	/// Send a ReportDigitalVideoSensorCapabilities message
	/// \param[in] queryDigitalVideoSensorCapabilities - Input Trigger.
	/// \return ReportDigitalVideoSensorCapabilities Output Message.
	virtual ReportDigitalVideoSensorCapabilities getReportDigitalVideoSensorCapabilities(QueryDigitalVideoSensorCapabilities *queryDigitalVideoSensorCapabilities);

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


	/// \brief isControllingDigitalVideoClient condition.
	/// isControllingDigitalVideoClient condition.
	/// \param[in]  setDigitalVideoSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingDigitalVideoClient(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration);

	/// \brief isControllingDigitalVideoClient condition.
	/// isControllingDigitalVideoClient condition.
	/// \param[in]  controlDigitalVideoSensorStream - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingDigitalVideoClient(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream);


	/// \brief isControllingVisualSensorClient condition.
	/// isControllingVisualSensorClient condition.
	/// \param[in]  setVisualSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration);


	// Start of user code for additional methods:
	// End of user code

protected:
	DigitalVideoDefaultLoop digitalVideoDefaultLoop;
	DigitalVideoControlledLoop digitalVideoControlledLoop;


	VisualSensorDefaultLoop visualSensorDefaultLoop;
	VisualSensorControlledLoop visualSensorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // DIGITALVIDEOSENSOR_H
