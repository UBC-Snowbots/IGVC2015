/**
\file StillImageSensor.h

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

#ifndef STILLIMAGESENSOR_COMPONENT_H
#define STILLIMAGESENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/environment/StillImageInterface.h"
#include "openjaus/environment/Transitions/StillImageDefaultLoop.h"
#include "openjaus/environment/Transitions/StillImageControlledLoop.h"
#include "openjaus/environment/Triggers/SetStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryStillImageData.h"
#include "openjaus/environment/Triggers/QueryStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryStillImageSensorCapabilities.h"
#include "openjaus/environment/Triggers/ConfirmStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportStillImageData.h"
#include "openjaus/environment/Triggers/ReportStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportStillImageSensorCapabilities.h"
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

/// \class StillImageSensor StillImageSensor.h
/// \brief %StillImageSensor Component implements the urn:jaus:jss:environmentSensing:StillImage, urn:jaus:jss:environmentSensing:VisualSensor services.
/// The %StillImageSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%StillImage Service</dt>
/// <dd><p>
/// This service provides access to the capabilities and configuration of a camera, allowing the controlling component
/// to set the camera to a particular operational profile and to obtain images from the camera.  While this service
/// reports each image individually, the Events service can be used to automatically report images at a specified rate
/// thereby simulating video (such as is typically done to create an MJPEG video stream).
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:environmentSensing:StillImage<br/><br/>
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
class OPENJAUS_EXPORT StillImageSensor : public core::Base, public environment::StillImageInterface, public environment::VisualSensorInterface
{

public:
	StillImageSensor();
	virtual ~StillImageSensor();

	/// \brief Send a ReportStillImageSensorCapabilities message
	/// Send a ReportStillImageSensorCapabilities message
	/// \param[in] queryStillImageSensorCapabilities - Input Trigger.
	/// \return ReportStillImageSensorCapabilities Output Message.
	virtual ReportStillImageSensorCapabilities getReportStillImageSensorCapabilities(QueryStillImageSensorCapabilities *queryStillImageSensorCapabilities);

	/// \brief Send a ReportStillImageSensorConfiguration message
	/// Send a ReportStillImageSensorConfiguration message
	/// \param[in] queryStillImageSensorConfiguration - Input Trigger.
	/// \return ReportStillImageSensorConfiguration Output Message.
	virtual ReportStillImageSensorConfiguration getReportStillImageSensorConfiguration(QueryStillImageSensorConfiguration *queryStillImageSensorConfiguration);

	/// \brief Send sendConfirmStillImageSensorConfiguration message
	/// Send sendConfirmStillImageSensorConfiguration message
	/// \param[in] setStillImageSensorConfiguration - Input Trigger.
	/// \return ConfirmStillImageSensorConfiguration Output Message.
	virtual ConfirmStillImageSensorConfiguration getConfirmStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration);

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setStillImageSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration);

	/// \brief Send a ReportStillImageData message in requested coordinate system
	/// Send a ReportStillImageData message in requested coordinate system
	/// \param[in] queryStillImageData - Input Trigger.
	/// \return ReportStillImageData Output Message.
	virtual ReportStillImageData getReportStillImageData(QueryStillImageData *queryStillImageData);

	/// \brief Send a ReportStillImageData message in native coordinate system
	/// Send a ReportStillImageData message in native coordinate system
	/// \param[in] queryStillImageData - Input Trigger.
	/// \return ReportStillImageData Output Message.
	virtual ReportStillImageData getReportStillImageDataInNativeSystem(QueryStillImageData *queryStillImageData);

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


	/// \brief isControllingStillImageClient condition.
	/// isControllingStillImageClient condition.
	/// \param[in]  setStillImageSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingStillImageClient(SetStillImageSensorConfiguration *setStillImageSensorConfiguration);


	/// \brief isCoordinateTransformSupported condition.
	/// isCoordinateTransformSupported condition.
	/// \param[in]  queryStillImageData - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCoordinateTransformSupported(QueryStillImageData *queryStillImageData);


	/// \brief isControllingVisualSensorClient condition.
	/// isControllingVisualSensorClient condition.
	/// \param[in]  setVisualSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration);


	// Start of user code for additional methods:
	// End of user code

protected:
	StillImageDefaultLoop stillImageDefaultLoop;
	StillImageControlledLoop stillImageControlledLoop;


	VisualSensorDefaultLoop visualSensorDefaultLoop;
	VisualSensorControlledLoop visualSensorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // STILLIMAGESENSOR_H
