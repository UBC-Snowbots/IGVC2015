/**
\file StillImage.h

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

#ifndef STILLIMAGE_SERVICE_INTERFACE_H
#define STILLIMAGE_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/environment/VisualSensorInterface.h"
#include "openjaus/environment/Triggers/SetStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryStillImageData.h"
#include "openjaus/environment/Triggers/QueryStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryStillImageSensorCapabilities.h"
#include "openjaus/environment/Triggers/ConfirmStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportStillImageData.h"
#include "openjaus/environment/Triggers/ReportStillImageSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportStillImageSensorCapabilities.h"
namespace openjaus
{
namespace environment
{

/// \class StillImageInterface StillImageInterface.h
/// \brief Provides an abstract interface for the %StillImage service. 
/// <p>
/// This service provides access to the capabilities and configuration of a camera, allowing the controlling component
/// to set the camera to a particular operational profile and to obtain images from the camera.  While this service
/// reports each image individually, the Events service can be used to automatically report images at a specified rate
/// thereby simulating video (such as is typically done to create an MJPEG video stream).
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:environmentSensing:StillImage<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:environmentSensing:VisualSensor</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT StillImageInterface
{
public:
	virtual ~StillImageInterface(){};
	
	/// \brief Send a ReportStillImageSensorCapabilities message
	/// Send a ReportStillImageSensorCapabilities message
	/// \param[in] queryStillImageSensorCapabilities - Input Trigger.
	/// \return ReportStillImageSensorCapabilities Output Message.
	virtual ReportStillImageSensorCapabilities getReportStillImageSensorCapabilities(QueryStillImageSensorCapabilities *queryStillImageSensorCapabilities) = 0;

	/// \brief Send a ReportStillImageSensorConfiguration message
	/// Send a ReportStillImageSensorConfiguration message
	/// \param[in] queryStillImageSensorConfiguration - Input Trigger.
	/// \return ReportStillImageSensorConfiguration Output Message.
	virtual ReportStillImageSensorConfiguration getReportStillImageSensorConfiguration(QueryStillImageSensorConfiguration *queryStillImageSensorConfiguration) = 0;

	/// \brief Send sendConfirmStillImageSensorConfiguration message
	/// Send sendConfirmStillImageSensorConfiguration message
	/// \param[in] setStillImageSensorConfiguration - Input Trigger.
	/// \return ConfirmStillImageSensorConfiguration Output Message.
	virtual ConfirmStillImageSensorConfiguration getConfirmStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration) = 0;

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setStillImageSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration) = 0;

	/// \brief Send a ReportStillImageData message in requested coordinate system
	/// Send a ReportStillImageData message in requested coordinate system
	/// \param[in] queryStillImageData - Input Trigger.
	/// \return ReportStillImageData Output Message.
	virtual ReportStillImageData getReportStillImageData(QueryStillImageData *queryStillImageData) = 0;

	/// \brief Send a ReportStillImageData message in native coordinate system
	/// Send a ReportStillImageData message in native coordinate system
	/// \param[in] queryStillImageData - Input Trigger.
	/// \return ReportStillImageData Output Message.
	virtual ReportStillImageData getReportStillImageDataInNativeSystem(QueryStillImageData *queryStillImageData) = 0;

	/// \brief isControllingStillImageClient condition.
	/// isControllingStillImageClient condition.
	/// \param[in]  setStillImageSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingStillImageClient(SetStillImageSensorConfiguration *setStillImageSensorConfiguration) = 0;

	/// \brief isCoordinateTransformSupported condition.
	/// isCoordinateTransformSupported condition.
	/// \param[in]  queryStillImageData - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCoordinateTransformSupported(QueryStillImageData *queryStillImageData) = 0;

};

} // namespace environment
} // namespace openjaus

#endif // STILLIMAGE_SERVICE_INTERFACE_H
