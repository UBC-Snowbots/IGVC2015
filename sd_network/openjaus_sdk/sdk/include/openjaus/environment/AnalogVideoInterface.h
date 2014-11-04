/**
\file AnalogVideo.h

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

#ifndef ANALOGVIDEO_SERVICE_INTERFACE_H
#define ANALOGVIDEO_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/environment/VisualSensorInterface.h"
#include "openjaus/environment/Triggers/SetAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryAnalogVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/ConfirmAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportAnalogVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportAnalogVideoSensorCapabilities.h"
namespace openjaus
{
namespace environment
{

/// \class AnalogVideoInterface AnalogVideoInterface.h
/// \brief Provides an abstract interface for the %AnalogVideo service. 
/// <p>
/// This service provides access to the capabilities and configuration of the analog visual sensor, allowing the
/// controlling component to set the visual sensor to a particular operational profile. The actual transmission of the
/// video stream is outside the scope of this service.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:environmentSensing:AnalogVideo<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:environmentSensing:VisualSensor</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT AnalogVideoInterface
{
public:
	virtual ~AnalogVideoInterface(){};
	
	/// \brief Send a ReportAnalogVideoSensorCapabilities message
	/// Send a ReportAnalogVideoSensorCapabilities message
	/// \param[in] queryAnalogVideoSensorCapabilities - Input Trigger.
	/// \return ReportAnalogVideoSensorCapabilities Output Message.
	virtual ReportAnalogVideoSensorCapabilities getReportAnalogVideoSensorCapabilities(QueryAnalogVideoSensorCapabilities *queryAnalogVideoSensorCapabilities) = 0;

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setAnalogVideoSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration) = 0;

	/// \brief Send sendConfirmAnalogVideoSensorConfiguration message
	/// Send sendConfirmAnalogVideoSensorConfiguration message
	/// \param[in] setAnalogVideoSensorConfiguration - Input Trigger.
	/// \return ConfirmAnalogVideoSensorConfiguration Output Message.
	virtual ConfirmAnalogVideoSensorConfiguration getConfirmAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration) = 0;

	/// \brief Send a ReportAnalogVideoSensorConfiguration message
	/// Send a ReportAnalogVideoSensorConfiguration message
	/// \param[in] queryAnalogVideoSensorConfiguration - Input Trigger.
	/// \return ReportAnalogVideoSensorConfiguration Output Message.
	virtual ReportAnalogVideoSensorConfiguration getReportAnalogVideoSensorConfiguration(QueryAnalogVideoSensorConfiguration *queryAnalogVideoSensorConfiguration) = 0;

	/// \brief isControllingAnalogVideoClient condition.
	/// isControllingAnalogVideoClient condition.
	/// \param[in]  setAnalogVideoSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingAnalogVideoClient(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration) = 0;

};

} // namespace environment
} // namespace openjaus

#endif // ANALOGVIDEO_SERVICE_INTERFACE_H
