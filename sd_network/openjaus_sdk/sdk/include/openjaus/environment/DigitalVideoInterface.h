/**
\file DigitalVideo.h

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

#ifndef DIGITALVIDEO_SERVICE_INTERFACE_H
#define DIGITALVIDEO_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/environment/VisualSensorInterface.h"
#include "openjaus/environment/Triggers/ControlDigitalVideoSensorStream.h"
#include "openjaus/environment/Triggers/SetDigitalVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryDigitalVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/QueryDigitalVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/ConfirmDigitalVideoSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportDigitalVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/ReportDigitalVideoSensorConfiguration.h"
namespace openjaus
{
namespace environment
{

/// \class DigitalVideoInterface DigitalVideoInterface.h
/// \brief Provides an abstract interface for the %DigitalVideo service. 
/// <p>
/// This service provides access to the capabilities and configuration of the digital visual sensor, allowing the
/// controlling component to set the visual sensor to a particular operational profile. The actual transmission of the
/// video stream is outside the scope of this service. The ability to start, stop and pause the video stream is provided
/// in the message protocol. There may also be mechanisms in the chosen video transmission protocol to control the video
/// stream. In such situations, the messages defined herein are redundant and either mechanism may be used by sensor's
/// client.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:environmentSensing:DigitalVideo<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:environmentSensing:VisualSensor</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT DigitalVideoInterface
{
public:
	virtual ~DigitalVideoInterface(){};
	
	/// \brief Modify the video stream according to the specified message.
	/// Modify the video stream according to the specified message.
	/// \param[in]  controlDigitalVideoSensorStream - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool modifyDigitalVideoSensorStream(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream) = 0;

	/// \brief Send sendConfirmDigitalVideoSensorConfiguration message
	/// Send sendConfirmDigitalVideoSensorConfiguration message
	/// \param[in] setDigitalVideoSensorConfiguration - Input Trigger.
	/// \return ConfirmDigitalVideoSensorConfiguration Output Message.
	virtual ConfirmDigitalVideoSensorConfiguration getConfirmDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration) = 0;

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setDigitalVideoSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration) = 0;

	/// \brief Send a ReportDigitalVideoSensorConfiguration message
	/// Send a ReportDigitalVideoSensorConfiguration message
	/// \param[in] queryDigitalVideoSensorConfiguration - Input Trigger.
	/// \return ReportDigitalVideoSensorConfiguration Output Message.
	virtual ReportDigitalVideoSensorConfiguration getReportDigitalVideoSensorConfiguration(QueryDigitalVideoSensorConfiguration *queryDigitalVideoSensorConfiguration) = 0;

	/// \brief Send a ReportDigitalVideoSensorCapabilities message
	/// Send a ReportDigitalVideoSensorCapabilities message
	/// \param[in] queryDigitalVideoSensorCapabilities - Input Trigger.
	/// \return ReportDigitalVideoSensorCapabilities Output Message.
	virtual ReportDigitalVideoSensorCapabilities getReportDigitalVideoSensorCapabilities(QueryDigitalVideoSensorCapabilities *queryDigitalVideoSensorCapabilities) = 0;

	/// \brief isControllingDigitalVideoClient condition.
	/// isControllingDigitalVideoClient condition.
	/// \param[in]  setDigitalVideoSensorConfiguration - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingDigitalVideoClient(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration) = 0;

	/// \brief isControllingDigitalVideoClient condition.
	/// isControllingDigitalVideoClient condition.
	/// \param[in]  controlDigitalVideoSensorStream - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingDigitalVideoClient(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream) = 0;

};

} // namespace environment
} // namespace openjaus

#endif // DIGITALVIDEO_SERVICE_INTERFACE_H
