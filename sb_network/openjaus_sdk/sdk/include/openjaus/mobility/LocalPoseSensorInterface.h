/**
\file LocalPoseSensor.h

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

#ifndef LOCALPOSESENSOR_SERVICE_INTERFACE_H
#define LOCALPOSESENSOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/AccessControlInterface.h"
#include "openjaus/mobility/Triggers/SetLocalPose.h"
#include "openjaus/mobility/Triggers/QueryLocalPose.h"
#include "openjaus/mobility/Triggers/ReportLocalPose.h"
namespace openjaus
{
namespace mobility
{

/// \class LocalPoseSensorInterface LocalPoseSensorInterface.h
/// \brief Provides an abstract interface for the %LocalPoseSensor service. 
/// <p>
/// The function of the Local Pose Sensor is to report the local position and orientation of the platform.  The Report
/// Local Pose message provides the position and orientation of the platform relative to a local reference frame.  The
/// origin of the local reference frame may be altered using the Set Local Pose message, which sets the current position
/// and orientation of the platform to the specified values.  Platform orientation is defined in Section 3 of this
/// document. 
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:LocalPoseSensor<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:AccessControl</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT LocalPoseSensorInterface
{
public:
	virtual ~LocalPoseSensorInterface(){};
	
	/// \brief Send action for ReportLocalPose with input message QueryLocalPose.
	/// Send action for ReportLocalPose with input message QueryLocalPose.
	/// \param[in] queryLocalPose - Input Trigger.
	/// \return ReportLocalPose Output Message.
	virtual ReportLocalPose getReportLocalPose(QueryLocalPose *queryLocalPose) = 0;

	/// \brief UpdateLocalPose action with input SetLocalPose.
	/// UpdateLocalPose action with input SetLocalPose.
	/// \param[in]  setLocalPose - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateLocalPose(SetLocalPose *setLocalPose) = 0;

	/// \brief isControllingLposClient condition.
	/// isControllingLposClient condition.
	/// \param[in]  setLocalPose - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLposClient(SetLocalPose *setLocalPose) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // LOCALPOSESENSOR_SERVICE_INTERFACE_H
