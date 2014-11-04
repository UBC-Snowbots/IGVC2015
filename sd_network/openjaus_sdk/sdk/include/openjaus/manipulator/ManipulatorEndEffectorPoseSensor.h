/**
\file ManipulatorEndEffectorPoseSensor.h

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

#ifndef MANIPULATORENDEFFECTORPOSESENSOR_COMPONENT_H
#define MANIPULATORENDEFFECTORPOSESENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/manipulator/ManipulatorEndEffectorPoseSensorInterface.h"
#include "openjaus/manipulator/Transitions/EndEffectorPoseSensorDefaultLoop.h"
#include "openjaus/manipulator/Transitions/EndEffectorPoseSensorControlledLoop.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryToolOffset.h"
#include "openjaus/manipulator/Triggers/SetToolOffset.h"
#include "openjaus/manipulator/Triggers/QueryEndEffectorPose.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportToolOffset.h"
#include "openjaus/manipulator/Triggers/ReportEndEffectorPose.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorEndEffectorPoseSensor ManipulatorEndEffectorPoseSensor.h
/// \brief %ManipulatorEndEffectorPoseSensor Component implements the urn:jaus:jss:manipulator:ManipulatorEndEffectorPoseSensor services.
/// The %ManipulatorEndEffectorPoseSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%ManipulatorEndEffectorPoseSensor Service</dt>
/// <dd><p>
/// The function of the End Effector Pose Sensor Service is to report the position and orientation of the tool tip with
/// respect to the vehicle coordinate system.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:ManipulatorEndEffectorPoseSensor<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT ManipulatorEndEffectorPoseSensor : public core::Base, public manipulator::ManipulatorEndEffectorPoseSensorInterface
{

public:
	ManipulatorEndEffectorPoseSensor();
	virtual ~ManipulatorEndEffectorPoseSensor();

	/// \brief Set the location of the tool tip as measured in the end effector coordinate system.
	/// Set the location of the tool tip as measured in the end effector coordinate system.
	/// \param[in]  queryToolOffset - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setToolOffset(QueryToolOffset *queryToolOffset);

	/// \brief Send a report manipulator specs message
	/// Send a report manipulator specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications);

	/// \brief Send a report tool offset message
	/// Send a report tool offset message
	/// \param[in] queryToolOffset - Input Trigger.
	/// \return ReportToolOffset Output Message.
	virtual ReportToolOffset getReportToolOffset(QueryToolOffset *queryToolOffset);

	/// \brief Send a report end effector pose message
	/// Send a report end effector pose message
	/// \param[in] queryEndEffectorPose - Input Trigger.
	/// \return ReportEndEffectorPose Output Message.
	virtual ReportEndEffectorPose getReportEndEffectorPose(QueryEndEffectorPose *queryEndEffectorPose);


	/// \brief isControllingEndEffectorPoseSensorClient condition.
	/// isControllingEndEffectorPoseSensorClient condition.
	/// \param[in]  queryToolOffset - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorPoseSensorClient(QueryToolOffset *queryToolOffset);


	// Start of user code for additional methods:
	// End of user code

protected:
	EndEffectorPoseSensorDefaultLoop endEffectorPoseSensorDefaultLoop;
	EndEffectorPoseSensorControlledLoop endEffectorPoseSensorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // MANIPULATORENDEFFECTORPOSESENSOR_H
