/**
\file PrimitiveManipulator.h

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

#ifndef PRIMITIVEMANIPULATOR_COMPONENT_H
#define PRIMITIVEMANIPULATOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/manipulator/PrimitiveManipulatorInterface.h"
#include "openjaus/manipulator/Transitions/PrimitiveManipulatorDefaultLoop.h"
#include "openjaus/manipulator/Transitions/PrimitiveManipulatorControlledLoop.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryJointEffort.h"
#include "openjaus/manipulator/Triggers/SetJointEffort.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportJointEffort.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class PrimitiveManipulator PrimitiveManipulator.h
/// \brief %PrimitiveManipulator Component implements the urn:jaus:jss:manipulator:PrimitiveManipulator services.
/// The %PrimitiveManipulator component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%PrimitiveManipulator Service</dt>
/// <dd><p>
/// This service is the low level interface to a manipulator arm. When queried, the service will reply with a
/// description of the manipulator’s specification parameters,  axes range of motion, and axes velocity limits.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:PrimitiveManipulator<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT PrimitiveManipulator : public core::Base, public manipulator::PrimitiveManipulatorInterface
{

public:
	PrimitiveManipulator();
	virtual ~PrimitiveManipulator();

	/// \brief Set the joint motion efforts for the manipulator.  The manipulator joints move accordingly
	/// Set the joint motion efforts for the manipulator.  The manipulator joints move accordingly
	/// \param[in]  setJointEffort - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setJointEffort(SetJointEffort *setJointEffort);

	/// \brief Send a report manipulator specs message
	/// Send a report manipulator specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications);

	/// \brief Send a report joint effort message
	/// Send a report joint effort message
	/// \param[in] queryJointEffort - Input Trigger.
	/// \return ReportJointEffort Output Message.
	virtual ReportJointEffort getReportJointEffort(QueryJointEffort *queryJointEffort);


	/// \brief isControllingPrimitiveManipulatorClient condition.
	/// isControllingPrimitiveManipulatorClient condition.
	/// \param[in]  setJointEffort - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingPrimitiveManipulatorClient(SetJointEffort *setJointEffort);


	// Start of user code for additional methods:
	// End of user code

protected:
	PrimitiveManipulatorDefaultLoop primitiveManipulatorDefaultLoop;
	PrimitiveManipulatorControlledLoop primitiveManipulatorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // PRIMITIVEMANIPULATOR_H
