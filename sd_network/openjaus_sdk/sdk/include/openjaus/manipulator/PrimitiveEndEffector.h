/**
\file PrimitiveEndEffector.h

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

#ifndef PRIMITIVEENDEFFECTOR_COMPONENT_H
#define PRIMITIVEENDEFFECTOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/manipulator/PrimitiveEndEffectorInterface.h"
#include "openjaus/manipulator/Transitions/PrimitiveEndEffectorDefaultLoop.h"
#include "openjaus/manipulator/Transitions/PrimitiveEndEffectorControlledLoop.h"
#include "openjaus/manipulator/Triggers/QueryEndEffectorSpecification.h"
#include "openjaus/manipulator/Triggers/QueryEndEffectorEffort.h"
#include "openjaus/manipulator/Triggers/SetEndEffectorEffort.h"
#include "openjaus/manipulator/Triggers/ReportEndEffectorSpecification.h"
#include "openjaus/manipulator/Triggers/ReportEndEffectorEffort.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class PrimitiveEndEffector PrimitiveEndEffector.h
/// \brief %PrimitiveEndEffector Component implements the urn:jaus:jss:manipulator:PrimitiveEndEffector services.
/// The %PrimitiveEndEffector component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%PrimitiveEndEffector Service</dt>
/// <dd><p>
/// This service is the low level interface to an end effector.  The End Effector is a one degree of freedom
/// manipulator, usually mounted on the end of an n-degree of freedom manipulator.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:PrimitiveEndEffector<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT PrimitiveEndEffector : public core::Managed, public manipulator::PrimitiveEndEffectorInterface
{

public:
	PrimitiveEndEffector();
	virtual ~PrimitiveEndEffector();

	/// \brief Set the effort for the end effector
	/// Set the effort for the end effector
	/// \param[in]  setEndEffectorEffort - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setEndEffectorEffort(SetEndEffectorEffort *setEndEffectorEffort);

	/// \brief Send a report End Effector spec message
	/// Send a report End Effector spec message
	/// \param[in] queryEndEffectorSpecification - Input Trigger.
	/// \return ReportEndEffectorSpecification Output Message.
	virtual ReportEndEffectorSpecification getReportEndEffectorSpecification(QueryEndEffectorSpecification *queryEndEffectorSpecification);

	/// \brief Send a report End Effector effort message
	/// Send a report End Effector effort message
	/// \param[in] queryEndEffectorEffort - Input Trigger.
	/// \return ReportEndEffectorEffort Output Message.
	virtual ReportEndEffectorEffort getReportEndEffectorEffort(QueryEndEffectorEffort *queryEndEffectorEffort);


	/// \brief isControllingPrimitiveEndEffectorClient condition.
	/// isControllingPrimitiveEndEffectorClient condition.
	/// \param[in]  setEndEffectorEffort - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingPrimitiveEndEffectorClient(SetEndEffectorEffort *setEndEffectorEffort);


	// Start of user code for additional methods:
	// End of user code

protected:
	PrimitiveEndEffectorDefaultLoop primitiveEndEffectorDefaultLoop;
	PrimitiveEndEffectorControlledLoop primitiveEndEffectorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // PRIMITIVEENDEFFECTOR_H
