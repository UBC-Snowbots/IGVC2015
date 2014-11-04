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

#ifndef PRIMITIVEENDEFFECTOR_SERVICE_INTERFACE_H
#define PRIMITIVEENDEFFECTOR_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/QueryEndEffectorSpecification.h"
#include "openjaus/manipulator/Triggers/QueryEndEffectorEffort.h"
#include "openjaus/manipulator/Triggers/SetEndEffectorEffort.h"
#include "openjaus/manipulator/Triggers/ReportEndEffectorSpecification.h"
#include "openjaus/manipulator/Triggers/ReportEndEffectorEffort.h"
namespace openjaus
{
namespace manipulator
{

/// \class PrimitiveEndEffectorInterface PrimitiveEndEffectorInterface.h
/// \brief Provides an abstract interface for the %PrimitiveEndEffector service. 
/// <p>
/// This service is the low level interface to an end effector.  The End Effector is a one degree of freedom
/// manipulator, usually mounted on the end of an n-degree of freedom manipulator.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:manipulator:PrimitiveEndEffector<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT PrimitiveEndEffectorInterface
{
public:
	virtual ~PrimitiveEndEffectorInterface(){};
	
	/// \brief Set the effort for the end effector
	/// Set the effort for the end effector
	/// \param[in]  setEndEffectorEffort - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setEndEffectorEffort(SetEndEffectorEffort *setEndEffectorEffort) = 0;

	/// \brief Send a report End Effector spec message
	/// Send a report End Effector spec message
	/// \param[in] queryEndEffectorSpecification - Input Trigger.
	/// \return ReportEndEffectorSpecification Output Message.
	virtual ReportEndEffectorSpecification getReportEndEffectorSpecification(QueryEndEffectorSpecification *queryEndEffectorSpecification) = 0;

	/// \brief Send a report End Effector effort message
	/// Send a report End Effector effort message
	/// \param[in] queryEndEffectorEffort - Input Trigger.
	/// \return ReportEndEffectorEffort Output Message.
	virtual ReportEndEffectorEffort getReportEndEffectorEffort(QueryEndEffectorEffort *queryEndEffectorEffort) = 0;

	/// \brief isControllingPrimitiveEndEffectorClient condition.
	/// isControllingPrimitiveEndEffectorClient condition.
	/// \param[in]  setEndEffectorEffort - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingPrimitiveEndEffectorClient(SetEndEffectorEffort *setEndEffectorEffort) = 0;

};

} // namespace manipulator
} // namespace openjaus

#endif // PRIMITIVEENDEFFECTOR_SERVICE_INTERFACE_H
