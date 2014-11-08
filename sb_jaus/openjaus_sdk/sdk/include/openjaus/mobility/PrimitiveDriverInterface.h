/**
\file PrimitiveDriver.h

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

#ifndef PRIMITIVEDRIVER_SERVICE_INTERFACE_H
#define PRIMITIVEDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/ManagementInterface.h"
#include "openjaus/mobility/Triggers/SetWrenchEffort.h"
#include "openjaus/mobility/Triggers/QueryWrenchEffort.h"
#include "openjaus/mobility/Triggers/ReportWrenchEffort.h"
namespace openjaus
{
namespace mobility
{

/// \class PrimitiveDriverInterface PrimitiveDriverInterface.h
/// \brief Provides an abstract interface for the %PrimitiveDriver service. 
/// <p>
/// 
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:PrimitiveDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Management</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT PrimitiveDriverInterface
{
public:
	virtual ~PrimitiveDriverInterface(){};
	
	/// \brief Send action for ReportWrenchEffort with input message QueryWrenchEffort.
	/// Send action for ReportWrenchEffort with input message QueryWrenchEffort.
	/// \param[in] queryWrenchEffort - Input Trigger.
	/// \return ReportWrenchEffort Output Message.
	virtual ReportWrenchEffort getReportWrenchEffort(QueryWrenchEffort *queryWrenchEffort) = 0;

	/// \brief SetWrenchEffort action with input SetWrenchEffort.
	/// SetWrenchEffort action with input SetWrenchEffort.
	/// \param[in]  setWrenchEffort - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setWrenchEffort(SetWrenchEffort *setWrenchEffort) = 0;

	/// \brief isControllingPdClient condition.
	/// isControllingPdClient condition.
	/// \param[in]  setWrenchEffort - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingPdClient(SetWrenchEffort *setWrenchEffort) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // PRIMITIVEDRIVER_SERVICE_INTERFACE_H
