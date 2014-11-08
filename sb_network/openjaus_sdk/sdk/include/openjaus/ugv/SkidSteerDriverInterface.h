/**
\file SkidSteerDriver.h

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

#ifndef SKIDSTEERDRIVER_SERVICE_INTERFACE_H
#define SKIDSTEERDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/ManagementInterface.h"
#include "openjaus/ugv/Triggers/SetSkidSteerEffort.h"
#include "openjaus/ugv/Triggers/QuerySkidSteerEffort.h"
#include "openjaus/ugv/Triggers/QueryPlatformSpecifications.h"
#include "openjaus/ugv/Triggers/ReportSkidSteerEffort.h"
#include "openjaus/ugv/Triggers/ReportPlatformSpecifications.h"
namespace openjaus
{
namespace ugv
{

/// \class SkidSteerDriverInterface SkidSteerDriverInterface.h
/// \brief Provides an abstract interface for the %SkidSteerDriver service. 
/// <p>
/// The SkidSteer Driver provides the means to control skid steer vehicles
/// </p><br/><br/>
/// <b>URI:</b> %AS4:AS6091:SkidSteerDriver<br/><br/>
/// <b>Version:</b> 0.1<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Management</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT SkidSteerDriverInterface
{
public:
	virtual ~SkidSteerDriverInterface(){};
	
	/// \brief null
	/// null
	/// \param[in]  setSkidSteerEffort - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setSkidSteerEffort(SetSkidSteerEffort *setSkidSteerEffort) = 0;

	/// \brief null
	/// null
	/// \param[in] queryPlatformSpecifications - Input Trigger.
	/// \return ReportPlatformSpecifications Output Message.
	virtual ReportPlatformSpecifications getReportPlatformSpecifications(QueryPlatformSpecifications *queryPlatformSpecifications) = 0;

	/// \brief null
	/// null
	/// \param[in] querySkidSteerEffort - Input Trigger.
	/// \return ReportSkidSteerEffort Output Message.
	virtual ReportSkidSteerEffort getReportSkidSteerEffort(QuerySkidSteerEffort *querySkidSteerEffort) = 0;

	/// \brief isControllingSkidSteerClient condition.
	/// isControllingSkidSteerClient condition.
	/// \param[in]  setSkidSteerEffort - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingSkidSteerClient(SetSkidSteerEffort *setSkidSteerEffort) = 0;

};

} // namespace ugv
} // namespace openjaus

#endif // SKIDSTEERDRIVER_SERVICE_INTERFACE_H
