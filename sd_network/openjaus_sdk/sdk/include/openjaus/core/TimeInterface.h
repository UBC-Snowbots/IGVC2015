/**
\file Time.h

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

#ifndef TIME_SERVICE_INTERFACE_H
#define TIME_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/AccessControlInterface.h"
#include "openjaus/core/Triggers/SetTime.h"
#include "openjaus/core/Triggers/QueryTime.h"
#include "openjaus/core/Triggers/ReportTime.h"
namespace openjaus
{
namespace core
{

/// \class TimeInterface TimeInterface.h
/// \brief Provides an abstract interface for the %Time service. 
/// <p>
/// The Time Service allows clients to query and set the system time for the component. Note that exclusive control is
/// required to set the time, but is not required to query it. 
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:core:Time<br/><br/>
/// <b>Version:</b> 1.1<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:AccessControl</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT TimeInterface
{
public:
	virtual ~TimeInterface(){};
	
	/// \brief Set the time to the specified time.
	/// Set the time to the specified time.
	/// \param[in]  setTime - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setTime(SetTime *setTime) = 0;

	/// \brief Send a Report Time message.
	/// Send a Report Time message.
	/// \param[in] queryTime - Input Trigger.
	/// \return ReportTime Output Message.
	virtual ReportTime getReportTime(QueryTime *queryTime) = 0;

	/// \brief True if the message that triggered the transition is received from the client that is in control of this service
	/// True if the message that triggered the transition is received from the client that is in control of this service
	/// \param[in]  setTime - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingClient(SetTime *setTime) = 0;

};

} // namespace core
} // namespace openjaus

#endif // TIME_SERVICE_INTERFACE_H
