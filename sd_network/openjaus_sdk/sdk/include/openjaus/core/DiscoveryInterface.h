/**
\file Discovery.h

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

#ifndef DISCOVERY_SERVICE_INTERFACE_H
#define DISCOVERY_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/EventsInterface.h"
#include "openjaus/core/Triggers/RegisterServices.h"
#include "openjaus/core/Triggers/QueryIdentification.h"
#include "openjaus/core/Triggers/QueryConfiguration.h"
#include "openjaus/core/Triggers/QuerySubsystemList.h"
#include "openjaus/core/Triggers/QueryServices.h"
#include "openjaus/core/Triggers/QueryServiceList.h"
#include "openjaus/core/Triggers/ReportIdentification.h"
#include "openjaus/core/Triggers/ReportConfiguration.h"
#include "openjaus/core/Triggers/ReportSubsystemList.h"
#include "openjaus/core/Triggers/ReportServices.h"
#include "openjaus/core/Triggers/ReportServiceList.h"
namespace openjaus
{
namespace core
{

/// \class DiscoveryInterface DiscoveryInterface.h
/// \brief Provides an abstract interface for the %Discovery service. 
/// <p>
/// The process of discovery is conducted at both the node level and the subsystem level.\nThis service supports the
/// discovery of both legacy components defined in the JAUS Reference Architecture versions 3.2+, and new components.
/// The Component IDs of legacy components were fixed at specification time (Primitive Driver = 33 for example) and
/// could contain only one service beyond the core service support. New components may use any component ID that is
/// outside the range of IDs that have been allocated to legacy component definitions. New components can also contain
/// two or more services beyond the core service support. 
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:core:Discovery<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Events</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT DiscoveryInterface
{
public:
	virtual ~DiscoveryInterface(){};
	
	/// \brief PublishServices action with input RegisterServices.
	/// PublishServices action with input RegisterServices.
	/// \param[in]  registerServices - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool publishServices(RegisterServices *registerServices) = 0;

	/// \brief Processes incoming report identification message and saves the name in the system tree
	/// Processes incoming report identification message and saves the name in the system tree
	/// \param[in]  reportIdentification - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool storeIdentification(ReportIdentification *reportIdentification) = 0;

	/// \brief Processes incoming report service list message and saves the contents in the system tree
	/// Processes incoming report service list message and saves the contents in the system tree
	/// \param[in]  reportServiceList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool storeServiceList(ReportServiceList *reportServiceList) = 0;

	/// \brief Processes incoming report configuration message and saves the contents in the system tree
	/// Processes incoming report configuration message and saves the contents in the system tree
	/// \param[in]  reportConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool storeConfiguration(ReportConfiguration *reportConfiguration) = 0;

	/// \brief Takes a report identification message and determines if a Query Configuration message should be generated as a response. Sends the response if needed.
	/// Takes a report identification message and determines if a Query Configuration message should be generated as a response. Sends the response if needed.
	/// \param[in]  reportIdentification - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool queryConfiguration(ReportIdentification *reportIdentification) = 0;

	/// \brief Send a Report Identification message to the component that sent the query.
	/// Send a Report Identification message to the component that sent the query.
	/// \param[in] queryIdentification - Input Trigger.
	/// \return ReportIdentification Output Message.
	virtual ReportIdentification getReportIdentification(QueryIdentification *queryIdentification) = 0;

	/// \brief Send a Report Configuration message to the component that sent the query.
	/// Send a Report Configuration message to the component that sent the query.
	/// \param[in] queryConfiguration - Input Trigger.
	/// \return ReportConfiguration Output Message.
	virtual ReportConfiguration getReportConfiguration(QueryConfiguration *queryConfiguration) = 0;

	/// \brief Send a Report Subsystems List message to the component that sent the query.
	/// Send a Report Subsystems List message to the component that sent the query.
	/// \param[in] querySubsystemList - Input Trigger.
	/// \return ReportSubsystemList Output Message.
	virtual ReportSubsystemList getReportSubsystemList(QuerySubsystemList *querySubsystemList) = 0;

	/// \brief Send a Report Services message to the component that sent the query.
	/// Send a Report Services message to the component that sent the query.
	/// \param[in] queryServices - Input Trigger.
	/// \return ReportServices Output Message.
	virtual ReportServices getReportServices(QueryServices *queryServices) = 0;

	/// \brief Send a Report Service List message to the component that sent the query
	/// Send a Report Service List message to the component that sent the query
	/// \param[in] queryServiceList - Input Trigger.
	/// \return ReportServiceList Output Message.
	virtual ReportServiceList getReportServiceList(QueryServiceList *queryServiceList) = 0;

	/// \brief Send a Register Services message to newly identified component.
	/// Send a Register Services message to newly identified component.
	/// \param[in] reportIdentification - Input Trigger.
	/// \return RegisterServices Output Message.
	virtual RegisterServices getRegisterServices(ReportIdentification *reportIdentification) = 0;

	/// \brief Send a Query Service List message to newly identified components.
	/// Send a Query Service List message to newly identified components.
	/// \param[in] reportConfiguration - Input Trigger.
	/// \return QueryServiceList Output Message.
	virtual QueryServiceList getQueryServiceList(ReportConfiguration *reportConfiguration) = 0;

};

} // namespace core
} // namespace openjaus

#endif // DISCOVERY_SERVICE_INTERFACE_H
