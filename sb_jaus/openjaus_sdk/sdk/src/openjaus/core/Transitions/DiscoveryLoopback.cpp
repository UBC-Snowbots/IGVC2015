/**
\file DiscoveryLoopback.h

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

#include <openjaus.h>
#include "openjaus/core/Transitions/DiscoveryLoopback.h"
#include "openjaus/core/Triggers/QueryConfiguration.h"
#include "openjaus/core/Triggers/QueryIdentification.h"
#include "openjaus/core/Triggers/QueryServices.h"
#include "openjaus/core/Triggers/QuerySubsystemList.h"
#include "openjaus/core/Triggers/ReportServices.h"
	
namespace openjaus
{
namespace core
{

DiscoveryLoopback::DiscoveryLoopback()
{
	setName("DiscoveryLoopback");
	setType(model::LOOPBACK);
}

DiscoveryLoopback::~DiscoveryLoopback()
{
}

bool DiscoveryLoopback::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case RegisterServices::ID:
		{
			RegisterServices registerServices(message);
			
			cmptInterface->publishServices(&registerServices);			
			return true;
			break;
		}
			
		case QueryIdentification::ID:
		{
			QueryIdentification queryIdentification(message);
			
			ReportIdentification *reportIdentification = new ReportIdentification();
			*reportIdentification = cmptInterface->getReportIdentification(&queryIdentification);
			reportIdentification->setDestination(message->getSource());
			transport->sendMessage(reportIdentification);			
			return true;
			break;
		}
			
		case QueryConfiguration::ID:
		{
			QueryConfiguration queryConfiguration(message);
			
			ReportConfiguration *reportConfiguration = new ReportConfiguration();
			*reportConfiguration = cmptInterface->getReportConfiguration(&queryConfiguration);
			reportConfiguration->setDestination(message->getSource());
			transport->sendMessage(reportConfiguration);			
			return true;
			break;
		}
			
		case QuerySubsystemList::ID:
		{
			QuerySubsystemList querySubsystemList(message);
			
			ReportSubsystemList *reportSubsystemList = new ReportSubsystemList();
			*reportSubsystemList = cmptInterface->getReportSubsystemList(&querySubsystemList);
			reportSubsystemList->setDestination(message->getSource());
			transport->sendMessage(reportSubsystemList);			
			return true;
			break;
		}
			
		case QueryServices::ID:
		{
			QueryServices queryServices(message);
			
			ReportServices *reportServices = new ReportServices();
			*reportServices = cmptInterface->getReportServices(&queryServices);
			reportServices->setDestination(message->getSource());
			transport->sendMessage(reportServices);			
			return true;
			break;
		}
			
		case QueryServiceList::ID:
		{
			QueryServiceList queryServiceList(message);
			
			ReportServiceList *reportServiceList = new ReportServiceList();
			*reportServiceList = cmptInterface->getReportServiceList(&queryServiceList);
			reportServiceList->setDestination(message->getSource());
			transport->sendMessage(reportServiceList);			
			return true;
			break;
		}
			
		case ReportIdentification::ID:
		{
			ReportIdentification reportIdentification(message);
			
			cmptInterface->storeIdentification(&reportIdentification);
			RegisterServices *registerServices = new RegisterServices();
			*registerServices = cmptInterface->getRegisterServices(&reportIdentification);
			registerServices->setDestination(message->getSource());
			transport->sendMessage(registerServices);
			cmptInterface->queryConfiguration(&reportIdentification);			
			return true;
			break;
		}
			
		case ReportConfiguration::ID:
		{
			ReportConfiguration reportConfiguration(message);
			
			cmptInterface->storeConfiguration(&reportConfiguration);
			QueryServiceList *queryServiceList = new QueryServiceList();
			*queryServiceList = cmptInterface->getQueryServiceList(&reportConfiguration);
			queryServiceList->setDestination(message->getSource());
			transport->sendMessage(queryServiceList);			
			return true;
			break;
		}
			
		case ReportServiceList::ID:
		{
			ReportServiceList reportServiceList(message);
			
			cmptInterface->storeServiceList(&reportServiceList);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* DiscoveryLoopback::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case RegisterServices::ID:
			break;
			
		case QueryIdentification::ID:
		{	
			QueryIdentification queryIdentification(message);
			ReportIdentification *reportIdentification = new ReportIdentification();
			*reportIdentification = cmptInterface->getReportIdentification(&queryIdentification);
			return reportIdentification;
		}
			
		case QueryConfiguration::ID:
		{	
			QueryConfiguration queryConfiguration(message);
			ReportConfiguration *reportConfiguration = new ReportConfiguration();
			*reportConfiguration = cmptInterface->getReportConfiguration(&queryConfiguration);
			return reportConfiguration;
		}
			
		case QuerySubsystemList::ID:
		{	
			QuerySubsystemList querySubsystemList(message);
			ReportSubsystemList *reportSubsystemList = new ReportSubsystemList();
			*reportSubsystemList = cmptInterface->getReportSubsystemList(&querySubsystemList);
			return reportSubsystemList;
		}
			
		case QueryServices::ID:
		{	
			QueryServices queryServices(message);
			ReportServices *reportServices = new ReportServices();
			*reportServices = cmptInterface->getReportServices(&queryServices);
			return reportServices;
		}
			
		case QueryServiceList::ID:
		{	
			QueryServiceList queryServiceList(message);
			ReportServiceList *reportServiceList = new ReportServiceList();
			*reportServiceList = cmptInterface->getReportServiceList(&queryServiceList);
			return reportServiceList;
		}
			
		case ReportIdentification::ID:
		{	
			ReportIdentification reportIdentification(message);
			RegisterServices *registerServices = new RegisterServices();
			*registerServices = cmptInterface->getRegisterServices(&reportIdentification);
			return registerServices;
		}
			
		case ReportConfiguration::ID:
		{	
			ReportConfiguration reportConfiguration(message);
			QueryServiceList *queryServiceList = new QueryServiceList();
			*queryServiceList = cmptInterface->getQueryServiceList(&reportConfiguration);
			return queryServiceList;
		}
			
		case ReportServiceList::ID:
			break;
			
	}
	
	return NULL;
}


bool DiscoveryLoopback::setInterface(DiscoveryInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool DiscoveryLoopback::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace core
} // namespace openjaus

