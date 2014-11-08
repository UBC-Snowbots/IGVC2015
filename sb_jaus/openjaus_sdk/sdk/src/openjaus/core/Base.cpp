/**
\file Base.cpp

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


#include "openjaus/core/Base.h"
// Start of user code for additional headers:
#include "openjaus/core/Triggers/Fields/ConfirmResponseCodeEnumeration.h"
#include "openjaus/transport/AddressMap.h"
#include <ctime>
// End of user code

namespace openjaus
{
namespace core
{

Base::Base() : Events(),
	discoveryLoopback(),
	heartbeatLoop(),
	defaultConfigurationLoop(),
	notControlledLoopback(),
	acceptControlTransition(),
	controlledLoopback(),
	releaseControlTransition(),
	defaultStateLoop(),
	notControlled(),
	controlled(),
	accessStateMachine(),
	timeDefaultLoop(),
	timeControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "Base";
	
	model::Service *discoveryService = new model::Service();
	discoveryService->setName("Discovery");
	discoveryService->setUri("urn:jaus:jss:core:Discovery");
	discoveryService->setVersionMajor(1);
	discoveryService->setVersionMinor(0);
	this->implements->push_back(discoveryService);
	
	model::Service *livenessService = new model::Service();
	livenessService->setName("Liveness");
	livenessService->setUri("urn:jaus:jss:core:Liveness");
	livenessService->setVersionMajor(1);
	livenessService->setVersionMinor(0);
	this->implements->push_back(livenessService);
	
	model::Service *configurationService = new model::Service();
	configurationService->setName("Configuration");
	configurationService->setUri("urn:openjaus:core:Configuration");
	configurationService->setVersionMajor(1);
	configurationService->setVersionMinor(0);
	this->implements->push_back(configurationService);
	
	model::Service *accessControlService = new model::Service();
	accessControlService->setName("AccessControl");
	accessControlService->setUri("urn:jaus:jss:core:AccessControl");
	accessControlService->setVersionMajor(1);
	accessControlService->setVersionMinor(1);
	this->implements->push_back(accessControlService);
	
	model::Service *timeService = new model::Service();
	timeService->setName("Time");
	timeService->setUri("urn:jaus:jss:core:Time");
	timeService->setVersionMajor(1);
	timeService->setVersionMinor(1);
	this->implements->push_back(timeService);
	
	
	
	
	notControlled.setName("NotControlled");
	controlled.setName("Controlled");
	

	discoveryLoopback.setInterface(this);
	discoveryLoopback.setTransportInterface(this);
	receive.addDefaultStateTransition(discoveryLoopback);
	
	heartbeatLoop.setInterface(this);
	heartbeatLoop.setTransportInterface(this);
	receive.addDefaultStateTransition(heartbeatLoop);
	
	defaultConfigurationLoop.setInterface(this);
	defaultConfigurationLoop.setTransportInterface(this);
	receive.addDefaultStateTransition(defaultConfigurationLoop);
	
	notControlledLoopback.setInterface(this);
	notControlledLoopback.setTransportInterface(this);
	notControlledLoopback.setEndState(&notControlled);
	notControlledLoopback.setStartState(&notControlled);
	notControlled.addTransition(notControlledLoopback);
	
	acceptControlTransition.setInterface(this);
	acceptControlTransition.setTransportInterface(this);
	acceptControlTransition.setEndState(&controlled);
	acceptControlTransition.setStartState(&notControlled);
	notControlled.addTransition(acceptControlTransition);
	
	controlledLoopback.setInterface(this);
	controlledLoopback.setTransportInterface(this);
	controlledLoopback.setEndState(&controlled);
	controlledLoopback.setStartState(&controlled);
	controlled.addTransition(controlledLoopback);
	
	releaseControlTransition.setInterface(this);
	releaseControlTransition.setTransportInterface(this);
	releaseControlTransition.setEndState(&notControlled);
	releaseControlTransition.setStartState(&controlled);
	controlled.addTransition(releaseControlTransition);
	
	defaultStateLoop.setInterface(this);
	defaultStateLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(defaultStateLoop);
	
	timeDefaultLoop.setInterface(this);
	timeDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(timeDefaultLoop);
	
	timeControlledLoop.setInterface(this);
	timeControlledLoop.setTransportInterface(this);
	controlled.addTransition(timeControlledLoop);
	
	accessStateMachine.setName("AccessStateMachine");
	accessStateMachine.addState(notControlled);
	accessStateMachine.addState(controlled);
	accessStateMachine.setStartingState(&notControlled);
	receivingState.addStateMachine(accessStateMachine);
	
    
	notControlled.addEntryAction(&Base::init, this);
	controlled.addExitAction(&Base::sendRejectControlReleased, this);
    
	// Start of user code for Constructor:
	hbSequenceNumber = 0;

	// TODO: Add callback for rejectControl
	timer = NULL;

	udpInterface->setRecvQueue(transport::CONFIGURATION, const_cast<system::PriorityQueue*>(&receiveThread.getQueue()));

	defaultAuthority = 1;

	LOG_DEBUG("Base Component Construction Complete. Waiting for Run...");
	// End of user code
}

Base::~Base()
{
	// Start of user code for Destructor:
	if(timer)
	{
		delete timer;
	}

	model::Component::stop();
	// End of user code
}

bool Base::publishServices(RegisterServices *registerServices)
{
	// Start of user code for action publishServices(RegisterServices *registerServices):
	LOG(name << " processing register services message from " << registerServices->getSource());
	//LOG_DEBUG("\n" << registerServices->toXml());

	if(!this->systemTree->hasSubsystem(registerServices->getSource()))
	{
		LOG("RegisterServices from unknown Subs, sending QueryId");

		// Register Services from someone I don't know... kick off the Subsystem Discovery Chain
		core::QueryIdentification *queryId = new core::QueryIdentification();
		queryId->setDestination(registerServices->getSource());
		queryId->setQueryType(core::SystemLevelEnumeration::SUBSYSTEM);
		sendMessage(queryId);
	}

	if(!this->systemTree->hasNode(registerServices->getSource()))
	{
		LOG("RegisterServices from unknown Node, sending QueryId");

		// Register Services from someone I don't know... kick off the Node Discovery Chain
		core::QueryIdentification *queryId = new core::QueryIdentification();
		queryId->setDestination(registerServices->getSource());
		queryId->setQueryType(core::SystemLevelEnumeration::NODE);
		sendMessage(queryId);
	}

	systemTree->lock(); // Lock system tree before possibly attempting to modify
	if(!this->systemTree->hasComponent(registerServices->getSource()))
	{
		this->systemTree->addComponent(registerServices->getSource());

		LOG("RegisterServices from unknown Cmpt, sending QueryId");

		// Register Services from someone I don't know... kick off the Component Discovery Chain
		core::QueryIdentification *queryId = new core::QueryIdentification();
		queryId->setDestination(registerServices->getSource());
		queryId->setQueryType(core::SystemLevelEnumeration::COMPONENT);
		sendMessage(queryId);
	}

	// Add services
	core::ServiceList &list = registerServices->getRSServiceList();
	for(size_t i = 0; i < list.size(); i++)
	{
		core::ServiceInformationRecord &serviceRec = list.get(i);
		this->systemTree->addService(registerServices->getSource(), serviceRec.getUri(), serviceRec.getMajorVersionNumber(), serviceRec.getMinorVersionNumber());
		LOG_DEBUG("Adding service: " << serviceRec.getUri() << ", to: " << registerServices->getSource());
	}

	LOG_DEBUG(systemTree->toString());
	systemTree->unlock();
	return false;
	// End of user code
}

bool Base::storeIdentification(ReportIdentification *reportIdentification)
{
	// Start of user code for action storeIdentification(ReportIdentification *reportIdentification):
	LOG(name << " processing report identification from " << reportIdentification->getSource());
	LOG_DEBUG("\n" << reportIdentification->toXml());

	transport::Address source = reportIdentification->getSource();

	systemTree->lock();

	switch(reportIdentification->getQueryType())
	{
		case core::SystemLevelEnumeration::SUBSYSTEM:
		{
			if(!systemTree->hasSubsystem(source))
			{
				LOG("Adding subsystem from source: " << source << " to system tree");
				systemTree->addSubsystem(source);
				LOG(systemTree->toString());
			}
			systemTree->setSubsystemIdentification(reportIdentification->getSource(), reportIdentification->getIdentification());
		}
		break;

		case core::SystemLevelEnumeration::NODE:
		{
			if(!systemTree->hasNode(source))
			{
				LOG("Adding node from source: " << source << " to system tree");
				systemTree->addNode(source);
				LOG(systemTree->toString())
			}

			systemTree->setNodeIdentification(reportIdentification->getSource(), reportIdentification->getIdentification());
		}
		break;

		case core::SystemLevelEnumeration::COMPONENT:
		{
			if(!systemTree->hasComponent(source))
			{
				LOG("Adding component from source: " << source << " to system tree");
				systemTree->addComponent(source);
				LOG(systemTree->toString());
			}

			systemTree->setComponentIdentification(reportIdentification->getSource(), reportIdentification->getIdentification());
		}
		break;

		default:
			THROW_EXCEPTION("Received ReportIdentification with bad query type: " << reportIdentification->toString());
			break;
	}

	LOG_DEBUG(systemTree->toString());
	systemTree->unlock();
	return true;
	// End of user code
}

bool Base::storeServiceList(ReportServiceList *reportServiceList)
{
	// Start of user code for action storeServiceList(ReportServiceList *reportServiceList):
	LOG("storeServiceList from " << reportServiceList->getSource());
//	LOG_DEBUG("\n" << reportServiceList->toXml());

	systemTree->lock();

	core::RSLSubsystemList& subsList = reportServiceList->getRSLSubsystemList();
	for(size_t i = 0; i < subsList.size(); i++)
	{
		core::RSLSubsystemRecord &subsRec = subsList.get(i);
		if(!this->systemTree->hasSubsystem(subsRec.getSubsystemID()))
		{
			this->systemTree->addSubsystem(subsRec.getSubsystemID());
		}

		core::ServicesNodeList& nodeList = subsRec.getRSLNodeList();
		for(size_t j = 0; j < nodeList.size(); j++)
		{
			core::ServicesNodeRecord &nodeRec = nodeList.get(j);
			if(!this->systemTree->hasNode(subsRec.getSubsystemID(), nodeRec.getNodeID()))
			{
				this->systemTree->addNode(subsRec.getSubsystemID(), nodeRec.getNodeID());
			}

			core::ServicesComponentList &cmptList = nodeRec.getServicesComponentList();
			for(size_t k = 0; k < cmptList.size(); k++)
			{
				core::ServicesComponentRecord &cmptRec = cmptList.get(k);

				if(this->systemTree->hasComponent(subsRec.getSubsystemID(), nodeRec.getNodeID(), cmptRec.getComponentID()))
				{
					// Returns a copy of the cmpt from the system tree
					model::Component cmpt(this->systemTree->getComponent(subsRec.getSubsystemID(), nodeRec.getNodeID(), cmptRec.getComponentID()));

					// TODO: If we sent a QueryServiceList with a search filter, this would erase all the services and only replace the matching one... bad!

					// Empty current services
					cmpt.clearServices();

					core::ServicesServiceList &serviceList = cmptRec.getServicesServiceList();

					for(size_t l = 0; l < serviceList.size(); l++)
					{
						core::ServiceInformationRecord &serviceRec = serviceList.get(l);
						model::Service *service = new model::Service();

						service->setName(serviceRec.getUri());
						service->setUri(serviceRec.getUri());
						service->setVersionMajor(serviceRec.getMajorVersionNumber());
						service->setVersionMinor(serviceRec.getMinorVersionNumber());

						cmpt.addService(service);
					}

					// This replaces the existing tree cmpt with this one
					this->systemTree->replaceComponent(cmpt);
				}
				else
				{
					model::Component cmpt;
					cmpt.setAddress(transport::Address(subsRec.getSubsystemID(), nodeRec.getNodeID(), cmptRec.getComponentID()));
					cmpt.setId(cmptRec.getComponentID());

					core::ServicesServiceList &serviceList = cmptRec.getServicesServiceList();
					for(size_t l = 0; l < serviceList.size(); l++)
					{
						core::ServiceInformationRecord &serviceRec = serviceList.get(l);
						model::Service *service = new model::Service();

						service->setName(serviceRec.getUri());
						service->setUri(serviceRec.getUri());
						service->setVersionMajor(serviceRec.getMajorVersionNumber());
						service->setVersionMinor(serviceRec.getMinorVersionNumber());

						cmpt.addService(service);
					}

					// This adds a cmpt to the tree along with identification & service list
					this->systemTree->addComponent(cmpt);
				}
			}
		}
	}
	systemTree->unlock();

	return true;
	// End of user code
}

bool Base::storeConfiguration(ReportConfiguration *reportConfiguration)
{
	// Start of user code for action storeConfiguration(ReportConfiguration *reportConfiguration):
	LOG("storeConfiguration from " << reportConfiguration->getSource());
//	LOG_DEBUG("\n" << reportConfiguration->toXml());

	uint16 subsId = reportConfiguration->getSource().getSubsystem();

	if(!this->systemTree->hasSubsystem(subsId))
	{
		THROW_EXCEPTION("Report Configuration from unknown Subsystem");
	}

	systemTree->lock();
	core::ConfigurationNodeList& nodeList = reportConfiguration->getNodeList();
	for(size_t i = 0; i < nodeList.size(); i++)
	{
		core::ConfigurationNodeRecord& nodeRec = nodeList.get(i);
		if(!this->systemTree->hasNode(subsId, nodeRec.getNodeID()))
		{
			this->systemTree->addNode(subsId, nodeRec.getNodeID());
		}

		core::ConfigurationComponentList& cmptList = nodeRec.getConfigurationComponentList();
		for(size_t j = 0; j < cmptList.size(); j++)
		{
			core::ConfigurationComponentRecord& cmptRec = cmptList.get(j);
			if(!this->systemTree->hasComponent(subsId, nodeRec.getNodeID(), cmptRec.getComponentID()))
			{
				this->systemTree->addComponent(subsId, nodeRec.getNodeID(), cmptRec.getComponentID());
			}
		}
	}
	systemTree->unlock();
	return true;
	// End of user code
}

bool Base::queryConfiguration(ReportIdentification *reportIdentification)
{
	// Start of user code for action queryConfiguration(ReportIdentification *reportIdentification):
	LOG("getQueryConfiguration from " << reportIdentification->getSource());
//	LOG_DEBUG("\n" << reportIdentification->toXml());

	switch(reportIdentification->getQueryType())
	{
		case core::SystemLevelEnumeration::SUBSYSTEM:
		case core::SystemLevelEnumeration::NODE:
		{
			// Create query message
			core::QueryConfiguration *query = new core::QueryConfiguration();
			query->setDestination(reportIdentification->getSource());
			query->setQueryType(reportIdentification->getQueryType());
			sendMessage(query);
			break;
		}

		case core::SystemLevelEnumeration::COMPONENT:
		{
			// Components do not have a configuration
			break;
		}

		default:
			THROW_EXCEPTION("Received ReportIdentification with bad query type: " << reportIdentification->getQueryType());
			break;
	}

	return true;
	// End of user code
}

core::ReportIdentification Base::getReportIdentification(QueryIdentification *queryIdentification)
{
	// Start of user code for action getReportIdentification(QueryIdentification *queryIdentification):
	LOG(name << " responding to query id from " << queryIdentification->getSource());
//	LOG_DEBUG("\n" << queryIdentification->toXml());

	core::ReportIdentification reportId;
	reportId.setDestination(queryIdentification->getSource());
	reportId.setQueryType(queryIdentification->getQueryType());

	switch(queryIdentification->getQueryType())
	{
		case core::SystemLevelEnumeration::SUBSYSTEM:
			reportId.setIdentification(system::Application::setting<std::string>("SubsystemName", "OpenJAUS_Subsystem"));
			system::Application::comment("SubsystemName", "This defines the Subsystem Name for this JAUS Subsystem. Options: Any string");
			break;

		case core::SystemLevelEnumeration::NODE:
			reportId.setIdentification(system::Application::setting<std::string>("NodeName", "OpenJAUS_Node"));
			system::Application::comment("NodeName", "This defines the Node Name for this JAUS Node. Options: Any string");
			break;

		case core::SystemLevelEnumeration::COMPONENT:
			reportId.setIdentification(system::Application::setting<std::string>(name + ".PublishedName", name));
			system::Application::comment(name + ".PublishedName", "This defines the Component Name for this JAUS Component. Options: Any string");
			break;

		default:
			THROW_EXCEPTION("Received a query identification with an unsupported Query Type: " << queryIdentification->toString());
			break;
	}

	return reportId;
	// End of user code
}

core::ReportConfiguration Base::getReportConfiguration(QueryConfiguration *queryConfiguration)
{
	// Start of user code for action getReportConfiguration(QueryConfiguration *queryConfiguration):
	LOG("getReportConfiguration from " << queryConfiguration->getSource());
//	LOG_DEBUG("\n" << queryConfiguration->toXml());

	core::ReportConfiguration message;

	switch(queryConfiguration->getQueryType())
	{
		case core::SystemLevelEnumeration::SUBSYSTEM:
		{
			model::Subsystem subsystem(this->systemTree->getSubsystem(this->address));

			const std::map< unsigned char, model::Node * >& nodeMap = subsystem.getNodes();
			std::map< unsigned char, model::Node * >::const_iterator nodeIter;
			for(nodeIter = nodeMap.begin(); nodeIter != nodeMap.end(); nodeIter++)
			{
				model::Node *node = nodeIter->second;

				core::ConfigurationNodeRecord nodeRec;
				nodeRec.setNodeID(node->getId());

				const std::map< unsigned char, model::Component * >& cmptMap = node->getComponents();
				std::map< unsigned char, model::Component * >::const_iterator iter;
				for(iter = cmptMap.begin(); iter != cmptMap.end(); iter++)
				{
					model::Component *cmpt = iter->second;

					core::ConfigurationComponentRecord cmptRec;
					cmptRec.setComponentID(cmpt->getId());
					cmptRec.setInstanceID(0);

					nodeRec.getConfigurationComponentList().add(cmptRec);
				}

				message.getNodeList().add(nodeRec);
			}
			break;
		}

		case core::SystemLevelEnumeration::NODE:
		{
			model::Node node(this->systemTree->getNode(this->address));

			core::ConfigurationNodeRecord nodeRec;
			nodeRec.setNodeID(node.getId());

			const std::map< unsigned char, model::Component * >& cmptMap = node.getComponents();
			std::map< unsigned char, model::Component * >::const_iterator iter;
			for(iter = cmptMap.begin(); iter != cmptMap.end(); iter++)
			{
				model::Component *cmpt = iter->second;

				core::ConfigurationComponentRecord cmptRec;
				cmptRec.setComponentID(cmpt->getId());
				cmptRec.setInstanceID(0);

				nodeRec.getConfigurationComponentList().add(cmptRec);
			}

			message.getNodeList().add(nodeRec);
			break;

		}

		default:
			THROW_EXCEPTION("Unknown QueryConfiguration Type recv'd");
			break;

	}

	return message;
	// End of user code
}

core::ReportSubsystemList Base::getReportSubsystemList(QuerySubsystemList *querySubsystemList)
{
	// Start of user code for action getReportSubsystemList(QuerySubsystemList *querySubsystemList):
	core::ReportSubsystemList message;
	return message;
	// End of user code
}

core::ReportServices Base::getReportServices(QueryServices *queryServices)
{
	// Start of user code for action getReportServices(QueryServices *queryServices):
	LOG("getReportServices from " << queryServices->getSource());
//	LOG_DEBUG("\n" << queryServices->toXml());

	// Note: This is deprecated in 5710A
	core::ReportServices message;

	model::Subsystem subsystem(this->systemTree->getSubsystem(this->address));

	// Note: I don't care what your query was, I'm sending you everything
	core::ServicesNodeList &nodeList = message.getRSNodeList();

	const std::map< unsigned char, model::Node * >& nodeMap = subsystem.getNodes();
	std::map< unsigned char, model::Node * >::const_iterator nodeIter;
	for(nodeIter = nodeMap.begin(); nodeIter != nodeMap.end(); nodeIter++)
	{
		model::Node *node = nodeIter->second;

		core::ServicesNodeRecord nodeRec;
		nodeRec.setNodeID(node->getId());

		core::ServicesComponentList &cmptList = nodeRec.getServicesComponentList();

		const std::map< unsigned char, model::Component * >& cmptMap = node->getComponents();
		std::map< unsigned char, model::Component * >::const_iterator cmptIter;
		for(cmptIter = cmptMap.begin(); cmptIter != cmptMap.end(); cmptIter++)
		{
			model::Component *cmpt = cmptIter->second;

			core::ServicesComponentRecord cmptRec;
			cmptRec.setComponentID(cmpt->getId());
			cmptRec.setInstanceID(0);

			core::ServicesServiceList serviceList = cmptRec.getServicesServiceList();

			const std::map< std::string, model::Service * >& serviceMap = cmpt->getServices();
			std::map< std::string, model::Service * >::const_iterator iter;
			for(iter = serviceMap.begin(); iter != serviceMap.end(); ++iter)
			{
				model::Service *service = iter->second;

				core::ServiceInformationRecord serviceRec;
				serviceRec.setUri(service->getUri());
				serviceRec.setMajorVersionNumber(service->getVersionMajor());
				serviceRec.setMinorVersionNumber(service->getVersionMinor());

				serviceList.add(serviceRec);
			}
			cmptList.add(cmptRec);
		}
		nodeList.add(nodeRec);
	}

	return message;
	// End of user code
}

core::ReportServiceList Base::getReportServiceList(QueryServiceList *queryServiceList)
{
	// Start of user code for action getReportServiceList(QueryServiceList *queryServiceList):
	LOG("getReportServiceList from " << queryServiceList->getSource());
//	LOG_DEBUG("\n" << queryServiceList->toXml());

	core::ReportServiceList message;

	core::QSLSubsystemList& querySubsList = queryServiceList->getQSLSubsystemList();
	for(size_t i = 0; i < querySubsList.size(); i++)
	{
		core::QSLSubsystemRecord& querySubsRec = querySubsList.get(i);
		if(querySubsRec.getSubsystemID() == 65535) // Magic Number (65535): All subsystems in the system
		{
			// Add all subs to report

			// List of all Subystems
			std::vector<uint16> subsystemIds = this->systemTree->getSubsystemIds();

			// For every Subs
			for(size_t j = 0; j < subsystemIds.size(); j++)
			{
				querySubsRec.setSubsystemID(subsystemIds.at(j)); // This is not a bug
				message.getRSLSubsystemList().add(reportServiceListGetSubsystem(querySubsRec));
			}
		}
		else
		{
			if(this->systemTree->hasSubsystem(querySubsRec.getSubsystemID()))
			{
				// Add specified subs to report
				message.getRSLSubsystemList().add(reportServiceListGetSubsystem(querySubsRec));
			}
		}
	}

	return message;
	// End of user code
}

core::RegisterServices Base::getRegisterServices(ReportIdentification *reportIdentification)
{
	// Start of user code for action getRegisterServices(ReportIdentification *reportIdentification):
	LOG("getRegisterServices from " << reportIdentification->getSource());
//	LOG_DEBUG("\n" << reportIdentification->toXml());

	core::RegisterServices message;

	core::ServiceList &list = message.getRSServiceList();

	for(size_t i = 0; i < this->implements->size(); i++)
	{
		core::ServiceInformationRecord serviceRec;
		model::Service *service = this->implements->at(i);
		serviceRec.setUri(service->getUri());
		serviceRec.setMajorVersionNumber(service->getVersionMajor());
		serviceRec.setMinorVersionNumber(service->getVersionMinor());
		list.add(serviceRec);
	}

	LOG_DEBUG("\nSending getRegisterServices message: " << message.toXml());

	return message;
	// End of user code
}

core::QueryServiceList Base::getQueryServiceList(ReportConfiguration *reportConfiguration)
{
	// Start of user code for action getQueryServiceList(ReportConfiguration *reportConfiguration):
	LOG("getQueryServiceList from " << reportConfiguration->getSource());
//	LOG_DEBUG("\n" << reportConfiguration->toXml());

	core::QueryServiceList message;

	core::QSLSubsystemRecord subsRec;
	subsRec.setSubsystemID(65535); // All Subsystems

	core::QSLNodeRecord nodeRec;
	nodeRec.setNodeID(255); // All Nodes

	core::QSLComponentRecord cmptRec;
	cmptRec.setComponentID(255); // All Components

	nodeRec.getQSLComponentList().add(cmptRec);
	subsRec.getQSLNodeList().add(nodeRec);
	message.getQSLSubsystemList().add(subsRec);

	return message;
	// End of user code
}

core::ReportHeartbeatPulse Base::getReportHeartbeatPulse(QueryHeartbeatPulse *queryHeartbeatPulse)
{
	// Start of user code for action getReportHeartbeatPulse(QueryHeartbeatPulse *queryHeartbeatPulse):
		LOG_DEBUG("Base Component: Got QueryHeartbeatPulse from: " << queryHeartbeatPulse->getSource());
		core::ReportHeartbeatPulse message;
		message.setSequenceNumber(queryHeartbeatPulse->getSequenceNumber());
		//message.setMustArrive(true);
		return message;
	// End of user code
}

bool Base::processHeartbeat(ReportHeartbeatPulse *reportHeartbeatPulse)
{
	// Start of user code for action processHeartbeat(ReportHeartbeatPulse *reportHeartbeatPulse):
	LOG_DEBUG("Recv Report HB Pulse from: " << reportHeartbeatPulse->getSource());

	if( reportHeartbeatPulse->getSource().getNode() == 1 &&
		!address.isValid() &&
		system::Application::setting<std::string>("StaticSubsystemID", "1") == "Node1Heartbeat")
	{
		address.setSubsystem(reportHeartbeatPulse->getSource().getSubsystem());
		LOG_DEBUG("Assuming Node 1's Subsystem ID: " << address.getSubsystem());
		if(address.isValid())
		{
			addSelfToSystemTree();
			sendQueryIdentifications(0);
		}
	}

	//TODO: Make this algorithm a little smarter
	if(	!systemTree->hasComponent(reportHeartbeatPulse->getSource()) ||
		!systemTree->hasComponentIdentification(reportHeartbeatPulse->getSource()))
	{
		LOG_DEBUG("Send Query Component Identification to: " << reportHeartbeatPulse->getSource());
		core::QueryIdentification *queryId = new core::QueryIdentification();
		queryId->setQueryType(core::SystemLevelEnumeration::COMPONENT);
		queryId->setDestination(reportHeartbeatPulse->getSource());
		//LOG_DEBUG(queryId->toXml());
		sendMessage(queryId);
	}

	if(	!systemTree->hasNode(reportHeartbeatPulse->getSource()) ||
		!systemTree->hasNodeIdentification(reportHeartbeatPulse->getSource()))
	{
		LOG_DEBUG("Send Query Node Identification to: " << reportHeartbeatPulse->getSource());
		core::QueryIdentification *queryId = new core::QueryIdentification();
		queryId->setQueryType(core::SystemLevelEnumeration::NODE);
		queryId->setDestination(reportHeartbeatPulse->getSource());
		//LOG_DEBUG(queryId->toXml());
		sendMessage(queryId);
	}

	if(	!systemTree->hasSubsystem(reportHeartbeatPulse->getSource()) ||
		!systemTree->hasSubsystemIdentification(reportHeartbeatPulse->getSource()))
	{
		LOG_DEBUG("Send Query Subsystem Identification to: " << reportHeartbeatPulse->getSource());
		core::QueryIdentification *queryId = new core::QueryIdentification();
		queryId->setQueryType(core::SystemLevelEnumeration::SUBSYSTEM);
		queryId->setDestination(reportHeartbeatPulse->getSource());
		//LOG_DEBUG(queryId->toXml());
		sendMessage(queryId);
	}

	return true;
	// End of user code
}

bool Base::sendJausAddress(QueryJausAddress *queryJausAddress)
{
	// Start of user code for action sendJausAddress(QueryJausAddress *queryJausAddress):
	if(!address.isValid())
	{
		return false;
	}

	core::ReportJausAddress *reportAddress = new core::ReportJausAddress();
	reportAddress->setDestination(queryJausAddress->getSource());
	core::AddressBitField& addressField = reportAddress->getAddress();
	addressField.setSubsystem(address.getSubsystem());
	addressField.setNode(address.getNode());
	addressField.setComponent(address.getComponent());

	transport::AS5669::JudpAddress *udpData = new transport::AS5669::JudpAddress();
	*udpData = *dynamic_cast<transport::AS5669::JudpAddress *>(queryJausAddress->getTransportData());
	reportAddress->setTransportData(udpData);

	LOG("Got Query Address, sending:\n" << reportAddress->toXml());

	sendMessage(reportAddress);
	return true;
	// End of user code
}

bool Base::setThisJausAddress(ReportJausAddress *reportJausAddress)
{
	// Start of user code for action setThisJausAddress(ReportJausAddress *reportJausAddress):
	core::AddressBitField& addressField = reportJausAddress->getAddress();

	LOG("Got Report Address:\n" << reportJausAddress->toXml());

	if(!address.getNode())
	{
		address.setSubsystem(addressField.getSubsystem());
	}

	if(!address.getComponent())
	{
		address.setNode(addressField.getNode());
	}

	transport::Address jausAddress(	addressField.getSubsystem(), addressField.getNode(), addressField.getComponent());

	transport::AS5669::JudpAddress *udpAddress = dynamic_cast<transport::AS5669::JudpAddress *>(reportJausAddress->getTransportData());
	if(!udpAddress)
	{
		THROW_EXCEPTION("Base Component: Could not retrieve transport data from reportJausAddress message");
	}
	LOG_DEBUG("\n\nBase Component: Setting Address Map: " << jausAddress << " -> " << udpAddress->toString()	);
	transport::AddressMap::setTransportData(jausAddress, *udpAddress);

	systemTree->lock();
	if(address.getComponent())
	{
		systemTree->addComponent(addressField.getSubsystem(), addressField.getNode(), addressField.getComponent());
	}
	else
	{	// If we don't know our own component id yet, this must be comming from a local component
		transport::Address localAddress(addressField.getSubsystem(), addressField.getNode(), addressField.getComponent());
		systemTree->addLocalComponent(localAddress);
	}
	systemTree->unlock();

	return true;
	// End of user code
}

void Base::sendRejectControlReleased()
{
	// Start of user code for action sendRejectControlReleased():
	// End of user code
}

void Base::init()
{
	// Start of user code for action init():
	// End of user code
}

core::ReportControl Base::getReportControl(QueryControl *queryControl)
{
	// Start of user code for action getReportControl(QueryControl *queryControl):
	core::ReportControl message;
	message.setSubsystemID(controllerAddress.getSubsystem());
	message.setNodeID(controllerAddress.getNode());
	message.setComponentID(controllerAddress.getComponent());
	message.setAuthorityCode(controllerAuthority);
	return message;
	// End of user code
}

core::ReportAuthority Base::getReportAuthority(QueryAuthority *queryAuthority)
{
	// Start of user code for action getReportAuthority(QueryAuthority *queryAuthority):
	core::ReportAuthority message;
	return message;
	// End of user code
}

/// Send action for ReportTimeout with input message SendReportTimeout.
/// \param[in]  sendReportTimeout                 - <describe this>
/// \return     core::ReportTimeout - <describe this>
core::ReportTimeout Base::getReportTimeout(model::Trigger *trigger)
{
	// Start of user code for action getReportTimeout(model::Trigger *trigger):
	core::ReportTimeout message;
	return message;
	// End of user code
}

bool Base::setAuthority(RequestControl *requestControl)
{
	// Start of user code for action setAuthority(RequestControl *requestControl):
	controllerAuthority = requestControl->getAuthorityCode();
	return true;
	// End of user code
}

bool Base::setAuthority(SetAuthority *setAuthority)
{
	// Start of user code for action setAuthority(SetAuthority *setAuthority):
	controllerAuthority = setAuthority->getAuthorityCode();
	return true;
	// End of user code
}

bool Base::resetTimer(RequestControl *requestControl)
{
	// Start of user code for action resetTimer(RequestControl *requestControl):
	return false;
	// End of user code
}

bool Base::sendConfirmControlNotAvailable(RequestControl *requestControl)
{
	// Start of user code for action sendConfirmControlNotAvailable(RequestControl *requestControl):
	LOG("sending confirm control not available to " << requestControl->getSource());
	LOG_DEBUG("\n" << requestControl->toXml());

	core::ConfirmControl *message = new core::ConfirmControl();
	message->setDestination(requestControl->getSource());
	message->setConfirmResponseCode(core::ConfirmResponseCodeEnumeration::NOT_AVAILABLE);
	sendMessage(message);
	return true;
	// End of user code
}

bool Base::sendConfirmControlInsufficientAuthority(RequestControl *requestControl)
{
	// Start of user code for action sendConfirmControlInsufficientAuthority(RequestControl *requestControl):
	LOG("sending confirm control insufficient authority to " << requestControl->getSource());
	LOG_DEBUG("\n" << requestControl->toXml());

	core::ConfirmControl *message = new core::ConfirmControl();
	message->setDestination(requestControl->getSource());
	message->setConfirmResponseCode(core::ConfirmResponseCodeEnumeration::INSUFFICIENT_AUTHORITY);
	sendMessage(message);
	return true;
	// End of user code
}

bool Base::sendConfirmControlAccepted(RequestControl *requestControl)
{
	// Start of user code for action sendConfirmControlAccepted(RequestControl *requestControl):
	LOG("sending confirm control accepted to " << requestControl->getSource());
	LOG_DEBUG("\n" << requestControl->toXml());

	core::ConfirmControl *message = new core::ConfirmControl();
	message->setDestination(requestControl->getSource());
	message->setConfirmResponseCode(core::ConfirmResponseCodeEnumeration::CONTROL_ACCEPTED);
	sendMessage(message);
	return true;
	// End of user code
}

bool Base::sendRejectControlToController(RequestControl *requestControl)
{
	// Start of user code for action sendRejectControlToController(RequestControl *requestControl):
	LOG("sending reject control to controller: " << requestControl->getSource());
	core::RejectControl *message = new core::RejectControl();
	message->setDestination(controllerAddress);
	message->setRejectResponseCode(core::RejectResponseCodeEnumeration::CONTROL_RELEASED);
	sendMessage(message);
	return true;
	// End of user code
}

bool Base::sendRejectControlNotAvailable(ReleaseControl *releaseControl)
{
	// Start of user code for action sendRejectControlNotAvailable(ReleaseControl *releaseControl):
	core::RejectControl *message = new core::RejectControl();
	message->setDestination(releaseControl->getSource());
	message->setRejectResponseCode(core::RejectResponseCodeEnumeration::NOT_AVAILABLE);
	sendMessage(message);
	return true;
	// End of user code
}

bool Base::sendRejectControlReleased(ReleaseControl *releaseControl)
{
	// Start of user code for action sendRejectControlReleased(ReleaseControl *releaseControl):
	core::RejectControl *message = new core::RejectControl();
	message->setDestination(releaseControl->getSource());
	message->setRejectResponseCode(core::RejectResponseCodeEnumeration::CONTROL_RELEASED);
	sendMessage(message);
	return true;
	// End of user code
}

bool Base::storeAddress(RequestControl *requestControl)
{
	// Start of user code for action storeAddress(RequestControl *requestControl):
	LOG("Storing controller address " << requestControl->getSource());

	controllerAddress = requestControl->getSource();
	return true;
	// End of user code
}

bool Base::updateControlledList(ConfirmControl *confirmControl)
{
	// Start of user code for action updateControlledList(ConfirmControl *confirmControl):
	LOG("Updating Controlled List:");
	LOG_DEBUG(confirmControl->toXml());

	std::map<int, void (*)(const model::ControlResponse&) >::iterator i = controlResponseCallbacks.find(confirmControl->getSource().getHash());
	if(i != controlResponseCallbacks.end() && i->second)
	{
		model::ControlResponse response;
		response.setAddress(confirmControl->getSource());
		switch(confirmControl->getConfirmResponseCode())
		{
			case ConfirmResponseCodeEnumeration::CONTROL_ACCEPTED:
				response.setResponseType(model::CONTROL_ACCEPTED);
				break;
			case ConfirmResponseCodeEnumeration::INSUFFICIENT_AUTHORITY:
				response.setResponseType(model::INSUFFICIENT_AUTHORITY);
				break;
			case ConfirmResponseCodeEnumeration::NOT_AVAILABLE:
				response.setResponseType(model::NOT_AVAILABLE);
				break;
		}
		i->second(response);
	}

	switch(confirmControl->getConfirmResponseCode())
	{
		case core::ConfirmResponseCodeEnumeration::CONTROL_ACCEPTED:
			for(size_t i = 0; i < controlledComponents.size(); ++i)
			{
				if(confirmControl->getSource() == controlledComponents[i])
				{
					return false;
				}
			}

			controlledComponents.push_back(confirmControl->getSource());
			break;

		case core::ConfirmResponseCodeEnumeration::NOT_AVAILABLE:
			THROW_EXCEPTION("Confirm control: not available at address: " << confirmControl->getSource());
			break;

		case core::ConfirmResponseCodeEnumeration::INSUFFICIENT_AUTHORITY:
			THROW_EXCEPTION("Confirm control: insufficient authority to control: " << confirmControl->getSource());
			break;

		default:
			THROW_EXCEPTION("Unknown confirm control response code: " << confirmControl->getConfirmResponseCode() << ", from: " << confirmControl->getSource());
			break;
	}

	return true;
	// End of user code
}

bool Base::updateControlledList(RejectControl *rejectControl)
{
	// Start of user code for action updateControlledList(RejectControl *rejectControl):
	LOG("Updating Controlled List:");
	LOG_DEBUG(rejectControl->toXml());

	std::map<int, void (*)(const model::ControlResponse&) >::iterator i = controlResponseCallbacks.find(rejectControl->getSource().getHash());
	if(i != controlResponseCallbacks.end() && i->second)
	{
		model::ControlResponse response;
		response.setAddress(rejectControl->getSource());
		switch(rejectControl->getRejectResponseCode())
		{
			case RejectResponseCodeEnumeration::CONTROL_RELEASED:
				response.setResponseType(model::CONTROL_RELEASED);
				break;
			case RejectResponseCodeEnumeration::NOT_AVAILABLE:
				response.setResponseType(model::NOT_AVAILABLE);
				break;
		}
		i->second(response);
	}

	switch(rejectControl->getRejectResponseCode())
	{
		case core::RejectResponseCodeEnumeration::CONTROL_RELEASED:
		{
			std::vector<transport::Address>::iterator i;
			for(i = controlledComponents.begin(); i != controlledComponents.end(); ++i)
			{
				if(rejectControl->getSource() == *i)
				{
					LOG("Erasing component from controlled list: " << *i);
					controlledComponents.erase(i);
					return true;
				}
			}
			THROW_EXCEPTION("Received reject control from uncontrolled address: " << rejectControl->getSource());
			break;
		}

		case core::RejectResponseCodeEnumeration::NOT_AVAILABLE:
			THROW_EXCEPTION("Reject control: not available at address: " << rejectControl->getSource());

		default:
			break;
	}

	THROW_EXCEPTION("Unknown reject control response code: " << rejectControl->getRejectResponseCode() << ", from: " << rejectControl->getSource());
	// End of user code
}

bool Base::setTime(SetTime *setTime)
{
	// Start of user code for action setTime(SetTime *setTime):
	LOG("Recv'd SetTime message from " << setTime->getSource().toString() <<". SetTime Message is DEPRECATED and not supported by default in OpenJAUS.");
	return false;
	// End of user code
}

core::ReportTime Base::getReportTime(QueryTime *queryTime)
{
	// Start of user code for action getReportTime(QueryTime *queryTime):
	core::ReportTime message;

	// System time in seconds & microseconds
	system::Time ojTime = system::Time::getTime();

	// Convert to struct tm in UTC time
	time_t epochTime = ojTime.getSeconds();
	struct tm *utcTm;
	utcTm = gmtime(&epochTime);

	// Year
	// tm_year = years since 1900
	// JAUS year = years since 2000
	message.getDate().setYear(utcTm->tm_year - 100);

	// Month
	// tm_mon = months since Jan (0-11)
	// JAUS mon = 1-12
	message.getDate().setMonth(utcTm->tm_mon+1);

	// Date
	// tm_mday = day of the month (1-31)
	// JAUS day = day of the month (1-31)
	message.getDate().setDay(utcTm->tm_mday);
	message.getTime().setDay(utcTm->tm_mday);

	// Hour
	// tm_hour = hours since midnight (0-23)
	// JAUS Hour = hours since midnight (0-23)
	message.getTime().setHour(utcTm->tm_hour);

	// Minute
	// tm_min = minutes after the hour (0-59)
	// JAUS Minutes = minutes after the hour (0-59)
	message.getTime().setMinutes(utcTm->tm_min);

	// Seconds
	// tm_sec = seconds after the minute (0-59)
	// JAUS Seconds = seconds after the minute (0-59)
	message.getTime().setSeconds(utcTm->tm_sec);

	// Milliseconds
	// JAUS Milliseconds = seconds after the minute (0-999)
	message.getTime().setMilliseconds(ojTime.getMicroseconds() / 1000);

	return message;
	// End of user code
}


bool Base::isDefaultAuthorityGreater(RequestControl *requestControl)
{
	// Start of user code for action isDefaultAuthorityGreater(RequestControl *requestControl):
	LOG("Checking default authority: " << defaultAuthority << " > " << requestControl->getAuthorityCode() << "?");
	if(defaultAuthority > requestControl->getAuthorityCode())
	{
		LOG_DEBUG("Default Authority Greater");
		return true;
	}
	return false;
	// End of user code
}


bool Base::isCurrentAuthorityLess(RequestControl *requestControl)
{
	// Start of user code for action isCurrentAuthorityLess(RequestControl *requestControl):
	LOG_DEBUG("Current Authority: " << controllerAuthority);
	if(controllerAuthority < requestControl->getAuthorityCode())
	{
		return true;
	}
	return false;
	// End of user code
}


bool Base::isAuthorityValid(SetAuthority *setAuthority)
{
	// Start of user code for action isAuthorityValid(SetAuthority *setAuthority):
	if(setAuthority->getAuthorityCode() >= defaultAuthority)
	{
		return true;
	}
	return false;
	// End of user code
}


bool Base::isControllingClient(RequestControl *requestControl)
{
	// Start of user code for action isControllingClient(RequestControl *requestControl):
	LOG("Checking is controlling client");
	if(requestControl->getSource() == controllerAddress)
	{
		return true;
	}
	return false;
	// End of user code
}

bool Base::isControllingClient(SetAuthority *setAuthority)
{
	// Start of user code for action isControllingClient(SetAuthority *setAuthority):
	if(setAuthority->getSource() == controllerAddress)
	{
		return true;
	}
	return false;

	// End of user code
}

bool Base::isControllingClient(ReleaseControl *releaseControl)
{
	// Start of user code for action isControllingClient(ReleaseControl *releaseControl):
	if(releaseControl->getSource() == controllerAddress)
	{
		return true;
	}
	return false;
	// End of user code
}


bool Base::isControllingClient(SetTime *setTime)
{
	// Start of user code for action isControllingClient(SetTime *setTime):
	if(setTime->getSource() == controllerAddress)
	{
		return true;
	}
	return false;
	// End of user code
}


// Start of user code for additional methods
void Base::requestControl(transport::Address destination, void (*responseCallback)(const model::ControlResponse&))
{
	if(responseCallback)
	{
		controlResponseCallbacks[destination.getHash()] = responseCallback;
	}

	core::RequestControl *request = new core::RequestControl();
	request->setDestination(destination);
	request->setAuthorityCode(this->authority);
	sendMessage(request);
}

void Base::releaseControl(transport::Address destination, void (*responseCallback)(const model::ControlResponse&))
{
	if(responseCallback)
	{
		controlResponseCallbacks[destination.getHash()] = responseCallback;
	}

	std::vector<transport::Address>::iterator i;
	for(i = controlledComponents.begin(); i != controlledComponents.end(); ++i)
	{
		if(destination == *i)
		{
			core::ReleaseControl *release = new core::ReleaseControl();
			release->setDestination(destination);
			sendMessage(release);
			return;
		}
	}

	THROW_EXCEPTION("Attempted to release control of un-controlled component");
}

core::RSLSubsystemRecord Base::reportServiceListGetSubsystem(core::QSLSubsystemRecord& querySubsRec)
{
	if(!this->systemTree->hasSubsystem(querySubsRec.getSubsystemID()))
	{
		THROW_EXCEPTION("Undefined subsId queried");
	}

	core::RSLSubsystemRecord reportSubsRec;
	reportSubsRec.setSubsystemID(querySubsRec.getSubsystemID());

	core::QSLNodeList &queryNodeList = querySubsRec.getQSLNodeList();
	for(size_t i = 0; i < queryNodeList.size(); i++)
	{
		core::QSLNodeRecord& queryNodeRec = queryNodeList.get(i);
		if(queryNodeRec.getNodeID() == 255) // All Nodes
		{
			model::Subsystem subsystem(this->systemTree->getSubsystem(querySubsRec.getSubsystemID()));
			const std::map< unsigned char, model::Node * >& nodeMap = subsystem.getNodes();
			std::map< unsigned char, model::Node * >::const_iterator iter;
			for(iter = nodeMap.begin(); iter != nodeMap.end(); iter++)
			{
				queryNodeRec.setNodeID(iter->second->getId());
				reportSubsRec.getRSLNodeList().add(reportServiceListGetNode(querySubsRec.getSubsystemID(), queryNodeRec));
			}
		}
		else
		{
			if(this->systemTree->hasNode(querySubsRec.getSubsystemID(), queryNodeRec.getNodeID()))
			{
				reportSubsRec.getRSLNodeList().add(reportServiceListGetNode(querySubsRec.getSubsystemID(), queryNodeRec));
			}
		}
	}

	return reportSubsRec;
}

core::ServicesNodeRecord Base::reportServiceListGetNode(uint16 subsId, core::QSLNodeRecord& queryNodeRec)
{
	if(!this->systemTree->hasNode(subsId, queryNodeRec.getNodeID()))
	{
		THROW_EXCEPTION("Undefined node queried");
	}

	core::ServicesNodeRecord reportNodeRec;
	reportNodeRec.setNodeID(queryNodeRec.getNodeID());

	core::QSLComponentList &queryCmptList = queryNodeRec.getQSLComponentList();
	for(size_t i = 0; i < queryCmptList.size(); i++)
	{
		core::QSLComponentRecord &queryCmptRec = queryCmptList.get(i);
		if(queryCmptRec.getComponentID() == 255) // All Cmpts
		{
			model::Node treeNode(this->systemTree->getNode(subsId, queryNodeRec.getNodeID()));

			const std::map< unsigned char, model::Component * >& cmptMap = treeNode.getComponents();
			std::map< unsigned char, model::Component * >::const_iterator iter;
			for(iter = cmptMap.begin(); iter != cmptMap.end(); iter++)
			{
				queryCmptRec.setComponentID(iter->second->getId());
				reportNodeRec.getServicesComponentList().add(reportServiceListGetComponent(subsId, queryNodeRec.getNodeID(), queryCmptRec));
			}
		}
		else
		{
			if(this->systemTree->hasComponent(subsId, queryNodeRec.getNodeID(), queryCmptRec.getComponentID()))
			{
				reportNodeRec.getServicesComponentList().add(reportServiceListGetComponent(subsId, queryNodeRec.getNodeID(), queryCmptRec));
			}
		}
	}

	return reportNodeRec;
}

core::ServicesComponentRecord Base::reportServiceListGetComponent(uint16 subsId, uint8 nodeId, core::QSLComponentRecord &queryCmptRec)
{
	if(!this->systemTree->hasComponent(subsId, nodeId, queryCmptRec.getComponentID()))
	{
		THROW_EXCEPTION("Undefined component queried: " << subsId << "," << static_cast<int>(nodeId) << "," << static_cast<int>(queryCmptRec.getComponentID()));
	}

	model::Component treeCmpt(this->systemTree->getComponent(subsId, nodeId, queryCmptRec.getComponentID()));

	core::ServicesComponentRecord reportCmptRec;
	reportCmptRec.setComponentID(queryCmptRec.getComponentID());

	if(queryCmptRec.isSearchFilterEnabled())
	{
		// Look through all this component's service URIs for the filter
		const std::map< std::string, model::Service * >& serviceMap = treeCmpt.getServices();
		std::map< std::string, model::Service * >::const_iterator iter;
		for(iter = serviceMap.begin(); iter != serviceMap.end(); ++iter)
		{
			if(iter->second->getUri().find(queryCmptRec.getSearchFilter()) != std::string::npos)
			{
				// Found, add the ServiceRec to the List
				core::ServiceInformationRecord serviceRec;
				serviceRec.setUri(iter->second->getUri());
				serviceRec.setMajorVersionNumber(iter->second->getVersionMajor());
				serviceRec.setMinorVersionNumber(iter->second->getVersionMinor());
				reportCmptRec.getServicesServiceList().add(serviceRec);
			}
		}
	}
	else
	{
		const std::map< std::string, model::Service * >& serviceMap = treeCmpt.getServices();
		std::map< std::string, model::Service * >::const_iterator iter;
		for(iter = serviceMap.begin(); iter != serviceMap.end(); ++iter)
		{
			core::ServiceInformationRecord serviceRec;
			serviceRec.setUri(iter->second->getUri());
			serviceRec.setMajorVersionNumber(iter->second->getVersionMajor());
			serviceRec.setMinorVersionNumber(iter->second->getVersionMinor());
			reportCmptRec.getServicesServiceList().add(serviceRec);
		}
	}

	return reportCmptRec;
}

void Base::addSelfToSystemTree()
{
	systemTree->lock();
	LOG_DEBUG(name << " adding itself to system tree");
	systemTree->addLocalComponent(address);

	systemTree->setSubsystemIdentification(address, system::Application::setting<std::string>("SubsystemName", "OpenJAUS_Subsystem"));
	system::Application::comment("SubsystemName", "This defines the Subsystem Name for this JAUS Subsystem. Options: Any string");

	systemTree->setNodeIdentification(address, system::Application::setting<std::string>("NodeName", "OpenJAUS_Node"));
	system::Application::comment("NodeName", "This defines the Node Name for this JAUS Node. Options: Any string");

	systemTree->setComponentIdentification(address, system::Application::setting<std::string>(name + ".PublishedName", name));
	system::Application::comment(name + ".PublishedName", "This defines the Component Name for this JAUS Component. Options: Any string");

	// Add my services to the systemTree
	for(size_t i = 0; i < this->implements->size(); i++)
	{
		model::Service *service = this->implements->at(i);
		systemTree->addService(address, service->getUri(), service->getVersionMajor(), service->getVersionMinor());
	}
	LOG(systemTree->toString());
	systemTree->unlock();
}

void Base::run()
{
	model::Component::run();

	std::string addressMethod = system::Application::setting<std::string>("AddressingMethod", "BroadcastDiscover");
	system::Application::comment("AddressingMethod", "Determines how the JAUS addresses are set. Options are: BroadcastDiscover or Static");
	if(addressMethod == "BroadcastDiscover")
	{
		core::QueryJausAddress *queryAddress = new core::QueryJausAddress();
		broadcastToNode(queryAddress);
		LOG(name << " Sent broadcast query address to node");

		timer = new system::Timer(TIMER_METHOD(Base, setComponentId), this);
		timer->setInterval(500); // Magic Number (500): Fire this trigger after 500 millisecs
	}
	else if(addressMethod == "Static")
	{
		// Test if StaticSubsystemID is equal to Node1Heartbeat, if so we need to wait for a HB from Node 1 to set our SubsystemID
		if(system::Application::setting<std::string>("StaticSubsystemID", "1") != "Node1Heartbeat")
		{
			address.setSubsystem(system::Application::setting<int>("StaticSubsystemID", 1));
		}
		system::Application::comment("StaticSubsystemID", "This is the Subsystem ID of the static JAUS Address. Options are: <1-254> or Node1Heartbeat");

		address.setNode(system::Application::setting<int>("StaticNodeID", 1));
		system::Application::comment("StaticNodeID", "This is the Node ID of the static JAUS Address. Options are: <1-254>");

		unsigned char availableCmptID = systemTree->getAvailableComponentId(address.getSubsystem(), address.getNode());
		address.setComponent(system::Application::setting<int>(name + ".StaticComponentID", (int)availableCmptID));
		system::Application::comment(name + ".StaticComponentID", "This is the Component ID of the static JAUS Address. Options are: <1-254>");

		if(address.isValid())
		{
			addSelfToSystemTree();
			sendQueryIdentifications(0);
		}
	}

	std::string heartbeatMethod = system::Application::setting<std::string>("HeartbeatMethod", "None");
	system::Application::comment("HeartbeatMethod", "This defines the Heartbeat behavior. Options are: None or Broadcast");
	if(heartbeatMethod != "None")
	{
		heartbeatTimer = new system::Timer(TIMER_METHOD(Base, sendHeartbeats), this);
		heartbeatTimer->setInterval(1000);
	}
}

void Base::sendHeartbeats(system::Timer *timer)
{
	hbSequenceNumber++;

	core::ReportHeartbeatPulse* message = new core::ReportHeartbeatPulse();
	message->setSequenceNumber(hbSequenceNumber);
	broadcastToSystem(message);

	message = new core::ReportHeartbeatPulse();
	message->setSequenceNumber(hbSequenceNumber);
	broadcastToSubsystem(message);

	message = new core::ReportHeartbeatPulse();
	message->setSequenceNumber(hbSequenceNumber);
	broadcastToNode(message);
}

void Base::setComponentId(system::Timer *timer)
{
	address.setComponent(systemTree->getAvailableComponentId(address.getSubsystem(), address.getNode()));

	LOG(name << " Setting: " << address);

	if(!address.getNode())
	{
		core::QueryJausAddress *queryAddress = new core::QueryJausAddress();
		queryAddress->setPayload(queryAddress);
		broadcastToSubsystem(queryAddress);
		LOG(name << " Sent broadcast query address to subsystem");
		timer->setTimerFunction(TIMER_METHOD(Base, setNodeId), this);
	}
	else
	{
		setNodeId(timer);
	}

}

void Base::setNodeId(system::Timer *timer)
{
	if(!address.getNode())
	{
		systemTree->lock();
		if(systemTree->getThisNode())
		{
			address.setNode(systemTree->getThisNode());
		}
		else
		{
			address.setNode(systemTree->getAvailableNodeId(address.getSubsystem()));
			systemTree->setThisNode(address.getNode());
		}
		systemTree->unlock();

		LOG(name << " Setting: " << address);
	}

	if(!address.getSubsystem())
	{
		core::QueryJausAddress *queryAddress = new core::QueryJausAddress();
		queryAddress->setPayload(queryAddress);
		broadcastToSystem(queryAddress);
		LOG(name << " Sent broadcast query address to system");
		timer->setTimerFunction(TIMER_METHOD(Base, setSubsystemId), this);
	}
	else
	{
		setSubsystemId(timer);
	}

}

void Base::setSubsystemId(system::Timer *timer)
{
	if(!address.getSubsystem())
	{
		systemTree->lock();
		if(systemTree->getThisSubsytem())
		{
			LOG_DEBUG(name << " this subsystem is: " << systemTree->getThisSubsytem());
			address.setSubsystem(systemTree->getThisSubsytem());
		}
		else
		{
			address.setSubsystem(systemTree->getAvailableSubsystemId());
			LOG_DEBUG(name << " setting this subsystem to: " << address.getSubsystem());
			systemTree->setThisSubsytem(address.getSubsystem());
		}
		systemTree->unlock();
		LOG(name << " Setting: " << address);
	}

	addSelfToSystemTree();
	// systemTree->registerService???

	timer->stop();
	sendQueryIdentifications(timer);
}

void Base::sendQueryIdentifications(system::Timer *timer)
{
	core::QueryIdentification *queryId;

	LOG(name << " Sending broadcast query cmpt ids to node");
	queryId = new core::QueryIdentification();
	queryId->setQueryType(core::SystemLevelEnumeration::COMPONENT);
	broadcastToNode(queryId);

	LOG(name << " Sending broadcast query node ids to subsystem");
	queryId = new core::QueryIdentification();
	queryId->setQueryType(core::SystemLevelEnumeration::NODE);
	broadcastToSubsystem(queryId);

	LOG(name << " Sending broadcast query subsystem ids to system");
	queryId = new core::QueryIdentification();
	queryId->setQueryType(core::SystemLevelEnumeration::SUBSYSTEM);
	broadcastToSystem(queryId);
}
// End of user code

} // namespace component
} // namespace openjaus

