/**
\file DigitalVideoSensor.cpp

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


#include "openjaus/environment/DigitalVideoSensor.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace environment
{

DigitalVideoSensor::DigitalVideoSensor() : Base(),
	digitalVideoDefaultLoop(),
	digitalVideoControlledLoop(),
	visualSensorDefaultLoop(),
	visualSensorControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "DigitalVideoSensor";
	
	model::Service *digitalVideoService = new model::Service();
	digitalVideoService->setName("DigitalVideo");
	digitalVideoService->setUri("urn:jaus:jss:environmentSensing:DigitalVideo");
	digitalVideoService->setVersionMajor(1);
	digitalVideoService->setVersionMinor(0);
	this->implements->push_back(digitalVideoService);
	
	model::Service *visualSensorService = new model::Service();
	visualSensorService->setName("VisualSensor");
	visualSensorService->setUri("urn:jaus:jss:environmentSensing:VisualSensor");
	visualSensorService->setVersionMajor(1);
	visualSensorService->setVersionMinor(0);
	this->implements->push_back(visualSensorService);
	
	
	

	digitalVideoDefaultLoop.setInterface(this);
	digitalVideoDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(digitalVideoDefaultLoop);
	
	digitalVideoControlledLoop.setInterface(this);
	digitalVideoControlledLoop.setTransportInterface(this);
	controlled.addTransition(digitalVideoControlledLoop);
	
	visualSensorDefaultLoop.setInterface(this);
	visualSensorDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(visualSensorDefaultLoop);
	
	visualSensorControlledLoop.setInterface(this);
	visualSensorControlledLoop.setTransportInterface(this);
	controlled.addTransition(visualSensorControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

DigitalVideoSensor::~DigitalVideoSensor()
{
	// Start of user code for Destructor:
	// End of user code
}

bool DigitalVideoSensor::modifyDigitalVideoSensorStream(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream)
{
	// Start of user code for action modifyDigitalVideoSensorStream(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream):
	return false;
	// End of user code
}

environment::ConfirmDigitalVideoSensorConfiguration DigitalVideoSensor::getConfirmDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration)
{
	// Start of user code for action getConfirmDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration):
	environment::ConfirmDigitalVideoSensorConfiguration message;
	return message;
	// End of user code
}

bool DigitalVideoSensor::updateDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration)
{
	// Start of user code for action updateDigitalVideoSensorConfiguration(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration):
	return false;
	// End of user code
}

environment::ReportDigitalVideoSensorConfiguration DigitalVideoSensor::getReportDigitalVideoSensorConfiguration(QueryDigitalVideoSensorConfiguration *queryDigitalVideoSensorConfiguration)
{
	// Start of user code for action getReportDigitalVideoSensorConfiguration(QueryDigitalVideoSensorConfiguration *queryDigitalVideoSensorConfiguration):
	environment::ReportDigitalVideoSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportDigitalVideoSensorCapabilities DigitalVideoSensor::getReportDigitalVideoSensorCapabilities(QueryDigitalVideoSensorCapabilities *queryDigitalVideoSensorCapabilities)
{
	// Start of user code for action getReportDigitalVideoSensorCapabilities(QueryDigitalVideoSensorCapabilities *queryDigitalVideoSensorCapabilities):
	environment::ReportDigitalVideoSensorCapabilities message;
	return message;
	// End of user code
}

environment::ReportVisualSensorGeometricProperties DigitalVideoSensor::getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties)
{
	// Start of user code for action getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties):
	environment::ReportVisualSensorGeometricProperties message;
	return message;
	// End of user code
}

environment::ConfirmVisualSensorConfiguration DigitalVideoSensor::getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	environment::ConfirmVisualSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportVisualSensorConfiguration DigitalVideoSensor::getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration)
{
	// Start of user code for action getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration):
	environment::ReportVisualSensorConfiguration message;
	return message;
	// End of user code
}

bool DigitalVideoSensor::updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	return false;
	// End of user code
}

environment::ReportVisualSensorCapabilities DigitalVideoSensor::getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities)
{
	// Start of user code for action getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities):
	environment::ReportVisualSensorCapabilities message;
	return message;
	// End of user code
}


bool DigitalVideoSensor::isControllingDigitalVideoClient(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration)
{
	// Start of user code for action isControllingDigitalVideoClient(SetDigitalVideoSensorConfiguration *setDigitalVideoSensorConfiguration):
	return false;
	// End of user code
}

bool DigitalVideoSensor::isControllingDigitalVideoClient(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream)
{
	// Start of user code for action isControllingDigitalVideoClient(ControlDigitalVideoSensorStream *controlDigitalVideoSensorStream):
	return false;
	// End of user code
}


bool DigitalVideoSensor::isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

