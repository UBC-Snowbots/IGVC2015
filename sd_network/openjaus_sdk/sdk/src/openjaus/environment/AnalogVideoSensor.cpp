/**
\file AnalogVideoSensor.cpp

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


#include "openjaus/environment/AnalogVideoSensor.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace environment
{

AnalogVideoSensor::AnalogVideoSensor() : Base(),
	analogVideoDefaultLoop(),
	analogVideoControlledLoop(),
	visualSensorDefaultLoop(),
	visualSensorControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "AnalogVideoSensor";
	
	model::Service *analogVideoService = new model::Service();
	analogVideoService->setName("AnalogVideo");
	analogVideoService->setUri("urn:jaus:jss:environmentSensing:AnalogVideo");
	analogVideoService->setVersionMajor(1);
	analogVideoService->setVersionMinor(0);
	this->implements->push_back(analogVideoService);
	
	model::Service *visualSensorService = new model::Service();
	visualSensorService->setName("VisualSensor");
	visualSensorService->setUri("urn:jaus:jss:environmentSensing:VisualSensor");
	visualSensorService->setVersionMajor(1);
	visualSensorService->setVersionMinor(0);
	this->implements->push_back(visualSensorService);
	
	
	

	analogVideoDefaultLoop.setInterface(this);
	analogVideoDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(analogVideoDefaultLoop);
	
	analogVideoControlledLoop.setInterface(this);
	analogVideoControlledLoop.setTransportInterface(this);
	controlled.addTransition(analogVideoControlledLoop);
	
	visualSensorDefaultLoop.setInterface(this);
	visualSensorDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(visualSensorDefaultLoop);
	
	visualSensorControlledLoop.setInterface(this);
	visualSensorControlledLoop.setTransportInterface(this);
	controlled.addTransition(visualSensorControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

AnalogVideoSensor::~AnalogVideoSensor()
{
	// Start of user code for Destructor:
	// End of user code
}

environment::ReportAnalogVideoSensorCapabilities AnalogVideoSensor::getReportAnalogVideoSensorCapabilities(QueryAnalogVideoSensorCapabilities *queryAnalogVideoSensorCapabilities)
{
	// Start of user code for action getReportAnalogVideoSensorCapabilities(QueryAnalogVideoSensorCapabilities *queryAnalogVideoSensorCapabilities):
	environment::ReportAnalogVideoSensorCapabilities message;
	return message;
	// End of user code
}

bool AnalogVideoSensor::updateAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration)
{
	// Start of user code for action updateAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration):
	return false;
	// End of user code
}

environment::ConfirmAnalogVideoSensorConfiguration AnalogVideoSensor::getConfirmAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration)
{
	// Start of user code for action getConfirmAnalogVideoSensorConfiguration(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration):
	environment::ConfirmAnalogVideoSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportAnalogVideoSensorConfiguration AnalogVideoSensor::getReportAnalogVideoSensorConfiguration(QueryAnalogVideoSensorConfiguration *queryAnalogVideoSensorConfiguration)
{
	// Start of user code for action getReportAnalogVideoSensorConfiguration(QueryAnalogVideoSensorConfiguration *queryAnalogVideoSensorConfiguration):
	environment::ReportAnalogVideoSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportVisualSensorGeometricProperties AnalogVideoSensor::getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties)
{
	// Start of user code for action getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties):
	environment::ReportVisualSensorGeometricProperties message;
	return message;
	// End of user code
}

environment::ConfirmVisualSensorConfiguration AnalogVideoSensor::getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	environment::ConfirmVisualSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportVisualSensorConfiguration AnalogVideoSensor::getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration)
{
	// Start of user code for action getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration):
	environment::ReportVisualSensorConfiguration message;
	return message;
	// End of user code
}

bool AnalogVideoSensor::updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	return false;
	// End of user code
}

environment::ReportVisualSensorCapabilities AnalogVideoSensor::getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities)
{
	// Start of user code for action getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities):
	environment::ReportVisualSensorCapabilities message;
	return message;
	// End of user code
}


bool AnalogVideoSensor::isControllingAnalogVideoClient(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration)
{
	// Start of user code for action isControllingAnalogVideoClient(SetAnalogVideoSensorConfiguration *setAnalogVideoSensorConfiguration):
	return false;
	// End of user code
}


bool AnalogVideoSensor::isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

