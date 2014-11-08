/**
\file StillImageSensor.cpp

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


#include "openjaus/environment/StillImageSensor.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace environment
{

StillImageSensor::StillImageSensor() : Base(),
	stillImageDefaultLoop(),
	stillImageControlledLoop(),
	visualSensorDefaultLoop(),
	visualSensorControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "StillImageSensor";
	
	model::Service *stillImageService = new model::Service();
	stillImageService->setName("StillImage");
	stillImageService->setUri("urn:jaus:jss:environmentSensing:StillImage");
	stillImageService->setVersionMajor(1);
	stillImageService->setVersionMinor(0);
	this->implements->push_back(stillImageService);
	
	model::Service *visualSensorService = new model::Service();
	visualSensorService->setName("VisualSensor");
	visualSensorService->setUri("urn:jaus:jss:environmentSensing:VisualSensor");
	visualSensorService->setVersionMajor(1);
	visualSensorService->setVersionMinor(0);
	this->implements->push_back(visualSensorService);
	
	
	

	stillImageDefaultLoop.setInterface(this);
	stillImageDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(stillImageDefaultLoop);
	
	stillImageControlledLoop.setInterface(this);
	stillImageControlledLoop.setTransportInterface(this);
	controlled.addTransition(stillImageControlledLoop);
	
	visualSensorDefaultLoop.setInterface(this);
	visualSensorDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(visualSensorDefaultLoop);
	
	visualSensorControlledLoop.setInterface(this);
	visualSensorControlledLoop.setTransportInterface(this);
	controlled.addTransition(visualSensorControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

StillImageSensor::~StillImageSensor()
{
	// Start of user code for Destructor:
	// End of user code
}

environment::ReportStillImageSensorCapabilities StillImageSensor::getReportStillImageSensorCapabilities(QueryStillImageSensorCapabilities *queryStillImageSensorCapabilities)
{
	// Start of user code for action getReportStillImageSensorCapabilities(QueryStillImageSensorCapabilities *queryStillImageSensorCapabilities):
	environment::ReportStillImageSensorCapabilities message;
	return message;
	// End of user code
}

environment::ReportStillImageSensorConfiguration StillImageSensor::getReportStillImageSensorConfiguration(QueryStillImageSensorConfiguration *queryStillImageSensorConfiguration)
{
	// Start of user code for action getReportStillImageSensorConfiguration(QueryStillImageSensorConfiguration *queryStillImageSensorConfiguration):
	environment::ReportStillImageSensorConfiguration message;
	return message;
	// End of user code
}

environment::ConfirmStillImageSensorConfiguration StillImageSensor::getConfirmStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration)
{
	// Start of user code for action getConfirmStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration):
	environment::ConfirmStillImageSensorConfiguration message;
	return message;
	// End of user code
}

bool StillImageSensor::updateStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration)
{
	// Start of user code for action updateStillImageSensorConfiguration(SetStillImageSensorConfiguration *setStillImageSensorConfiguration):
	return false;
	// End of user code
}

environment::ReportStillImageData StillImageSensor::getReportStillImageData(QueryStillImageData *queryStillImageData)
{
	// Start of user code for action getReportStillImageData(QueryStillImageData *queryStillImageData):
	environment::ReportStillImageData message;
	return message;
	// End of user code
}

environment::ReportStillImageData StillImageSensor::getReportStillImageDataInNativeSystem(QueryStillImageData *queryStillImageData)
{
	// Start of user code for action getReportStillImageDataInNativeSystem(QueryStillImageData *queryStillImageData):
	environment::ReportStillImageData message;
	return message;
	// End of user code
}

environment::ReportVisualSensorGeometricProperties StillImageSensor::getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties)
{
	// Start of user code for action getReportVisualSensorGeometricProperties(QueryVisualSensorGeometricProperties *queryVisualSensorGeometricProperties):
	environment::ReportVisualSensorGeometricProperties message;
	return message;
	// End of user code
}

environment::ConfirmVisualSensorConfiguration StillImageSensor::getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action getConfirmVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	environment::ConfirmVisualSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportVisualSensorConfiguration StillImageSensor::getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration)
{
	// Start of user code for action getReportVisualSensorConfiguration(QueryVisualSensorConfiguration *queryVisualSensorConfiguration):
	environment::ReportVisualSensorConfiguration message;
	return message;
	// End of user code
}

bool StillImageSensor::updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action updateVisualSensorConfiguration(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	return false;
	// End of user code
}

environment::ReportVisualSensorCapabilities StillImageSensor::getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities)
{
	// Start of user code for action getReportVisualSensorCapabilities(QueryVisualSensorCapabilities *queryVisualSensorCapabilities):
	environment::ReportVisualSensorCapabilities message;
	return message;
	// End of user code
}


bool StillImageSensor::isControllingStillImageClient(SetStillImageSensorConfiguration *setStillImageSensorConfiguration)
{
	// Start of user code for action isControllingStillImageClient(SetStillImageSensorConfiguration *setStillImageSensorConfiguration):
	return false;
	// End of user code
}


bool StillImageSensor::isCoordinateTransformSupported(QueryStillImageData *queryStillImageData)
{
	// Start of user code for action isCoordinateTransformSupported(QueryStillImageData *queryStillImageData):
	return false;
	// End of user code
}


bool StillImageSensor::isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration)
{
	// Start of user code for action isControllingVisualSensorClient(SetVisualSensorConfiguration *setVisualSensorConfiguration):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

