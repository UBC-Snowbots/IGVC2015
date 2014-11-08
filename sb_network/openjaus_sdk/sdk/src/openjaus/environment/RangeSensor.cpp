/**
\file RangeSensor.cpp

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


#include "openjaus/environment/RangeSensor.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace environment
{

RangeSensor::RangeSensor() : Base(),
	rangeSensorDefaultLoop(),
	rangeSensorControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "RangeSensor";
	
	model::Service *rangeSensorService = new model::Service();
	rangeSensorService->setName("RangeSensor");
	rangeSensorService->setUri("urn:jaus:jss:environmentSensing:RangeSensor");
	rangeSensorService->setVersionMajor(1);
	rangeSensorService->setVersionMinor(0);
	this->implements->push_back(rangeSensorService);
	
	

	rangeSensorDefaultLoop.setInterface(this);
	rangeSensorDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(rangeSensorDefaultLoop);
	
	rangeSensorControlledLoop.setInterface(this);
	rangeSensorControlledLoop.setTransportInterface(this);
	controlled.addTransition(rangeSensorControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

RangeSensor::~RangeSensor()
{
	// Start of user code for Destructor:
	// End of user code
}

environment::ReportRangeSensorData RangeSensor::getReportRangeSensorData(QueryRangeSensorData *queryRangeSensorData)
{
	// Start of user code for action getReportRangeSensorData(QueryRangeSensorData *queryRangeSensorData):
	environment::ReportRangeSensorData message;
	return message;
	// End of user code
}

environment::ReportRangeSensorGeometricProperties RangeSensor::getReportRangeSensorGeometricProperties(QuerySensorGeometricProperties *querySensorGeometricProperties)
{
	// Start of user code for action getReportRangeSensorGeometricProperties(QuerySensorGeometricProperties *querySensorGeometricProperties):
	environment::ReportRangeSensorGeometricProperties message;
	return message;
	// End of user code
}

bool RangeSensor::updateRangeSensorConfiguration(SetRangeSensorConfiguration *setRangeSensorConfiguration)
{
	// Start of user code for action updateRangeSensorConfiguration(SetRangeSensorConfiguration *setRangeSensorConfiguration):
	return false;
	// End of user code
}

environment::ConfirmRangeSensorConfiguration RangeSensor::getConfirmRangeSensorConfiguration(SetRangeSensorConfiguration *setRangeSensorConfiguration)
{
	// Start of user code for action getConfirmRangeSensorConfiguration(SetRangeSensorConfiguration *setRangeSensorConfiguration):
	environment::ConfirmRangeSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportRangeSensorConfiguration RangeSensor::getReportRangeSensorConfiguration(QueryRangeSensorConfiguration *queryRangeSensorConfiguration)
{
	// Start of user code for action getReportRangeSensorConfiguration(QueryRangeSensorConfiguration *queryRangeSensorConfiguration):
	environment::ReportRangeSensorConfiguration message;
	return message;
	// End of user code
}

environment::ReportRangeSensorCompressedData RangeSensor::getReportRangeSensorCompressedData(QueryRangeSensorCompressedData *queryRangeSensorCompressedData)
{
	// Start of user code for action getReportRangeSensorCompressedData(QueryRangeSensorCompressedData *queryRangeSensorCompressedData):
	environment::ReportRangeSensorCompressedData message;
	return message;
	// End of user code
}

environment::ReportRangeSensorCapabilities RangeSensor::getReportRangeSensorCapabilities(QueryRangeSensorCapabilities *queryRangeSensorCapabilities)
{
	// Start of user code for action getReportRangeSensorCapabilities(QueryRangeSensorCapabilities *queryRangeSensorCapabilities):
	environment::ReportRangeSensorCapabilities message;
	return message;
	// End of user code
}

environment::ReportRangeSensorData RangeSensor::getReportRangeSensorDataInNativeSystem(QueryRangeSensorData *queryRangeSensorData)
{
	// Start of user code for action getReportRangeSensorDataInNativeSystem(QueryRangeSensorData *queryRangeSensorData):
	environment::ReportRangeSensorData message;
	return message;
	// End of user code
}

environment::ReportRangeSensorCompressedData RangeSensor::getReportRangeSensorCompressedDataInNativeSystem(QueryRangeSensorCompressedData *queryRangeSensorCompressedData)
{
	// Start of user code for action getReportRangeSensorCompressedDataInNativeSystem(QueryRangeSensorCompressedData *queryRangeSensorCompressedData):
	environment::ReportRangeSensorCompressedData message;
	return message;
	// End of user code
}


bool RangeSensor::isCoordinateTransformSupported(QueryRangeSensorData *queryRangeSensorData)
{
	// Start of user code for action isCoordinateTransformSupported(QueryRangeSensorData *queryRangeSensorData):
	return false;
	// End of user code
}

bool RangeSensor::isCoordinateTransformSupported(QueryRangeSensorCompressedData *queryRangeSensorCompressedData)
{
	// Start of user code for action isCoordinateTransformSupported(QueryRangeSensorCompressedData *queryRangeSensorCompressedData):
	return false;
	// End of user code
}


bool RangeSensor::isControllingRangeSensorClient(model::Trigger *trigger)
{
	// Start of user code for action isControllingRangeSensorClient(model::Trigger *trigger):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

