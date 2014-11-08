/**
\file GlobalPoseSensor.cpp

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


#include "openjaus/mobility/GlobalPoseSensor.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

GlobalPoseSensor::GlobalPoseSensor() : Base(),
	gposDefaultLoop(),
	gposControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "GlobalPoseSensor";
	
	model::Service *globalPoseSensorService = new model::Service();
	globalPoseSensorService->setName("GlobalPoseSensor");
	globalPoseSensorService->setUri("urn:jaus:jss:mobility:GlobalPoseSensor");
	globalPoseSensorService->setVersionMajor(1);
	globalPoseSensorService->setVersionMinor(0);
	this->implements->push_back(globalPoseSensorService);
	
	

	gposDefaultLoop.setInterface(this);
	gposDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(gposDefaultLoop);
	
	gposControlledLoop.setInterface(this);
	gposControlledLoop.setTransportInterface(this);
	controlled.addTransition(gposControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

GlobalPoseSensor::~GlobalPoseSensor()
{
	// Start of user code for Destructor:
	// End of user code
}

mobility::ReportGlobalPose GlobalPoseSensor::getReportGlobalPose(QueryGlobalPose *queryGlobalPose)
{
	// Start of user code for action getReportGlobalPose(QueryGlobalPose *queryGlobalPose):
	mobility::ReportGlobalPose message;
	return message;
	// End of user code
}

mobility::ReportGeomagneticProperty GlobalPoseSensor::getReportGeomagneticProperty(QueryGeomagneticProperty *queryGeomagneticProperty)
{
	// Start of user code for action getReportGeomagneticProperty(QueryGeomagneticProperty *queryGeomagneticProperty):
	mobility::ReportGeomagneticProperty message;
	return message;
	// End of user code
}

bool GlobalPoseSensor::updateGlobalPose(SetGlobalPose *setGlobalPose)
{
	// Start of user code for action updateGlobalPose(SetGlobalPose *setGlobalPose):
	return false;
	// End of user code
}

bool GlobalPoseSensor::updateGeomagneticProperty(SetGeomagneticProperty *setGeomagneticProperty)
{
	// Start of user code for action updateGeomagneticProperty(SetGeomagneticProperty *setGeomagneticProperty):
	return false;
	// End of user code
}


bool GlobalPoseSensor::isControllingGposClient(SetGlobalPose *setGlobalPose)
{
	// Start of user code for action isControllingGposClient(SetGlobalPose *setGlobalPose):
	return false;
	// End of user code
}

bool GlobalPoseSensor::isControllingGposClient(SetGeomagneticProperty *setGeomagneticProperty)
{
	// Start of user code for action isControllingGposClient(SetGeomagneticProperty *setGeomagneticProperty):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

