/**
\file ManipulatorGeometricPropertiesRecord.h

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
#include "openjaus/environment/Triggers/Fields/ManipulatorGeometricPropertiesRecord.h"

namespace openjaus
{
namespace environment
{

ManipulatorGeometricPropertiesRecord::ManipulatorGeometricPropertiesRecord():
	subsystemID(),
	nodeID(),
	componentID(),
	jointNumber(),
	sensorPosition(),
	unitQuaternion()
{

	fields.push_back(&subsystemID);
	subsystemID.setName("SubsystemID");
	subsystemID.setOptional(false);
	subsystemID.setValue(0);

	fields.push_back(&nodeID);
	nodeID.setName("NodeID");
	nodeID.setOptional(false);
	nodeID.setValue(0);

	fields.push_back(&componentID);
	componentID.setName("ComponentID");
	componentID.setOptional(false);
	componentID.setValue(0);

	fields.push_back(&jointNumber);
	jointNumber.setName("JointNumber");
	jointNumber.setOptional(false);
	jointNumber.setValue(0);

	fields.push_back(&sensorPosition);
	sensorPosition.setName("SensorPosition");
	sensorPosition.setOptional(false);
	// Nothing to Init

	fields.push_back(&unitQuaternion);
	unitQuaternion.setName("UnitQuaternion");
	unitQuaternion.setOptional(false);
	// Nothing to Init

}

ManipulatorGeometricPropertiesRecord::ManipulatorGeometricPropertiesRecord(const ManipulatorGeometricPropertiesRecord &source)
{
	this->copy(const_cast<ManipulatorGeometricPropertiesRecord&>(source));
}

ManipulatorGeometricPropertiesRecord::~ManipulatorGeometricPropertiesRecord()
{

}


uint16_t ManipulatorGeometricPropertiesRecord::getSubsystemID(void)
{
	return this->subsystemID.getValue();
}

void ManipulatorGeometricPropertiesRecord::setSubsystemID(uint16_t value)
{
	this->subsystemID.setValue(value);
}

uint8_t ManipulatorGeometricPropertiesRecord::getNodeID(void)
{
	return this->nodeID.getValue();
}

void ManipulatorGeometricPropertiesRecord::setNodeID(uint8_t value)
{
	this->nodeID.setValue(value);
}

uint8_t ManipulatorGeometricPropertiesRecord::getComponentID(void)
{
	return this->componentID.getValue();
}

void ManipulatorGeometricPropertiesRecord::setComponentID(uint8_t value)
{
	this->componentID.setValue(value);
}

uint8_t ManipulatorGeometricPropertiesRecord::getJointNumber(void)
{
	return this->jointNumber.getValue();
}

void ManipulatorGeometricPropertiesRecord::setJointNumber(uint8_t value)
{
	this->jointNumber.setValue(value);
}

SensorPositionRefArray& ManipulatorGeometricPropertiesRecord::getSensorPosition(void)
{
	return this->sensorPosition;
}

UnitQuaternionRefArray& ManipulatorGeometricPropertiesRecord::getUnitQuaternion(void)
{
	return this->unitQuaternion;
}

int ManipulatorGeometricPropertiesRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(subsystemID);
	byteSize += dst->pack(nodeID);
	byteSize += dst->pack(componentID);
	byteSize += dst->pack(jointNumber);
	byteSize += dst->pack(sensorPosition);
	byteSize += dst->pack(unitQuaternion);
	return byteSize;
}
int ManipulatorGeometricPropertiesRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(subsystemID);
	byteSize += src->unpack(nodeID);
	byteSize += src->unpack(componentID);
	byteSize += src->unpack(jointNumber);
	byteSize += src->unpack(sensorPosition);
	byteSize += src->unpack(unitQuaternion);
	return byteSize;
}

int ManipulatorGeometricPropertiesRecord::length(void)
{
	int length = 0;
	length += subsystemID.length(); // subsystemID
	length += nodeID.length(); // nodeID
	length += componentID.length(); // componentID
	length += jointNumber.length(); // jointNumber
	length += sensorPosition.length(); // sensorPosition
	length += unitQuaternion.length(); // unitQuaternion
	return length;
}

std::string ManipulatorGeometricPropertiesRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ManipulatorGeometricPropertiesRecord\">\n";
	oss << subsystemID.toXml(level+1); // subsystemID
	oss << nodeID.toXml(level+1); // nodeID
	oss << componentID.toXml(level+1); // componentID
	oss << jointNumber.toXml(level+1); // jointNumber
	oss << sensorPosition.toXml(level+1); // sensorPosition
	oss << unitQuaternion.toXml(level+1); // unitQuaternion
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ManipulatorGeometricPropertiesRecord::copy(ManipulatorGeometricPropertiesRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->subsystemID.setName("SubsystemID");
	this->subsystemID.setOptional(false);
	this->subsystemID.setValue(source.getSubsystemID()); 
 
	this->nodeID.setName("NodeID");
	this->nodeID.setOptional(false);
	this->nodeID.setValue(source.getNodeID()); 
 
	this->componentID.setName("ComponentID");
	this->componentID.setOptional(false);
	this->componentID.setValue(source.getComponentID()); 
 
	this->jointNumber.setName("JointNumber");
	this->jointNumber.setOptional(false);
	this->jointNumber.setValue(source.getJointNumber()); 
 
	this->sensorPosition.copy(source.getSensorPosition()); 
 
	this->unitQuaternion.copy(source.getUnitQuaternion()); 
 
}

} // namespace environment
} // namespace openjaus

