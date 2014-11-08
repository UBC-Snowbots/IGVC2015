/**
\file StaticGeometricPropertiesRecord.h

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
#include "openjaus/environment/Triggers/Fields/StaticGeometricPropertiesRecord.h"

namespace openjaus
{
namespace environment
{

StaticGeometricPropertiesRecord::StaticGeometricPropertiesRecord():
	sensorPosition(),
	unitQuaternion()
{

	fields.push_back(&sensorPosition);
	sensorPosition.setName("SensorPosition");
	sensorPosition.setOptional(false);
	// Nothing to Init

	fields.push_back(&unitQuaternion);
	unitQuaternion.setName("UnitQuaternion");
	unitQuaternion.setOptional(false);
	// Nothing to Init

}

StaticGeometricPropertiesRecord::StaticGeometricPropertiesRecord(const StaticGeometricPropertiesRecord &source)
{
	this->copy(const_cast<StaticGeometricPropertiesRecord&>(source));
}

StaticGeometricPropertiesRecord::~StaticGeometricPropertiesRecord()
{

}


SensorPositionRefArray& StaticGeometricPropertiesRecord::getSensorPosition(void)
{
	return this->sensorPosition;
}

UnitQuaternionRefArray& StaticGeometricPropertiesRecord::getUnitQuaternion(void)
{
	return this->unitQuaternion;
}

int StaticGeometricPropertiesRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorPosition);
	byteSize += dst->pack(unitQuaternion);
	return byteSize;
}
int StaticGeometricPropertiesRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorPosition);
	byteSize += src->unpack(unitQuaternion);
	return byteSize;
}

int StaticGeometricPropertiesRecord::length(void)
{
	int length = 0;
	length += sensorPosition.length(); // sensorPosition
	length += unitQuaternion.length(); // unitQuaternion
	return length;
}

std::string StaticGeometricPropertiesRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"StaticGeometricPropertiesRecord\">\n";
	oss << sensorPosition.toXml(level+1); // sensorPosition
	oss << unitQuaternion.toXml(level+1); // unitQuaternion
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void StaticGeometricPropertiesRecord::copy(StaticGeometricPropertiesRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->sensorPosition.copy(source.getSensorPosition()); 
 
	this->unitQuaternion.copy(source.getUnitQuaternion()); 
 
}

} // namespace environment
} // namespace openjaus

