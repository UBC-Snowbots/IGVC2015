/**
\file AnalogVideoSensorConfigurationRecord.h

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
#include "openjaus/environment/Triggers/Fields/AnalogVideoSensorConfigurationRecord.h"

namespace openjaus
{
namespace environment
{

AnalogVideoSensorConfigurationRecord::AnalogVideoSensorConfigurationRecord():
	sensorID(),
	analogFormat()
{

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&analogFormat);
	analogFormat.setName("AnalogFormat");
	analogFormat.setOptional(false);
	// Nothing to init

}

AnalogVideoSensorConfigurationRecord::AnalogVideoSensorConfigurationRecord(const AnalogVideoSensorConfigurationRecord &source)
{
	this->copy(const_cast<AnalogVideoSensorConfigurationRecord&>(source));
}

AnalogVideoSensorConfigurationRecord::~AnalogVideoSensorConfigurationRecord()
{

}


uint16_t AnalogVideoSensorConfigurationRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void AnalogVideoSensorConfigurationRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

AnalogFormatEnumeration::AnalogFormatEnum AnalogVideoSensorConfigurationRecord::getAnalogFormat(void)
{
	return this->analogFormat.getValue();
}

void AnalogVideoSensorConfigurationRecord::setAnalogFormat(AnalogFormatEnumeration::AnalogFormatEnum value)
{
	this->analogFormat.setValue(value);
}

int AnalogVideoSensorConfigurationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(analogFormat);
	return byteSize;
}
int AnalogVideoSensorConfigurationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(analogFormat);
	return byteSize;
}

int AnalogVideoSensorConfigurationRecord::length(void)
{
	int length = 0;
	length += sensorID.length(); // sensorID
	length += analogFormat.length(); // analogFormat
	return length;
}

std::string AnalogVideoSensorConfigurationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"AnalogVideoSensorConfigurationRecord\">\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << analogFormat.toXml(level+1); // analogFormat
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void AnalogVideoSensorConfigurationRecord::copy(AnalogVideoSensorConfigurationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->analogFormat.setName("AnalogFormat");
	this->analogFormat.setOptional(false);
	this->analogFormat.setValue(source.getAnalogFormat()); 
 
}

} // namespace environment
} // namespace openjaus

