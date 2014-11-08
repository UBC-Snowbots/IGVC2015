/**
\file DigitalVideoSensorErrorRecord.h

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
#include "openjaus/environment/Triggers/Fields/DigitalVideoSensorErrorRecord.h"

namespace openjaus
{
namespace environment
{

DigitalVideoSensorErrorRecord::DigitalVideoSensorErrorRecord():
	sensorID(),
	digitalVideoErrorCode(),
	errorMessage()
{

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&digitalVideoErrorCode);
	digitalVideoErrorCode.setName("DigitalVideoErrorCode");
	digitalVideoErrorCode.setOptional(false);
	// Nothing to init

	fields.push_back(&errorMessage);
	errorMessage.setName("ErrorMessage");
	errorMessage.setOptional(false);
	errorMessage.setSizeType(model::fields::UNSIGNED_BYTE);

}

DigitalVideoSensorErrorRecord::DigitalVideoSensorErrorRecord(const DigitalVideoSensorErrorRecord &source)
{
	this->copy(const_cast<DigitalVideoSensorErrorRecord&>(source));
}

DigitalVideoSensorErrorRecord::~DigitalVideoSensorErrorRecord()
{

}


uint16_t DigitalVideoSensorErrorRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void DigitalVideoSensorErrorRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

DigitalVideoErrorCodeEnumeration::DigitalVideoErrorCodeEnum DigitalVideoSensorErrorRecord::getDigitalVideoErrorCode(void)
{
	return this->digitalVideoErrorCode.getValue();
}

void DigitalVideoSensorErrorRecord::setDigitalVideoErrorCode(DigitalVideoErrorCodeEnumeration::DigitalVideoErrorCodeEnum value)
{
	this->digitalVideoErrorCode.setValue(value);
}

std::string DigitalVideoSensorErrorRecord::getErrorMessage(void)
{
	return this->errorMessage.getValue();
}

void DigitalVideoSensorErrorRecord::setErrorMessage(std::string value)
{
	this->errorMessage.setValue(value);
}

int DigitalVideoSensorErrorRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(digitalVideoErrorCode);
	byteSize += dst->pack(errorMessage);
	return byteSize;
}
int DigitalVideoSensorErrorRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(digitalVideoErrorCode);
	byteSize += src->unpack(errorMessage);
	return byteSize;
}

int DigitalVideoSensorErrorRecord::length(void)
{
	int length = 0;
	length += sensorID.length(); // sensorID
	length += digitalVideoErrorCode.length(); // digitalVideoErrorCode
	length += errorMessage.length(); // errorMessage
	return length;
}

std::string DigitalVideoSensorErrorRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"DigitalVideoSensorErrorRecord\">\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << digitalVideoErrorCode.toXml(level+1); // digitalVideoErrorCode
	oss << errorMessage.toXml(level+1); // errorMessage
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void DigitalVideoSensorErrorRecord::copy(DigitalVideoSensorErrorRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->digitalVideoErrorCode.setName("DigitalVideoErrorCode");
	this->digitalVideoErrorCode.setOptional(false);
	this->digitalVideoErrorCode.setValue(source.getDigitalVideoErrorCode()); 
 
	this->errorMessage.setName("ErrorMessage");
	this->errorMessage.setOptional(false);
	this->errorMessage.setValue(source.getErrorMessage());
	this->errorMessage.setSizeType(model::fields::UNSIGNED_BYTE); 
 
}

} // namespace environment
} // namespace openjaus

