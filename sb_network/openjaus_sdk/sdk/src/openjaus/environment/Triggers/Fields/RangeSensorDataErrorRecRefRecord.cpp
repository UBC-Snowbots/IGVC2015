/**
\file RangeSensorDataErrorRecRefRecord.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorDataErrorRecRefRecord.h"

namespace openjaus
{
namespace environment
{

RangeSensorDataErrorRecRefRecord::RangeSensorDataErrorRecRefRecord():
	sensorID(),
	dataErrorCode(),
	errorMessage()
{

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&dataErrorCode);
	dataErrorCode.setName("DataErrorCode");
	dataErrorCode.setOptional(false);
	// Nothing to init

	fields.push_back(&errorMessage);
	errorMessage.setName("ErrorMessage");
	errorMessage.setOptional(false);
	errorMessage.setSizeType(model::fields::UNSIGNED_BYTE);

}

RangeSensorDataErrorRecRefRecord::RangeSensorDataErrorRecRefRecord(const RangeSensorDataErrorRecRefRecord &source)
{
	this->copy(const_cast<RangeSensorDataErrorRecRefRecord&>(source));
}

RangeSensorDataErrorRecRefRecord::~RangeSensorDataErrorRecRefRecord()
{

}


uint16_t RangeSensorDataErrorRecRefRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void RangeSensorDataErrorRecRefRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

DataErrorCodeEnumeration::DataErrorCodeEnum RangeSensorDataErrorRecRefRecord::getDataErrorCode(void)
{
	return this->dataErrorCode.getValue();
}

void RangeSensorDataErrorRecRefRecord::setDataErrorCode(DataErrorCodeEnumeration::DataErrorCodeEnum value)
{
	this->dataErrorCode.setValue(value);
}

std::string RangeSensorDataErrorRecRefRecord::getErrorMessage(void)
{
	return this->errorMessage.getValue();
}

void RangeSensorDataErrorRecRefRecord::setErrorMessage(std::string value)
{
	this->errorMessage.setValue(value);
}

int RangeSensorDataErrorRecRefRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(dataErrorCode);
	byteSize += dst->pack(errorMessage);
	return byteSize;
}
int RangeSensorDataErrorRecRefRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(dataErrorCode);
	byteSize += src->unpack(errorMessage);
	return byteSize;
}

int RangeSensorDataErrorRecRefRecord::length(void)
{
	int length = 0;
	length += sensorID.length(); // sensorID
	length += dataErrorCode.length(); // dataErrorCode
	length += errorMessage.length(); // errorMessage
	return length;
}

std::string RangeSensorDataErrorRecRefRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RangeSensorDataErrorRecRefRecord\">\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << dataErrorCode.toXml(level+1); // dataErrorCode
	oss << errorMessage.toXml(level+1); // errorMessage
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void RangeSensorDataErrorRecRefRecord::copy(RangeSensorDataErrorRecRefRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->dataErrorCode.setName("DataErrorCode");
	this->dataErrorCode.setOptional(false);
	this->dataErrorCode.setValue(source.getDataErrorCode()); 
 
	this->errorMessage.setName("ErrorMessage");
	this->errorMessage.setOptional(false);
	this->errorMessage.setValue(source.getErrorMessage());
	this->errorMessage.setSizeType(model::fields::UNSIGNED_BYTE); 
 
}

} // namespace environment
} // namespace openjaus

