/**
\file VisualSensorErrorRecord.h

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
#include "openjaus/environment/Triggers/Fields/VisualSensorErrorRecord.h"

namespace openjaus
{
namespace environment
{

VisualSensorErrorRecord::VisualSensorErrorRecord():
	sensorID(),
	visualSensorErrorCode(),
	errorMessage()
{

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&visualSensorErrorCode);
	visualSensorErrorCode.setName("VisualSensorErrorCode");
	visualSensorErrorCode.setOptional(false);
	// Nothing to init

	fields.push_back(&errorMessage);
	errorMessage.setName("ErrorMessage");
	errorMessage.setOptional(false);
	errorMessage.setSizeType(model::fields::UNSIGNED_BYTE);

}

VisualSensorErrorRecord::VisualSensorErrorRecord(const VisualSensorErrorRecord &source)
{
	this->copy(const_cast<VisualSensorErrorRecord&>(source));
}

VisualSensorErrorRecord::~VisualSensorErrorRecord()
{

}


uint16_t VisualSensorErrorRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void VisualSensorErrorRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

VisualSensorErrorCodeEnumeration::VisualSensorErrorCodeEnum VisualSensorErrorRecord::getVisualSensorErrorCode(void)
{
	return this->visualSensorErrorCode.getValue();
}

void VisualSensorErrorRecord::setVisualSensorErrorCode(VisualSensorErrorCodeEnumeration::VisualSensorErrorCodeEnum value)
{
	this->visualSensorErrorCode.setValue(value);
}

std::string VisualSensorErrorRecord::getErrorMessage(void)
{
	return this->errorMessage.getValue();
}

void VisualSensorErrorRecord::setErrorMessage(std::string value)
{
	this->errorMessage.setValue(value);
}

int VisualSensorErrorRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(visualSensorErrorCode);
	byteSize += dst->pack(errorMessage);
	return byteSize;
}
int VisualSensorErrorRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(visualSensorErrorCode);
	byteSize += src->unpack(errorMessage);
	return byteSize;
}

int VisualSensorErrorRecord::length(void)
{
	int length = 0;
	length += sensorID.length(); // sensorID
	length += visualSensorErrorCode.length(); // visualSensorErrorCode
	length += errorMessage.length(); // errorMessage
	return length;
}

std::string VisualSensorErrorRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"VisualSensorErrorRecord\">\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << visualSensorErrorCode.toXml(level+1); // visualSensorErrorCode
	oss << errorMessage.toXml(level+1); // errorMessage
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void VisualSensorErrorRecord::copy(VisualSensorErrorRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->visualSensorErrorCode.setName("VisualSensorErrorCode");
	this->visualSensorErrorCode.setOptional(false);
	this->visualSensorErrorCode.setValue(source.getVisualSensorErrorCode()); 
 
	this->errorMessage.setName("ErrorMessage");
	this->errorMessage.setOptional(false);
	this->errorMessage.setValue(source.getErrorMessage());
	this->errorMessage.setSizeType(model::fields::UNSIGNED_BYTE); 
 
}

} // namespace environment
} // namespace openjaus

