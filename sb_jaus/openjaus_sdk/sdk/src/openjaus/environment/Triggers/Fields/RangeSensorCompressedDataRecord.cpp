/**
\file RangeSensorCompressedDataRecord.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorCompressedDataRecord.h"

namespace openjaus
{
namespace environment
{

RangeSensorCompressedDataRecord::RangeSensorCompressedDataRecord():
	sensorID(),
	reportCoordinateSystem(),
	timeStamp(),
	dataCompression(),
	compressedData()
{

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&reportCoordinateSystem);
	reportCoordinateSystem.setName("ReportCoordinateSystem");
	reportCoordinateSystem.setOptional(false);
	// Nothing to init

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(false);
	// Nothing

	fields.push_back(&dataCompression);
	dataCompression.setName("DataCompression");
	dataCompression.setOptional(false);
	// Nothing to init

	fields.push_back(&compressedData);
	compressedData.setName("CompressedData");
	compressedData.setOptional(false);
	compressedData.setInterpretation("After decompressiong, the expanded data will have the format of a RangeSensorDataPointList as defined in the ReportRangeSensorData message.");
	// Nothing to init

}

RangeSensorCompressedDataRecord::RangeSensorCompressedDataRecord(const RangeSensorCompressedDataRecord &source)
{
	this->copy(const_cast<RangeSensorCompressedDataRecord&>(source));
}

RangeSensorCompressedDataRecord::~RangeSensorCompressedDataRecord()
{

}


uint16_t RangeSensorCompressedDataRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void RangeSensorCompressedDataRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

ReportCoordinateSystemRefEnumeration::ReportCoordinateSystemRefEnum RangeSensorCompressedDataRecord::getReportCoordinateSystem(void)
{
	return this->reportCoordinateSystem.getValue();
}

void RangeSensorCompressedDataRecord::setReportCoordinateSystem(ReportCoordinateSystemRefEnumeration::ReportCoordinateSystemRefEnum value)
{
	this->reportCoordinateSystem.setValue(value);
}

TimeStampRefBitField& RangeSensorCompressedDataRecord::getTimeStamp(void)
{
	return this->timeStamp;
}

DataCompressionRefEnumeration::DataCompressionRefEnum RangeSensorCompressedDataRecord::getDataCompression(void)
{
	return this->dataCompression.getValue();
}

void RangeSensorCompressedDataRecord::setDataCompression(DataCompressionRefEnumeration::DataCompressionRefEnum value)
{
	this->dataCompression.setValue(value);
}

CompressedDataBlob& RangeSensorCompressedDataRecord::getCompressedData(void)
{
	return this->compressedData;
}

int RangeSensorCompressedDataRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(reportCoordinateSystem);
	byteSize += dst->pack(timeStamp);
	byteSize += dst->pack(dataCompression);
	byteSize += dst->pack(compressedData);
	return byteSize;
}
int RangeSensorCompressedDataRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(reportCoordinateSystem);
	byteSize += src->unpack(timeStamp);
	byteSize += src->unpack(dataCompression);
	byteSize += src->unpack(compressedData);
	return byteSize;
}

int RangeSensorCompressedDataRecord::length(void)
{
	int length = 0;
	length += sensorID.length(); // sensorID
	length += reportCoordinateSystem.length(); // reportCoordinateSystem
	length += timeStamp.length(); // timeStamp
	length += dataCompression.length(); // dataCompression
	length += compressedData.length(); // compressedData
	return length;
}

std::string RangeSensorCompressedDataRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RangeSensorCompressedDataRecord\">\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << reportCoordinateSystem.toXml(level+1); // reportCoordinateSystem
	oss << timeStamp.toXml(level+1); // timeStamp
	oss << dataCompression.toXml(level+1); // dataCompression
	oss << compressedData.toXml(level+1); // compressedData
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void RangeSensorCompressedDataRecord::copy(RangeSensorCompressedDataRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->reportCoordinateSystem.setName("ReportCoordinateSystemRef");
	this->reportCoordinateSystem.setOptional(false);
	this->reportCoordinateSystem.setValue(source.getReportCoordinateSystem()); 
 
	this->timeStamp.copy(source.getTimeStamp()); 
 
	this->dataCompression.setName("DataCompressionRef");
	this->dataCompression.setOptional(false);
	this->dataCompression.setValue(source.getDataCompression()); 
 
	this->compressedData.copy(source.getCompressedData()); 
 
}

} // namespace environment
} // namespace openjaus

