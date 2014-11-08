/**
\file QueryRangeSensorDataRecord.h

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
#include "openjaus/environment/Triggers/Fields/QueryRangeSensorDataRecord.h"

namespace openjaus
{
namespace environment
{

QueryRangeSensorDataRecord::QueryRangeSensorDataRecord():
	sensorID(),
	reportCoordinateSystem(),
	queryPresenceVector()
{

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&reportCoordinateSystem);
	reportCoordinateSystem.setName("ReportCoordinateSystem");
	reportCoordinateSystem.setOptional(false);
	// Nothing to init

	fields.push_back(&queryPresenceVector);
	queryPresenceVector.setName("QueryPresenceVector");
	queryPresenceVector.setOptional(false);
	queryPresenceVector.setValue(0);

}

QueryRangeSensorDataRecord::QueryRangeSensorDataRecord(const QueryRangeSensorDataRecord &source)
{
	this->copy(const_cast<QueryRangeSensorDataRecord&>(source));
}

QueryRangeSensorDataRecord::~QueryRangeSensorDataRecord()
{

}


uint16_t QueryRangeSensorDataRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void QueryRangeSensorDataRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

ReportCoordinateSystemRefEnumeration::ReportCoordinateSystemRefEnum QueryRangeSensorDataRecord::getReportCoordinateSystem(void)
{
	return this->reportCoordinateSystem.getValue();
}

void QueryRangeSensorDataRecord::setReportCoordinateSystem(ReportCoordinateSystemRefEnumeration::ReportCoordinateSystemRefEnum value)
{
	this->reportCoordinateSystem.setValue(value);
}

uint16_t QueryRangeSensorDataRecord::getQueryPresenceVector(void)
{
	return this->queryPresenceVector.getValue();
}

void QueryRangeSensorDataRecord::setQueryPresenceVector(uint16_t value)
{
	this->queryPresenceVector.setValue(value);
}

int QueryRangeSensorDataRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(reportCoordinateSystem);
	byteSize += dst->pack(queryPresenceVector);
	return byteSize;
}
int QueryRangeSensorDataRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(reportCoordinateSystem);
	byteSize += src->unpack(queryPresenceVector);
	return byteSize;
}

int QueryRangeSensorDataRecord::length(void)
{
	int length = 0;
	length += sensorID.length(); // sensorID
	length += reportCoordinateSystem.length(); // reportCoordinateSystem
	length += queryPresenceVector.length(); // queryPresenceVector
	return length;
}

std::string QueryRangeSensorDataRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"QueryRangeSensorDataRecord\">\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << reportCoordinateSystem.toXml(level+1); // reportCoordinateSystem
	oss << queryPresenceVector.toXml(level+1); // queryPresenceVector
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void QueryRangeSensorDataRecord::copy(QueryRangeSensorDataRecord& source)
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
 
	this->queryPresenceVector.setName("QueryPresenceVector");
	this->queryPresenceVector.setOptional(false);
	this->queryPresenceVector.setValue(source.getQueryPresenceVector()); 
 
}

} // namespace environment
} // namespace openjaus

