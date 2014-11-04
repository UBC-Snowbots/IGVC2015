/**
\file StillImageDataRecord.h

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
#include "openjaus/environment/Triggers/Fields/StillImageDataRecord.h"

namespace openjaus
{
namespace environment
{

StillImageDataRecord::StillImageDataRecord():
	sensorID(),
	reportCoordinateSystem(),
	timeStamp(),
	imageFrame()
{
	this->presenceVector = 0;

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
	timeStamp.setOptional(true);
	// Nothing

	fields.push_back(&imageFrame);
	imageFrame.setName("ImageFrame");
	imageFrame.setOptional(false);
	// Nothing to init

}

StillImageDataRecord::StillImageDataRecord(const StillImageDataRecord &source)
{
	this->copy(const_cast<StillImageDataRecord&>(source));
}

StillImageDataRecord::~StillImageDataRecord()
{

}


uint16_t StillImageDataRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void StillImageDataRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

ReportCoordinateSystemRefEnumeration::ReportCoordinateSystemRefEnum StillImageDataRecord::getReportCoordinateSystem(void)
{
	return this->reportCoordinateSystem.getValue();
}

void StillImageDataRecord::setReportCoordinateSystem(ReportCoordinateSystemRefEnumeration::ReportCoordinateSystemRefEnum value)
{
	this->reportCoordinateSystem.setValue(value);
}

TimeStampRefBitField& StillImageDataRecord::getTimeStamp(void)
{
	return this->timeStamp;
}

ImageFrameBlob& StillImageDataRecord::getImageFrame(void)
{
	return this->imageFrame;
}

int StillImageDataRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(reportCoordinateSystem);
	if(this->isTimeStampEnabled())
	{
		byteSize += dst->pack(timeStamp);
	}
	byteSize += dst->pack(imageFrame);
	return byteSize;
}
int StillImageDataRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(reportCoordinateSystem);
	if(this->isTimeStampEnabled())
	{
		byteSize += src->unpack(timeStamp);
	}
	byteSize += src->unpack(imageFrame);
	return byteSize;
}

int StillImageDataRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += sensorID.length(); // sensorID
	length += reportCoordinateSystem.length(); // reportCoordinateSystem
	if(this->isTimeStampEnabled())
	{
		length += timeStamp.length(); // timeStamp
	}
	length += imageFrame.length(); // imageFrame
	return length;
}

std::string StillImageDataRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"StillImageDataRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTimeStampEnabled value=\"" << std::boolalpha << this->isTimeStampEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << reportCoordinateSystem.toXml(level+1); // reportCoordinateSystem
	if(this->isTimeStampEnabled())
	{
		oss << timeStamp.toXml(level+1); // timeStamp
	}
	oss << imageFrame.toXml(level+1); // imageFrame
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void StillImageDataRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t StillImageDataRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool StillImageDataRecord::isTimeStampEnabled(void) const
{
	return (this->presenceVector & (0x01 << StillImageDataRecord::TIMESTAMP));
}

void StillImageDataRecord::enableTimeStamp(void)
{
	this->presenceVector |= 0x01 << StillImageDataRecord::TIMESTAMP;
}

void StillImageDataRecord::disableTimeStamp(void)
{
	this->presenceVector &= ~(0x01 << StillImageDataRecord::TIMESTAMP);
}


void StillImageDataRecord::copy(StillImageDataRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->reportCoordinateSystem.setName("ReportCoordinateSystemRef");
	this->reportCoordinateSystem.setOptional(false);
	this->reportCoordinateSystem.setValue(source.getReportCoordinateSystem()); 
 
	this->timeStamp.copy(source.getTimeStamp()); 
 
	this->imageFrame.copy(source.getImageFrame()); 
 
}

} // namespace environment
} // namespace openjaus

