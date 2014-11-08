
/**
\file ReportRangeSensorCompressedData.h

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
#include "openjaus/environment/Triggers/ReportRangeSensorCompressedData.h"

namespace openjaus
{
namespace environment
{

ReportRangeSensorCompressedData::ReportRangeSensorCompressedData() : 
	model::Message(),
	rangeSensorCompressedDataList()
{
	this->id = ReportRangeSensorCompressedData::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&rangeSensorCompressedDataList);
	rangeSensorCompressedDataList.setName("RangeSensorCompressedDataList");
	rangeSensorCompressedDataList.setOptional(false);
	// Nothing to Init

}

ReportRangeSensorCompressedData::ReportRangeSensorCompressedData(model::Message *message) :
	model::Message(message),
	rangeSensorCompressedDataList()
{
	this->id = ReportRangeSensorCompressedData::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&rangeSensorCompressedDataList);
	rangeSensorCompressedDataList.setName("RangeSensorCompressedDataList");
	rangeSensorCompressedDataList.setOptional(false);
	// Nothing to Init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportRangeSensorCompressedData::~ReportRangeSensorCompressedData()
{

}


RangeSensorCompressedDataList& ReportRangeSensorCompressedData::getRangeSensorCompressedDataList(void)
{
	return this->rangeSensorCompressedDataList;
}

int ReportRangeSensorCompressedData::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(rangeSensorCompressedDataList);
	return byteSize;
}

int ReportRangeSensorCompressedData::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(rangeSensorCompressedDataList);
	return byteSize;
}

int ReportRangeSensorCompressedData::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += rangeSensorCompressedDataList.length(); // rangeSensorCompressedDataList
	return length;
}

std::string ReportRangeSensorCompressedData::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportRangeSensorCompressedData\"";
	oss << " id=\"0x4804\" >\n";
	oss << rangeSensorCompressedDataList.toXml(level+1); // rangeSensorCompressedDataList
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

