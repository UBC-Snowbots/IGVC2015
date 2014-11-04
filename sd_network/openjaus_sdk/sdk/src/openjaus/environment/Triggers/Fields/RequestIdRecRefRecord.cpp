/**
\file RequestIdRecRefRecord.h

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
#include "openjaus/environment/Triggers/Fields/RequestIdRecRefRecord.h"

namespace openjaus
{
namespace environment
{

RequestIdRecRefRecord::RequestIdRecRefRecord():
	requestID()
{

	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setValue(0);

}

RequestIdRecRefRecord::RequestIdRecRefRecord(const RequestIdRecRefRecord &source)
{
	this->copy(const_cast<RequestIdRecRefRecord&>(source));
}

RequestIdRecRefRecord::~RequestIdRecRefRecord()
{

}


uint8_t RequestIdRecRefRecord::getRequestID(void)
{
	return this->requestID.getValue();
}

void RequestIdRecRefRecord::setRequestID(uint8_t value)
{
	this->requestID.setValue(value);
}

int RequestIdRecRefRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(requestID);
	return byteSize;
}
int RequestIdRecRefRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(requestID);
	return byteSize;
}

int RequestIdRecRefRecord::length(void)
{
	int length = 0;
	length += requestID.length(); // requestID
	return length;
}

std::string RequestIdRecRefRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RequestIdRecRefRecord\">\n";
	oss << requestID.toXml(level+1); // requestID
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void RequestIdRecRefRecord::copy(RequestIdRecRefRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->requestID.setName("RequestID");
	this->requestID.setOptional(false);
	this->requestID.setValue(source.getRequestID());
}

} // namespace environment
} // namespace openjaus

