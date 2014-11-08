/**
\file ServiceInformationRecord.h

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
#include "openjaus/core/Triggers/Fields/ServiceInformationRecord.h"

namespace openjaus
{
namespace core
{

ServiceInformationRecord::ServiceInformationRecord():
	uri(),
	majorVersionNumber(),
	minorVersionNumber()
{

	fields.push_back(&uri);
	uri.setName("Uri");
	uri.setOptional(false);
	uri.setInterpretation("Service URI");
	uri.setSizeType(model::fields::UNSIGNED_BYTE);

	fields.push_back(&majorVersionNumber);
	majorVersionNumber.setName("MajorVersionNumber");
	majorVersionNumber.setOptional(false);
	majorVersionNumber.setInterpretation("Major version number");
	majorVersionNumber.setValue(0);

	fields.push_back(&minorVersionNumber);
	minorVersionNumber.setName("MinorVersionNumber");
	minorVersionNumber.setOptional(false);
	minorVersionNumber.setInterpretation("Minor version number");
	minorVersionNumber.setValue(0);

}

ServiceInformationRecord::ServiceInformationRecord(const ServiceInformationRecord &source)
{
	this->copy(const_cast<ServiceInformationRecord&>(source));
}

ServiceInformationRecord::~ServiceInformationRecord()
{

}


std::string ServiceInformationRecord::getUri(void)
{
	return this->uri.getValue();
}

void ServiceInformationRecord::setUri(std::string value)
{
	this->uri.setValue(value);
}

uint8_t ServiceInformationRecord::getMajorVersionNumber(void)
{
	return this->majorVersionNumber.getValue();
}

void ServiceInformationRecord::setMajorVersionNumber(uint8_t value)
{
	this->majorVersionNumber.setValue(value);
}

uint8_t ServiceInformationRecord::getMinorVersionNumber(void)
{
	return this->minorVersionNumber.getValue();
}

void ServiceInformationRecord::setMinorVersionNumber(uint8_t value)
{
	this->minorVersionNumber.setValue(value);
}

int ServiceInformationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(uri);
	byteSize += dst->pack(majorVersionNumber);
	byteSize += dst->pack(minorVersionNumber);
	return byteSize;
}
int ServiceInformationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(uri);
	byteSize += src->unpack(majorVersionNumber);
	byteSize += src->unpack(minorVersionNumber);
	return byteSize;
}

int ServiceInformationRecord::length(void)
{
	int length = 0;
	length += uri.length(); // uri
	length += majorVersionNumber.length(); // majorVersionNumber
	length += minorVersionNumber.length(); // minorVersionNumber
	return length;
}

std::string ServiceInformationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ServiceInformationRecord\">\n";
	oss << uri.toXml(level+1); // uri
	oss << majorVersionNumber.toXml(level+1); // majorVersionNumber
	oss << minorVersionNumber.toXml(level+1); // minorVersionNumber
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ServiceInformationRecord::copy(ServiceInformationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->uri.setName("Uri");
	this->uri.setOptional(false);
	this->uri.setInterpretation("Service URI");
	this->uri.setValue(source.getUri());
	this->uri.setSizeType(model::fields::UNSIGNED_BYTE); 
 
	this->majorVersionNumber.setName("MajorVersionNumber");
	this->majorVersionNumber.setOptional(false);
	this->majorVersionNumber.setInterpretation("Major version number");
	this->majorVersionNumber.setValue(source.getMajorVersionNumber()); 
 
	this->minorVersionNumber.setName("MinorVersionNumber");
	this->minorVersionNumber.setOptional(false);
	this->minorVersionNumber.setInterpretation("Minor version number");
	this->minorVersionNumber.setValue(source.getMinorVersionNumber()); 
 
}

} // namespace core
} // namespace openjaus

