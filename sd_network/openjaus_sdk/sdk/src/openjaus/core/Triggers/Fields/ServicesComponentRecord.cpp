/**
\file ServicesComponentRecord.h

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
#include "openjaus/core/Triggers/Fields/ServicesComponentRecord.h"

namespace openjaus
{
namespace core
{

ServicesComponentRecord::ServicesComponentRecord():
	componentID(),
	instanceID(),
	servicesServiceList()
{

	fields.push_back(&componentID);
	componentID.setName("ComponentID");
	componentID.setOptional(false);
	componentID.setInterpretation("Comp ID.  For Single Component reporting this field shall contain the Component ID as specified in the Destination Address of the Query Configuration message and shall be the only Component reported");
	componentID.setValue(0);

	fields.push_back(&instanceID);
	instanceID.setName("InstanceID");
	instanceID.setOptional(false);
	instanceID.setInterpretation("Inst ID when legacy Components are reported; a value of zero (0) shall be used for non-legacy components.");
	instanceID.setValue(0);

	fields.push_back(&servicesServiceList);
	servicesServiceList.setName("ServicesServiceList");
	servicesServiceList.setOptional(false);
	// Nothing to Init

}

ServicesComponentRecord::ServicesComponentRecord(const ServicesComponentRecord &source)
{
	this->copy(const_cast<ServicesComponentRecord&>(source));
}

ServicesComponentRecord::~ServicesComponentRecord()
{

}


uint8_t ServicesComponentRecord::getComponentID(void)
{
	return this->componentID.getValue();
}

void ServicesComponentRecord::setComponentID(uint8_t value)
{
	this->componentID.setValue(value);
}

uint8_t ServicesComponentRecord::getInstanceID(void)
{
	return this->instanceID.getValue();
}

void ServicesComponentRecord::setInstanceID(uint8_t value)
{
	this->instanceID.setValue(value);
}

ServicesServiceList& ServicesComponentRecord::getServicesServiceList(void)
{
	return this->servicesServiceList;
}

int ServicesComponentRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(componentID);
	byteSize += dst->pack(instanceID);
	byteSize += dst->pack(servicesServiceList);
	return byteSize;
}
int ServicesComponentRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(componentID);
	byteSize += src->unpack(instanceID);
	byteSize += src->unpack(servicesServiceList);
	return byteSize;
}

int ServicesComponentRecord::length(void)
{
	int length = 0;
	length += componentID.length(); // componentID
	length += instanceID.length(); // instanceID
	length += servicesServiceList.length(); // servicesServiceList
	return length;
}

std::string ServicesComponentRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ServicesComponentRecord\">\n";
	oss << componentID.toXml(level+1); // componentID
	oss << instanceID.toXml(level+1); // instanceID
	oss << servicesServiceList.toXml(level+1); // servicesServiceList
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ServicesComponentRecord::copy(ServicesComponentRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->componentID.setName("ComponentID");
	this->componentID.setOptional(false);
	this->componentID.setInterpretation("Comp ID.  For Single Component reporting this field shall contain the Component ID as specified in the Destination Address of the Query Configuration message and shall be the only Component reported");
	this->componentID.setValue(source.getComponentID()); 
 
	this->instanceID.setName("InstanceID");
	this->instanceID.setOptional(false);
	this->instanceID.setInterpretation("Inst ID when legacy Components are reported; a value of zero (0) shall be used for non-legacy components.");
	this->instanceID.setValue(source.getInstanceID()); 
 
	this->servicesServiceList.copy(source.getServicesServiceList()); 
 
}

} // namespace core
} // namespace openjaus

